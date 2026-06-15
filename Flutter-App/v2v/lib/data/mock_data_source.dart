import 'dart:async';
import 'dart:math';

import 'package:latlong2/latlong.dart';

import '../domain/models/v2v_frame.dart';
import '../domain/models/vehicle_state.dart';
import '../domain/services/csv_loader.dart';
import 'data_source.dart';

// ============================================================
// 🔥 MOCK DATA SOURCE
//    Generate V2VFrame untuk testing UI tanpa hardware Pi.
//
//    Ego  : pakai CSV genba (speed, lat, lon, heading dll).
//    Engine RPM/Temp: derived dari speed (placeholder).
//    Neighbors: scripted scenario — 1 mobil approach dari
//    LEFT, lalu RIGHT, dengan emergency_status cycling.
//
//    Neighbor posisi di-set body-frame relatif ke ego, lalu
//    di-convert ke lat/lon (supaya format data sama dengan
//    yang akan dikirim Pi dari Neighbour Track).
// ============================================================
class MockDataSource implements DataSource {
  // ---- Konfigurasi playback ----
  static const int _csvIntervalMs = 10;
  static const int _renderIntervalMs = 16;
  static const double _playbackSpeed = 1.0;

  // ---- Konfigurasi scenario neighbor ----
  static const double _scenarioPeriodSec = 25.0;

  // ---- Konstanta geo ----
  static const double _earthR = 6371000.0;

  // ---- Internal state ----
  List<VehicleState> _csv = [];
  bool _disposed = false;

  @override
  Stream<V2VFrame> stream() async* {
    _csv = await loadCSV();

    if (_csv.length < 2) {
      yield V2VFrame(
        timestamp: DateTime.now().millisecondsSinceEpoch,
        ego: EgoState.empty,
        neighbors: const [],
      );
      return;
    }

    final clock = Stopwatch()..start();

    while (!_disposed) {
      final elapsedMs = clock.elapsedMilliseconds * _playbackSpeed;
      final ego = _computeEgoAt(elapsedMs);
      final neighbors = _generateNeighbors(elapsedMs / 1000.0, ego);

      yield V2VFrame(
        timestamp: DateTime.now().millisecondsSinceEpoch,
        ego: ego,
        neighbors: neighbors,
      );

      await Future.delayed(const Duration(milliseconds: _renderIntervalMs));
    }
  }

  @override
  Future<void> dispose() async {
    _disposed = true;
  }

  // ============================================================
  // EGO STATE dari CSV (dengan interpolasi smooth)
  // ============================================================
  EgoState _computeEgoAt(double elapsedMs) {
    final indexFloat = elapsedMs / _csvIntervalMs;
    int index = indexFloat.floor();

    if (index >= _csv.length - 1) {
      index = (_csv.length - 2);
    }
    final t = (indexFloat - index).clamp(0.0, 1.0);

    final cur = _csv[index];
    final next = _csv[index + 1];

    final speed = _lerp(cur.speed, next.speed, t);

    final lat = _lerp(cur.lat, next.lat, t);
    final lon = _lerp(cur.lng, next.lng, t);

    final heading = _bearing(
      LatLng(cur.lat, cur.lng),
      LatLng(next.lat, next.lng),
    );

    // Fuel: mulai 78%, turun perlahan seiring waktu (placeholder mock)
    final fuelPct = (78 - (elapsedMs / 1000 / 60) * 0.5).clamp(0.0, 100.0);

    return EgoState(
      lat: lat,
      lon: lon,
      speedKmh: speed,
      headingDeg: heading,
      engineRpm: (speed * 45).clamp(800, 8000),
      engineTempC: 85 + (speed / 160) * 15,
      fuelLevelPct: fuelPct,
    );
  }

  // ============================================================
  // NEIGHBORS — scripted scenario (body-frame → lat/lon)
  // ============================================================
  // Phase    Time(s)   Posisi body-frame   Emergency status
  // ────────────────────────────────────────────────────────
  //   1      0  - 8    LEFT, 50m → 5m      NORMAL → EMERGENCY
  //   2      8  - 12   far (off-screen)    NORMAL
  //   3      12 - 20   RIGHT, 50m → 5m     NORMAL → EMERGENCY
  //   4      20 - 25   far (off-screen)    NORMAL
  // ============================================================
  List<NeighborState> _generateNeighbors(double tSec, EgoState ego) {
    final t = tSec % _scenarioPeriodSec;

    NeighborState b01;

    if (t < 8) {
      final progress = t / 8.0;
      final distance = 50 - (progress * 45);
      b01 = _placeNeighborInBodyFrame(
        id: 'B01',
        ego: ego,
        dxBody: -distance, // negatif = kiri
        dyBody: 0,
        status: distance < 10
            ? EmergencyStatus.emergency
            : EmergencyStatus.normal,
      );
    } else if (t < 12) {
      b01 = _placeNeighborInBodyFrame(
        id: 'B01',
        ego: ego,
        dxBody: -80,
        dyBody: 30,
        status: EmergencyStatus.normal,
      );
    } else if (t < 20) {
      final progress = (t - 12) / 8.0;
      final distance = 50 - (progress * 45);
      b01 = _placeNeighborInBodyFrame(
        id: 'B01',
        ego: ego,
        dxBody: distance, // positif = kanan
        dyBody: 0,
        status: distance < 10
            ? EmergencyStatus.emergency
            : EmergencyStatus.normal,
      );
    } else {
      b01 = _placeNeighborInBodyFrame(
        id: 'B01',
        ego: ego,
        dxBody: 80,
        dyBody: 30,
        status: EmergencyStatus.normal,
      );
    }

    return [b01];
  }

  /// Tempatkan neighbor dengan offset BODY-FRAME (dxBody right, dyBody forward),
  /// lalu convert ke world lat/lon supaya format sesuai output Pi.
  NeighborState _placeNeighborInBodyFrame({
    required String id,
    required EgoState ego,
    required double dxBody,
    required double dyBody,
    required EmergencyStatus status,
  }) {
    // 1) Body frame → world ENU (rotasi balik pakai heading ego)
    final h = ego.headingDeg * pi / 180;
    final dxEast = dxBody * cos(h) + dyBody * sin(h);
    final dyNorth = -dxBody * sin(h) + dyBody * cos(h);

    // 2) ENU offset (meter) → lat/lon offset
    final egoLatRad = ego.lat * pi / 180;
    final dLat = dyNorth / _earthR * 180 / pi;
    final dLon = dxEast / (_earthR * cos(egoLatRad)) * 180 / pi;

    return NeighborState(
      id: id,
      lat: ego.lat + dLat,
      lon: ego.lon + dLon,
      speedKmh: 40,
      headingDeg: 0,
      emergencyStatus: status,
    );
  }

  // ============================================================
  // UTILS
  // ============================================================
  static double _lerp(double a, double b, double t) => a + (b - a) * t;

  static double _bearing(LatLng start, LatLng end) {
    final dLon = (end.longitude - start.longitude) * pi / 180;
    final lat1 = start.latitude * pi / 180;
    final lat2 = end.latitude * pi / 180;
    final y = sin(dLon) * cos(lat2);
    final x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    return (atan2(y, x) * 180 / pi + 360) % 360;
  }
}
