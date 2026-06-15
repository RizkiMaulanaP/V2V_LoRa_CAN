import 'dart:convert';
import 'package:flutter/services.dart';
import '../models/vehicle_state.dart';

Future<List<VehicleState>> loadCSV() async {
  final raw = await rootBundle.loadString('assets/processed.csv');
  final lines = const LineSplitter().convert(raw);

  if (lines.isEmpty) return [];

  final headers = lines.first.split(',');

  List<VehicleState> vehicles = [];

  for (int i = 1; i < lines.length; i++) {
    final row = lines[i].split(',');
    if (row.length != headers.length) continue;

    final data = <String, dynamic>{
      for (int j = 0; j < headers.length; j++)
        headers[j].trim(): row[j].trim(),
    };

    /// 🔥 DEBUG (biar yakin kebaca)
    if (i < 5) {
    }

    final lat = double.tryParse(data['lat'] ?? '');
    final lon = double.tryParse(data['lon'] ?? '');

    /// ❌ skip kalau invalid
    if (lat == null || lon == null) continue;

    vehicles.add(
      VehicleState(
        id: "ego",
        lat: lat,
        lng: lon, // 🔥 tetap pakai lng di model
        speed: double.tryParse(data['speed_kmh'] ?? '') ?? 0,
        heading: double.tryParse(data['yaw_deg'] ?? '') ?? 0,
        x: double.tryParse(data['pos_east_m'] ?? '') ?? 0,
        y: double.tryParse(data['pos_north_m'] ?? '') ?? 0,
      ),
    );
  }

  return vehicles;
}