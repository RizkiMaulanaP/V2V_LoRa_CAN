// ignore_for_file: unused_import, unused_element

import 'dart:async';
import 'dart:convert';

import '../domain/models/v2v_frame.dart';
import 'data_source.dart';

// ============================================================
// 🔥 SERIAL DATA SOURCE (PRODUCTION — STUB)
//    Baca frame V2V dari Raspberry Pi via USB CDC (serial).
//
//    ⚠️ BELUM DIIMPLEMENTASI. Aktifkan saat:
//       1. Hardware Pi sudah ready
//       2. Data contract (JSON format) sudah disepakati tim
//       3. Package serial sudah di-add ke pubspec.yaml
//
//    Saat ini panggilan ke source ini akan throw UnimplementedError.
// ============================================================
//
// 📋 LANGKAH IMPLEMENTASI NANTI:
//
// 1. Tambah dependency di pubspec.yaml:
//      flutter_libserialport: ^0.4.0   (Flutter Linux untuk Pi)
//      atau
//      usb_serial: ^0.5.0              (kalau target Android head unit)
//
// 2. Cari port yang Pi exposed. Biasanya di Linux: /dev/ttyACM0 atau
//    /dev/ttyUSB0. Cek dengan `ls /dev/tty*` saat Pi connected.
//
// 3. Baud rate: sesuai konfig MCU di Pi (umumnya 115200).
//
// 4. Format frame (sesuai data contract):
//      Setiap baris JSON diakhiri '\n':
//      {"ts":1748534400123,"ego":{...},"neighbors":[...]}\n
//
// 5. Implementasi method `stream()`:
//      - Buka port serial
//      - Listen ke port.read stream
//      - Buffer bytes, split di '\n'
//      - Parse tiap line dengan jsonDecode → V2VFrame
//      - yield V2VFrame
//
// 6. Implementasi `dispose()`:
//      - port.close()
//      - cancel subscription
//
// ============================================================

class SerialDataSource implements DataSource {
  /// Path ke serial device. Override saat construct.
  /// Default: /dev/ttyACM0 (USB CDC standard di Linux).
  final String portPath;

  /// Baud rate. Harus match dengan MCU/Pi side.
  final int baudRate;

  SerialDataSource({
    this.portPath = '/dev/ttyACM0',
    this.baudRate = 115200,
  });

  @override
  Stream<V2VFrame> stream() async* {
    throw UnimplementedError(
      'SerialDataSource belum diimplementasi. '
      'Lihat TODO di file untuk langkah implementasinya. '
      'Sementara pakai MockDataSource() di main.dart.',
    );

    // ============================================================
    // PSEUDOCODE — implementasi nanti
    // ============================================================
    //
    // final port = SerialPort(portPath)..openReadWrite();
    // port.config = SerialPortConfig()
    //   ..baudRate = baudRate
    //   ..bits = 8
    //   ..parity = SerialPortParity.none
    //   ..stopBits = 1;
    //
    // final reader = SerialPortReader(port);
    // final buffer = StringBuffer();
    //
    // await for (final chunk in reader.stream) {
    //   buffer.write(utf8.decode(chunk, allowMalformed: true));
    //   final text = buffer.toString();
    //   final lines = text.split('\n');
    //
    //   // Sisakan baris terakhir (mungkin belum lengkap)
    //   buffer.clear();
    //   buffer.write(lines.removeLast());
    //
    //   for (final line in lines) {
    //     if (line.trim().isEmpty) continue;
    //     try {
    //       final json = jsonDecode(line) as Map<String, dynamic>;
    //       yield _parseFrame(json);
    //     } catch (e) {
    //       // log & skip frame rusak
    //       continue;
    //     }
    //   }
    // }
  }

  @override
  Future<void> dispose() async {
    // TODO: port.close(), reader.dispose()
  }

  /// Helper parser — sesuai data contract:
  ///   { "ts": ..., "ego": {...}, "neighbors": [...] }
  V2VFrame _parseFrame(Map<String, dynamic> json) {
    final ego = json['ego'] as Map<String, dynamic>;
    final neighbors = (json['neighbors'] as List? ?? const [])
        .cast<Map<String, dynamic>>();

    return V2VFrame(
      timestamp: (json['ts'] as num?)?.toInt() ??
          DateTime.now().millisecondsSinceEpoch,
      ego: EgoState(
        lat: (ego['lat'] as num).toDouble(),
        lon: (ego['lon'] as num).toDouble(),
        speedKmh: (ego['speed_kmh'] as num).toDouble(),
        headingDeg: (ego['heading_deg'] as num?)?.toDouble() ?? 0,
        engineRpm: (ego['engine_rpm'] as num?)?.toDouble() ?? 0,
        engineTempC: (ego['engine_temp_c'] as num?)?.toDouble() ?? 0,
        fuelLevelPct: (ego['fuel_level_pct'] as num?)?.toDouble() ?? 0,
      ),
      neighbors: neighbors.map((n) {
        return NeighborState(
          id: n['id'] as String,
          lat: (n['lat'] as num).toDouble(),
          lon: (n['lon'] as num).toDouble(),
          speedKmh: (n['speed_kmh'] as num).toDouble(),
          headingDeg: (n['heading_deg'] as num?)?.toDouble() ?? 0,
          emergencyStatus: _parseStatus(n['emergency_status'] as String?),
        );
      }).toList(),
    );
  }

  static EmergencyStatus _parseStatus(String? s) {
    switch (s?.toUpperCase()) {
      case 'EMERGENCY':
        return EmergencyStatus.emergency;
      case 'WARNING':
        return EmergencyStatus.warning;
      default:
        return EmergencyStatus.normal;
    }
  }
}

