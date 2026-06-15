import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';

import '../domain/models/v2v_frame.dart';
import 'data_source.dart';

// ============================================================
// 🔥 TCP DATA SOURCE (PRODUCTION)
//    Baca frame V2V dari host UKF (python-app) via TCP socket.
//
//    Host: main_beamng.py  (BeamNG sim)   → default port 8765
//          main_ukf.py --json-tcp 8765    (hardware MCU)
//
//    Wire format: newline-delimited JSON, 1 object per baris:
//      {"ts":...,"ego":{...},"neighbors":[...]}\n
//    (lihat python-app/v2v_json.py — kontrak ini identik dengan
//     V2VFrame.fromJson di domain/models/v2v_frame.dart)
//
//    Auto-reconnect: kalau koneksi putus, retry tiap `reconnectDelay`.
// ============================================================
class TcpDataSource implements DataSource {
  /// IP host yang menjalankan python-app. 'localhost' kalau 1 mesin,
  /// atau IP Raspberry Pi / laptop sim di jaringan yang sama.
  final String host;

  /// Port JSON server (default cocok dengan main_beamng.py / --json-tcp).
  final int port;

  /// Jeda sebelum mencoba reconnect setelah koneksi putus.
  final Duration reconnectDelay;

  TcpDataSource({
    this.host = 'localhost',
    this.port = 8765,
    this.reconnectDelay = const Duration(seconds: 2),
  });

  Socket? _socket;
  bool _disposed = false;

  @override
  Stream<V2VFrame> stream() async* {
    while (!_disposed) {
      Socket socket;
      try {
        socket = await Socket.connect(host, port,
            timeout: const Duration(seconds: 5));
        _socket = socket;
        debugPrint('[TCP] connected to $host:$port');
      } catch (e) {
        debugPrint('[TCP] connect failed ($e) — retry in '
            '${reconnectDelay.inSeconds}s');
        await Future.delayed(reconnectDelay);
        continue; // retry the connect loop
      }

      // Decode bytes → text → split per baris. utf8.decoder + LineSplitter
      // menangani frame yang ke-split antar paket TCP.
      final lines = socket
          .cast<List<int>>()
          .transform(utf8.decoder)
          .transform(const LineSplitter());

      try {
        await for (final line in lines) {
          if (_disposed) break;
          final trimmed = line.trim();
          if (trimmed.isEmpty) continue;
          try {
            final json = jsonDecode(trimmed) as Map<String, dynamic>;
            yield V2VFrame.fromJson(json);
          } catch (e) {
            debugPrint('[TCP] skip bad frame: $e');
          }
        }
      } catch (e) {
        debugPrint('[TCP] stream error: $e');
      }

      // Sampai sini = koneksi tertutup oleh host. Reconnect kecuali disposed.
      _socket?.destroy();
      _socket = null;
      if (_disposed) break;
      debugPrint('[TCP] disconnected — reconnect in '
          '${reconnectDelay.inSeconds}s');
      await Future.delayed(reconnectDelay);
    }
  }

  @override
  Future<void> dispose() async {
    _disposed = true;
    _socket?.destroy();
    _socket = null;
  }
}
