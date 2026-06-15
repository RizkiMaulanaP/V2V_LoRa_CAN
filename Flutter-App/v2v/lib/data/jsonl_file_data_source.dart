import 'dart:async';
import 'dart:convert';

import 'package:flutter/services.dart';

import '../domain/models/v2v_frame.dart';
import 'data_source.dart';

// ============================================================
// 🔥 JSONL FILE DATA SOURCE
//    Playback V2VFrame dari file recording hasil sensor test.
//
//    Format file: JSONL (JSON Lines) — satu V2VFrame per baris.
//    Schema persis sama dengan wire format yang Pi kirim live.
//
//    Cara pakai:
//      final source = JsonlFileDataSource('assets/recordings/test_1.jsonl');
//      source.stream().listen((frame) { ... });
//
//    Playback otomatis pakai timestamp di tiap frame, jadi
//    realistis sesuai recording asli.
// ============================================================
class JsonlFileDataSource implements DataSource {
  /// Path file di assets/ (relatif ke project root).
  /// Pastikan path-nya juga di-register di pubspec.yaml > flutter > assets.
  final String assetPath;

  /// 1.0 = realtime sesuai timestamp recording.
  /// 2.0 = playback 2x lebih cepat (untuk demo cepat).
  /// 0.5 = setengah kecepatan (untuk debug).
  final double playbackSpeed;

  /// Kalau true, akan loop ulang saat sampai akhir file.
  final bool loop;

  JsonlFileDataSource(
    this.assetPath, {
    this.playbackSpeed = 1.0,
    this.loop = true,
  });

  bool _disposed = false;

  @override
  Stream<V2VFrame> stream() async* {
    // Load file dari assets
    final raw = await rootBundle.loadString(assetPath);
    final lines = const LineSplitter()
        .convert(raw)
        .where((l) => l.trim().isNotEmpty)
        .toList();

    if (lines.isEmpty) {
      yield V2VFrame(
        timestamp: DateTime.now().millisecondsSinceEpoch,
        ego: EgoState.empty,
        neighbors: const [],
      );
      return;
    }

    // Parse semua frame dulu (file recording biasanya kecil, <50MB)
    final frames = <V2VFrame>[];
    for (final line in lines) {
      try {
        final json = jsonDecode(line) as Map<String, dynamic>;
        frames.add(_parseFrame(json));
      } catch (e) {
        // Skip line yang rusak, jangan crash
        continue;
      }
    }

    if (frames.isEmpty) {
      yield V2VFrame(
        timestamp: DateTime.now().millisecondsSinceEpoch,
        ego: EgoState.empty,
        neighbors: const [],
      );
      return;
    }

    while (!_disposed) {
      final clock = Stopwatch()..start();
      final startTs = frames.first.timestamp;

      for (int i = 0; i < frames.length && !_disposed; i++) {
        final frame = frames[i];

        // Hitung kapan frame ini harus tampil
        final targetElapsedMs =
            (frame.timestamp - startTs) / playbackSpeed;

        // Tunggu sampai waktunya
        final waitMs = targetElapsedMs - clock.elapsedMilliseconds;
        if (waitMs > 0) {
          await Future.delayed(Duration(milliseconds: waitMs.round()));
        }

        if (_disposed) return;

        yield frame;
      }

      if (!loop) break;
    }
  }

  @override
  Future<void> dispose() async {
    _disposed = true;
  }

  // ============================================================
  // Parser (schema match dengan SerialDataSource)
  // ============================================================
  // Same contract as the live host — delegate so gps/warning fields are parsed.
  V2VFrame _parseFrame(Map<String, dynamic> json) => V2VFrame.fromJson(json);
}
