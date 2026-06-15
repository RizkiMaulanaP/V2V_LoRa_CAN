import '../domain/models/v2v_frame.dart';

// ============================================================
// 🔥 DATA SOURCE INTERFACE
//    Abstraksi sumber data untuk Flutter UI.
//
//    Implementasi yang ada:
//      • MockDataSource    — testing, baca CSV + fake neighbors
//      • SerialDataSource  — production, baca USB CDC dari Pi
//
//    UI tidak perlu tahu mana sumbernya. Tinggal swap di main.dart.
// ============================================================
abstract class DataSource {
  /// Stream snapshot V2V dari sumber data.
  /// Idealnya emit pada ~10-60Hz tergantung implementasi.
  Stream<V2VFrame> stream();

  /// Cleanup (tutup port, cancel timer, dll).
  Future<void> dispose();
}
