// ============================================================
// 🔥 V2V FRAME MODEL
//    Bentuk data yang mengalir dari Pi ke Flutter UI.
//
//    Mengikuti diagram arsitektur:
//
//      [Sensor] ─► [MCU: pack frame] ─► [Pi: UKF + Neighbour Track]
//                                            │
//                            output (lat/lon, speed, heading):
//                              • Ego:        dari UKF
//                              • Neighbors:  dari Neighbour Track
//                                            │
//                                      ▼ JSON @ ~10Hz
//                                 [Flutter UI]
//
//    Format koordinat: GEOGRAPHIC (lat/lon WGS84), bukan ENU/x-y.
//    Flutter yang lakukan konversi lat/lon → distance & arah relatif.
// ============================================================

/// Status emergency yang di-broadcast tiap kendaraan via LoRA.
enum EmergencyStatus {
  normal,    // NORMAL — kondisi aman
  warning,   // WARNING — perlu hati-hati (misal: pengereman)
  emergency, // EMERGENCY — bahaya (misal: kecelakaan, mogok)
}

/// State mobil ego (kendaraan kita sendiri).
/// Output dari Unscented Kalman Filter di Pi.
class EgoState {
  /// Posisi geografis (lat/lon WGS84).
  final double lat;
  final double lon;

  /// Speed dari OBD (km/h).
  final double speedKmh;

  /// Heading (derajat, 0=Utara, 90=Timur, clockwise).
  /// Penting untuk hitung arah RELATIF saat menentukan warning LEFT/RIGHT/FRONT/REAR.
  final double headingDeg;

  /// Engine RPM dari OBD PID 0x0C.
  final double engineRpm;

  /// Engine temperature dari OBD PID 0x05 (Celsius).
  final double engineTempC;

  /// Fuel level dari OBD PID 0x2F. Range 0-100 (persen).
  /// 0 = tangki kosong, 100 = tangki penuh.
  final double fuelLevelPct;

  /// GPS health — untuk indikator & GPS-lost warning.
  final bool fixValid;     // true kalau GPS punya fix valid
  final double hdop;       // horizontal dilution of precision (kecil = akurat)
  final int satellites;    // jumlah satelit terkunci

  const EgoState({
    required this.lat,
    required this.lon,
    required this.speedKmh,
    required this.headingDeg,
    required this.engineRpm,
    required this.engineTempC,
    required this.fuelLevelPct,
    this.fixValid = true,
    this.hdop = 0,
    this.satellites = 0,
  });

  /// True bila posisi ego tidak bisa dipercaya (no fix / akurasi rendah).
  bool get gpsLost => !fixValid || hdop > 5.0 || satellites < 4;

  /// Empty state — placeholder saat data belum masuk.
  static const EgoState empty = EgoState(
    lat: 0,
    lon: 0,
    speedKmh: 0,
    headingDeg: 0,
    engineRpm: 0,
    engineTempC: 0,
    fuelLevelPct: 0,
    fixValid: false,
  );

  /// Parse dari kontrak JSON host (python-app/v2v_json.py).
  factory EgoState.fromJson(Map<String, dynamic> j) => EgoState(
        lat: (j['lat'] as num).toDouble(),
        lon: (j['lon'] as num).toDouble(),
        speedKmh: (j['speed_kmh'] as num?)?.toDouble() ?? 0,
        headingDeg: (j['heading_deg'] as num?)?.toDouble() ?? 0,
        engineRpm: (j['engine_rpm'] as num?)?.toDouble() ?? 0,
        engineTempC: (j['engine_temp_c'] as num?)?.toDouble() ?? 0,
        fuelLevelPct: (j['fuel_level_pct'] as num?)?.toDouble() ?? 0,
        fixValid: (j['fix_valid'] as num?)?.toInt() != 0,
        hdop: (j['hdop'] as num?)?.toDouble() ?? 0,
        satellites: (j['satellites'] as num?)?.toInt() ?? 0,
      );
}

/// State mobil lain (neighbor) — output dari Neighbour Track di Pi.
/// Posisi datang sebagai LoRA broadcast lalu di-track/propagate.
class NeighborState {
  /// ID unik mobil lain (misal MAC LoRA atau VIN).
  final String id;

  /// Posisi geografis dari broadcast LoRA.
  final double lat;
  final double lon;

  /// Speed (km/h) dari neighbor.
  final double speedKmh;

  /// Heading (derajat, 0=Utara).
  final double headingDeg;

  /// Status emergency yang DIDEKLARASIKAN sendiri oleh mobil itu via LoRA.
  /// Beda dengan warning UI yang dihitung ego dari jarak.
  final EmergencyStatus emergencyStatus;

  const NeighborState({
    required this.id,
    required this.lat,
    required this.lon,
    required this.speedKmh,
    required this.headingDeg,
    required this.emergencyStatus,
  });

  factory NeighborState.fromJson(Map<String, dynamic> j) => NeighborState(
        id: j['id'] as String,
        lat: (j['lat'] as num).toDouble(),
        lon: (j['lon'] as num).toDouble(),
        speedKmh: (j['speed_kmh'] as num?)?.toDouble() ?? 0,
        headingDeg: (j['heading_deg'] as num?)?.toDouble() ?? 0,
        emergencyStatus: parseEmergencyStatus(j['emergency_status'] as String?),
      );
}

/// Map string status (NORMAL/WARNING/EMERGENCY) → enum.
EmergencyStatus parseEmergencyStatus(String? s) {
  switch (s?.toUpperCase()) {
    case 'EMERGENCY':
      return EmergencyStatus.emergency;
    case 'WARNING':
      return EmergencyStatus.warning;
    default:
      return EmergencyStatus.normal;
  }
}

/// Collision warning yang DIHITUNG host (v2v_warnings.py) dan ditampilkan UI.
/// Raw strings di sini di-map ke enum UI di presentation layer.
class Warning {
  final String level;        // "warning" | "danger"
  final String type;         // forward_collision / emergency_brake / cross_traffic / ...
  final String? direction;   // front | rear | left | right | null
  final double? distanceM;
  final double? ttcS;        // time-to-collision (s)
  final double? closingKmh;  // closing speed (km/h, +ve = mendekat)
  final String? neighborId;

  const Warning({
    required this.level,
    required this.type,
    this.direction,
    this.distanceM,
    this.ttcS,
    this.closingKmh,
    this.neighborId,
  });

  factory Warning.fromJson(Map<String, dynamic> j) => Warning(
        level: (j['level'] as String?) ?? 'warning',
        type: (j['type'] as String?) ?? 'forward_collision',
        direction: j['direction'] as String?,
        distanceM: (j['distance_m'] as num?)?.toDouble(),
        ttcS: (j['ttc_s'] as num?)?.toDouble(),
        closingKmh: (j['closing_kmh'] as num?)?.toDouble(),
        neighborId: j['neighbor_id'] as String?,
      );
}

/// Satu frame snapshot V2V — semua yang UI butuhkan untuk 1 render cycle.
class V2VFrame {
  /// Timestamp millis since epoch.
  final int timestamp;

  final EgoState ego;
  final List<NeighborState> neighbors;

  /// Warning host-computed (null = aman / host tidak mengirim).
  final Warning? warning;

  const V2VFrame({
    required this.timestamp,
    required this.ego,
    required this.neighbors,
    this.warning,
  });

  /// Parse satu frame JSON dari host (python-app/v2v_json.py contract):
  ///   { "ts":..., "ego":{...}, "neighbors":[...], "warning":{...}|null }
  factory V2VFrame.fromJson(Map<String, dynamic> j) => V2VFrame(
        timestamp: (j['ts'] as num?)?.toInt() ??
            DateTime.now().millisecondsSinceEpoch,
        ego: EgoState.fromJson(j['ego'] as Map<String, dynamic>),
        neighbors: ((j['neighbors'] as List?) ?? const [])
            .cast<Map<String, dynamic>>()
            .map(NeighborState.fromJson)
            .toList(),
        warning: (j['warning'] is Map<String, dynamic>)
            ? Warning.fromJson(j['warning'] as Map<String, dynamic>)
            : null,
      );
}
