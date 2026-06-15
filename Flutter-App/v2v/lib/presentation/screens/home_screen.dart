import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';

import '../../data/data_source.dart';
import '../../data/mock_data_source.dart';
import '../../data/tcp_data_source.dart';
// import '../../data/jsonl_file_data_source.dart'; // ← untuk playback file recording
// import '../../data/serial_data_source.dart';     // ← untuk live data dari Pi
import '../../domain/models/v2v_frame.dart';
import '../widgets/sensor_view.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {

  // Data source — pick one:
  //   MockDataSource()                      : CSV playback baked into the app (offline demo)
  //   TcpDataSource(host:'localhost')       : live host / simulator over TCP JSON :8765
  //                                           (main_ukf.py --json-tcp 8765, or simulate_v2v.py)
  //   SerialDataSource()                    : direct USB CDC from the MCU
  // For a remote dashboard (phone/tablet), set host to the PC's LAN IP.
  // late final DataSource _source = MockDataSource();
  late final DataSource _source = TcpDataSource(host: 'localhost', port: 8765);

  EgoState _ego = EgoState.empty;
  WarningInfo? _warning;
  String _timeText = "";
  String _dateText = "";
  ConnectionStatus _connectionStatus = const ConnectionStatus.mock();

  StreamSubscription<V2VFrame>? _sub;
  Timer? _clockTimer;
  Timer? _staleWatcher;
  DateTime? _lastFrameAt;
  bool _firstFrame = true;

  @override
  void initState() {
    super.initState();
    _initConnectionStatus();
    _startClock();
    _startStream();
    _startStaleWatcher();
  }

  @override
  void dispose() {
    _sub?.cancel();
    _clockTimer?.cancel();
    _staleWatcher?.cancel();
    _source.dispose();
    super.dispose();
  }

  // Connection status
  void _initConnectionStatus() {
    if (_source is MockDataSource) {
      _connectionStatus = const ConnectionStatus.mock();
    } else {
      _connectionStatus = const ConnectionStatus.live();
    }
  }

  void _startStaleWatcher() {
    _staleWatcher = Timer.periodic(const Duration(seconds: 1), (_) {
      if (!mounted) return;
      if (_source is MockDataSource) return; // mock selalu hidup
      if (_lastFrameAt == null) return;

      final ageSec = DateTime.now().difference(_lastFrameAt!).inSeconds;
      ConnectionStatus newStatus;
      if (ageSec >= 5) {
        newStatus = const ConnectionStatus.disconnected();
      } else if (ageSec >= 2) {
        newStatus = const ConnectionStatus.stale();
      } else {
        newStatus = const ConnectionStatus.live();
      }
      if (newStatus.level != _connectionStatus.level) {
        setState(() => _connectionStatus = newStatus);
      }
    });
  }

  // Stream subscription dari DataSource
  void _startStream() {
    _sub = _source.stream().listen(
      (frame) {
        if (!mounted) return;
        _lastFrameAt = DateTime.now();
        setState(() {
          _ego = frame.ego;
          // Prefer the host-computed warning (TTC / path-filtered / EEBL).
          // Fall back to a local estimate for sources that don't send one
          // (e.g. MockDataSource).
          _warning = frame.warning != null
              ? WarningInfo.fromWarning(frame.warning)
              : _computeWarning(frame.ego, frame.neighbors);
          _firstFrame = false;
        });
      },
      onError: (e) {
        debugPrint('DataSource error: $e');
        if (mounted) {
          setState(() => _connectionStatus = const ConnectionStatus.disconnected());
        }
      },
    );
  }

  // Local fallback warning (coarse) — only used when the host doesn't supply
  // a `warning` (e.g. MockDataSource). The live/sim path uses the host engine.
  WarningInfo? _computeWarning(EgoState ego, List<NeighborState> neighbors) {
    WarningInfo? best;
    double bestDist = double.infinity;

    for (final n in neighbors) {
      const earthR = 6371000.0;
      final egoLatRad = ego.lat * pi / 180;
      final dxEast = (n.lon - ego.lon) * pi / 180 * earthR * cos(egoLatRad);
      final dyNorth = (n.lat - ego.lat) * pi / 180 * earthR;
      final h = ego.headingDeg * pi / 180;
      final dxBody = dxEast * cos(h) - dyNorth * sin(h);
      final dyBody = dxEast * sin(h) + dyNorth * cos(h);

      // Distance & direction
      final distance = sqrt(dxBody * dxBody + dyBody * dyBody);

      // Level dari jarak
      WarningLevel level;
      if (distance < 10) {
        level = WarningLevel.danger;
      } else if (distance < 25) {
        level = WarningLevel.warning;
      } else {
        level = WarningLevel.safe;
      }

      // Override dari emergency_status broadcast
      WarningType type = WarningType.forwardCollision;
      if (n.emergencyStatus == EmergencyStatus.emergency) {
        level = WarningLevel.danger;
        type = WarningType.emergencyBroadcast;
      } else if (n.emergencyStatus == EmergencyStatus.warning &&
          level == WarningLevel.safe) {
        level = WarningLevel.warning;
        type = WarningType.emergencyBroadcast;
      }

      if (level == WarningLevel.safe) continue;
      if (distance >= bestDist) continue;

      // Tentukan arah relatif (body frame)
      WarningDirection dir;
      if (dyBody > 5) {
        dir = WarningDirection.front;
      } else if (dyBody < -5) {
        dir = WarningDirection.rear;
      } else if (dxBody < 0) {
        dir = WarningDirection.left;
      } else {
        dir = WarningDirection.right;
      }

      best = WarningInfo(
        level: level,
        type: type,
        direction: dir,
        distance: distance,
      );
      bestDist = distance;
    }

    // No neighbour threat — surface a GPS-lost warning if positioning is bad.
    if (best == null && ego.gpsLost) {
      return const WarningInfo(
        level: WarningLevel.warning,
        type: WarningType.gpsLost,
      );
    }
    return best;
  }

  // ============================================================
  // Clock di gauge kiri
  void _startClock() {
    _updateTime();
    _clockTimer = Timer.periodic(const Duration(seconds: 15), (_) {
      if (mounted) _updateTime();
    });
  }

  void _updateTime() {
    final now = DateTime.now();
    int hour = now.hour;
    final period = hour >= 12 ? "PM" : "AM";
    hour = hour % 12;
    if (hour == 0) hour = 12;
    final mm = now.minute.toString().padLeft(2, '0');

    const months = [
      'January', 'February', 'March', 'April', 'May', 'June',
      'July', 'August', 'September', 'October', 'November', 'December'
    ];
    final dateStr = '${now.day} ${months[now.month - 1]} ${now.year}';

    setState(() {
      _timeText = "$hour:$mm $period";
      _dateText = dateStr;
    });
  }

  @override
  Widget build(BuildContext context) {
    if (_firstFrame) {
      return const Scaffold(
        backgroundColor: Color(0xFF0A0E2A),
        body: Center(
          child: CircularProgressIndicator(color: Colors.white70),
        ),
      );
    }

    return Scaffold(
      backgroundColor: const Color(0xFF06091F),
      body: SensorView(
        ego: _ego,
        warning: _warning,
        timeText: _timeText,
        dateText: _dateText,
        connectionStatus: _connectionStatus,
      ),
    );
  }
}
