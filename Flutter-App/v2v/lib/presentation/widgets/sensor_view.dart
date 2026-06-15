import 'package:flutter/material.dart';
import 'package:syncfusion_flutter_gauges/gauges.dart';

import '../../domain/models/v2v_frame.dart';

// WARNING MODEL
enum WarningLevel { safe, warning, danger }
enum WarningDirection { left, right, front, rear }

/// Differentiated alert categories (matches v2v_warnings.py type strings).
enum WarningType {
  forwardCollision,
  emergencyBrake,
  crossTraffic,
  approachingRear,
  emergencyBroadcast,
  gpsLost,
}

class WarningInfo {
  final WarningLevel level;
  final WarningType type;
  final WarningDirection? direction;   // null for gps-lost
  final double distance;               // metres (0 if N/A)
  final double? ttcS;                  // time-to-collision (s)
  final double? closingKmh;            // closing speed (km/h)

  const WarningInfo({
    required this.level,
    this.type = WarningType.forwardCollision,
    this.direction,
    this.distance = 0,
    this.ttcS,
    this.closingKmh,
  });

  /// Map the host `Warning` (raw strings) → UI model. null stays null.
  static WarningInfo? fromWarning(Warning? w) {
    if (w == null) return null;
    return WarningInfo(
      level: w.level == 'danger' ? WarningLevel.danger : WarningLevel.warning,
      type: _typeFromString(w.type),
      direction: _dirFromString(w.direction),
      distance: w.distanceM ?? 0,
      ttcS: w.ttcS,
      closingKmh: w.closingKmh,
    );
  }

  static WarningType _typeFromString(String s) {
    switch (s) {
      case 'emergency_brake':
        return WarningType.emergencyBrake;
      case 'cross_traffic':
        return WarningType.crossTraffic;
      case 'approaching_rear':
        return WarningType.approachingRear;
      case 'emergency_broadcast':
        return WarningType.emergencyBroadcast;
      case 'gps_lost':
        return WarningType.gpsLost;
      default:
        return WarningType.forwardCollision;
    }
  }

  static WarningDirection? _dirFromString(String? s) {
    switch (s) {
      case 'front':
        return WarningDirection.front;
      case 'rear':
        return WarningDirection.rear;
      case 'left':
        return WarningDirection.left;
      case 'right':
        return WarningDirection.right;
      default:
        return null;
    }
  }
}

// CONNECTION STATUS
enum ConnectionLevel { mock, ok, stale, error }

class ConnectionStatus {
  final String label;
  final ConnectionLevel level;

  const ConnectionStatus({required this.label, required this.level});

  const ConnectionStatus.mock()
      : label = 'MOCK DATA',
        level = ConnectionLevel.mock;
  const ConnectionStatus.live()
      : label = 'LIVE — PI',
        level = ConnectionLevel.ok;
  const ConnectionStatus.stale()
      : label = 'NO DATA',
        level = ConnectionLevel.stale;
  const ConnectionStatus.disconnected()
      : label = 'DISCONNECTED',
        level = ConnectionLevel.error;
}

// MAIN VIEW
class SensorView extends StatelessWidget {
  final EgoState ego;
  final WarningInfo? warning;
  final String timeText;
  final ConnectionStatus connectionStatus;
  final String dateText;

  const SensorView({
    super.key,
    required this.ego,
    this.warning,
    this.timeText = "6:30 PM",
    this.dateText = "6 June 2026",
    this.connectionStatus = const ConnectionStatus.mock(),
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: const BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topCenter,
          end: Alignment.bottomCenter,
          colors: [
            Color(0xFF06091F),
            Color(0xFF0B1430),
            Color(0xFF0E1B3D),
          ],
          stops: [0.0, 0.55, 1.0],
        ),
      ),
      child: SafeArea(
        child: LayoutBuilder(
          builder: (context, c) {
            // Layout sizing — proportional ke layar. Lebih kecil supaya
            // pas di kolom & tidak kepotong di kanan/kiri
            final gaugeSize = (c.maxHeight * 0.68).clamp(260.0, 420.0);

            return Stack(
              children: [
                // Lane lines + pink glow di belakang
                Positioned.fill(
                  child: CustomPaint(painter: _LanePainter()),
                ),

                // Konten utama
                Column(
                  children: [
                    _TopBar(
                      time: timeText,
                      date: dateText,
                      gpsLost: ego.gpsLost,
                      satellites: ego.satellites,
                    ),
                    Expanded(
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.center,
                        children: [
                          Expanded(
                            flex: 5,
                            child: Center(
                              child: _RpmGauge(
                                rpm: ego.engineRpm,
                                speedForGear: ego.speedKmh,
                                size: gaugeSize,
                              ),
                            ),
                          ),
                          Expanded(
                            flex: 5,
                            child: _CenterStage(warning: warning),
                          ),
                          Expanded(
                            flex: 5,
                            child: Center(
                              child: _SpeedGauge(
                                speed: ego.speedKmh,
                                size: gaugeSize,
                              ),
                            ),
                          ),
                        ],
                      ),
                    ),
                    _BottomBar(
                      tempC: ego.engineTempC,
                      fuelLevelPct: ego.fuelLevelPct,
                    ),
                  ],
                ),

                // Positioned(
                //   bottom: 8,
                //   right: 16,
                //   child: _ConnectionBadge(status: connectionStatus),
                // ),
              ],
            );
          },
        ),
      ),
    );
  }
}

// TOP BAR — shelf 
class _TopBar extends StatelessWidget {
  final String time;
  final String date;
  final bool gpsLost;
  final int satellites;
  const _TopBar({
    required this.time,
    required this.date,
    this.gpsLost = false,
    this.satellites = 0,
  });

  static const double _widthFactor = 0.45;

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      height: 54,
      width: double.infinity,
      child: Align(
        alignment: Alignment.topCenter,
        child: FractionallySizedBox(
          widthFactor: _widthFactor,
          child: Container(
            decoration: BoxDecoration(
              color: const Color(0xFF06091F),
              borderRadius: const BorderRadius.only(
                bottomLeft: Radius.circular(36),
                bottomRight: Radius.circular(36),
              ),
              boxShadow: [
                BoxShadow(
                  color: const Color(0xFF4DA3FF).withValues(alpha: 0.25),
                  blurRadius: 20,
                  offset: const Offset(0, 3),
                ),
                BoxShadow(
                  color: const Color(0xFF7BD3F7).withValues(alpha: 0.10),
                  blurRadius: 30,
                  offset: const Offset(0, 3),
                ),
              ],
            ),
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 32, vertical: 12),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text(
                    time,
                    style: const TextStyle(
                      color: Colors.white,
                      fontSize: 17,
                      fontWeight: FontWeight.w400,
                      letterSpacing: 1.0,
                    ),
                  ),
                  _GpsChip(lost: gpsLost, satellites: satellites),
                  Text(
                    date,
                    style: const TextStyle(
                      color: Colors.white,
                      fontSize: 17,
                      fontWeight: FontWeight.w400,
                      letterSpacing: 1.0,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }
}

// GPS health chip — green when locked, red when lost/low-accuracy.
class _GpsChip extends StatelessWidget {
  final bool lost;
  final int satellites;
  const _GpsChip({required this.lost, required this.satellites});

  @override
  Widget build(BuildContext context) {
    final color = lost ? const Color(0xFFFF3B30) : const Color(0xFF38D66B);
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Icon(lost ? Icons.gps_off_rounded : Icons.gps_fixed_rounded,
            color: color, size: 16),
        const SizedBox(width: 5),
        Text(
          lost ? "GPS" : "$satellites",
          style: TextStyle(
            color: color,
            fontSize: 14,
            fontWeight: FontWeight.w600,
            letterSpacing: 0.5,
          ),
        ),
      ],
    );
  }
}

// CENTER STAGE — car + warning card
class _CenterStage extends StatelessWidget {
  final WarningInfo? warning;
  const _CenterStage({this.warning});

  @override
  Widget build(BuildContext context) {
    return Stack(
      alignment: Alignment.center,
      children: [
        // Pink glow di bawah mobil
        Positioned(
          bottom: 7,
          child: CustomPaint(
            size: const Size(320, 150),
            painter: _GlowPainter(),
          ),
        ),
        // Mobil
        Positioned(
          bottom: 40,
          child: Image.asset(
            'assets/car_behind.png',
            width: 150,
            fit: BoxFit.contain,
            errorBuilder: (_, _, _) => Image.asset(
              'assets/fortuner_top.png',
              width: 180,
            ),
          ),
        ),
        // Warning card
        Positioned(
          top: 50,
          child: AnimatedSwitcher(
            duration: const Duration(milliseconds: 350),
            switchInCurve: Curves.easeOut,
            switchOutCurve: Curves.easeIn,
            transitionBuilder: (child, anim) {
              return FadeTransition(
                opacity: anim,
                child: SlideTransition(
                  position: Tween<Offset>(
                    begin: const Offset(0, -0.25),
                    end: Offset.zero,
                  ).animate(anim),
                  child: child,
                ),
              );
            },
            child: (warning == null || warning!.level == WarningLevel.safe)
                ? const SizedBox.shrink(key: ValueKey('safe'))
                : _WarningCard(
                    key: ValueKey('${warning!.level}-${warning!.direction}'),
                    info: warning!,
                  ),
          ),
        ),
      ],
    );
  }
}

// WARNING CARD
class _WarningCard extends StatelessWidget {
  final WarningInfo info;
  const _WarningCard({super.key, required this.info});

  Color get _accent => info.level == WarningLevel.danger
      ? const Color(0xFFFF3B30)
      : const Color(0xFFFFB020);

  // Per-type icon (overrides direction for the special alert types).
  IconData get _icon {
    switch (info.type) {
      case WarningType.emergencyBrake:
        return Icons.car_crash_rounded;
      case WarningType.gpsLost:
        return Icons.gps_off_rounded;
      case WarningType.emergencyBroadcast:
        return Icons.warning_amber_rounded;
      case WarningType.crossTraffic:
        return info.direction == WarningDirection.left
            ? Icons.subdirectory_arrow_left_rounded
            : Icons.subdirectory_arrow_right_rounded;
      default:
        switch (info.direction) {
          case WarningDirection.left:
            return Icons.arrow_back_rounded;
          case WarningDirection.right:
            return Icons.arrow_forward_rounded;
          case WarningDirection.rear:
            return Icons.arrow_downward_rounded;
          case WarningDirection.front:
          default:
            return Icons.arrow_upward_rounded;
        }
    }
  }

  String get _dirWord {
    switch (info.direction) {
      case WarningDirection.left:
        return "LEFT";
      case WarningDirection.right:
        return "RIGHT";
      case WarningDirection.rear:
        return "BEHIND";
      case WarningDirection.front:
        return "AHEAD";
      default:
        return "";
    }
  }

  // Headline per alert type.
  String get _title {
    switch (info.type) {
      case WarningType.emergencyBrake:
        return "BRAKING $_dirWord";
      case WarningType.gpsLost:
        return "GPS SIGNAL LOST";
      case WarningType.emergencyBroadcast:
        return "EMERGENCY $_dirWord";
      case WarningType.crossTraffic:
        return "CROSS TRAFFIC $_dirWord";
      case WarningType.approachingRear:
        return "APPROACHING REAR";
      case WarningType.forwardCollision:
        return "VEHICLE $_dirWord";
    }
  }

  // Subtitle: distance + TTC / closing, or a hint for gps-lost.
  String get _subtitle {
    if (info.type == WarningType.gpsLost) return "Position unreliable";
    final parts = <String>[];
    if (info.distance > 0) parts.add("${info.distance.toStringAsFixed(0)} m");
    if (info.ttcS != null && info.ttcS! > 0 && info.ttcS! < 99) {
      parts.add("TTC ${info.ttcS!.toStringAsFixed(1)} s");
    }
    if (info.closingKmh != null && info.closingKmh! > 1) {
      parts.add("+${info.closingKmh!.toStringAsFixed(0)} km/h");
    }
    return parts.join("  •  ");
  }

  @override
  Widget build(BuildContext context) {
    final isDanger = info.level == WarningLevel.danger;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 22, vertical: 14),
      decoration: BoxDecoration(
        color: const Color(0xFF0A0E2A).withValues(alpha: 0.85),
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: _accent, width: 2),
        boxShadow: [
          BoxShadow(
            color: _accent.withValues(alpha: 0.6),
            blurRadius: 28,
            spreadRadius: 2,
          ),
        ],
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(_icon, color: _accent, size: 30),
          const SizedBox(width: 14),
          Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
              Text(
                _title,
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  letterSpacing: 0.3,
                ),
              ),
              if (_subtitle.isNotEmpty) ...[
                const SizedBox(height: 2),
                Text(
                  _subtitle,
                  style: TextStyle(
                    color: Colors.white.withValues(alpha: 0.85),
                    fontSize: 13,
                    fontWeight: FontWeight.w500,
                  ),
                ),
              ],
            ],
          ),
          const SizedBox(width: 12),
          if (isDanger) _PulsingDot(color: _accent),
        ],
      ),
    );
  }
}

class _PulsingDot extends StatefulWidget {
  final Color color;
  const _PulsingDot({required this.color});

  @override
  State<_PulsingDot> createState() => _PulsingDotState();
}

class _PulsingDotState extends State<_PulsingDot>
    with SingleTickerProviderStateMixin {
  late final AnimationController _c =
      AnimationController(vsync: this, duration: const Duration(milliseconds: 800))
        ..repeat(reverse: true);

  @override
  void dispose() {
    _c.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return FadeTransition(
      opacity: Tween<double>(begin: 0.3, end: 1.0).animate(_c),
      child: Container(
        width: 10,
        height: 10,
        decoration: BoxDecoration(
          shape: BoxShape.circle,
          color: widget.color,
          boxShadow: [
            BoxShadow(color: widget.color, blurRadius: 8),
          ],
        ),
      ),
    );
  }
}

// RPM GAUGE (KIRI) 
class _RpmGauge extends StatelessWidget {
  final double rpm;
  final double speedForGear;
  final double size;

  const _RpmGauge({
    required this.rpm,
    required this.speedForGear,
    required this.size,
  });

  double get _rpmK => (rpm / 1000).clamp(0, 8).toDouble();

  int get _gear {
    if (speedForGear < 1) return 0;
    if (speedForGear < 20) return 1;
    if (speedForGear < 40) return 2;
    if (speedForGear < 60) return 3;
    if (speedForGear < 90) return 4;
    if (speedForGear < 120) return 5;
    return 6;
  }

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: size,
      height: size,
      child: SfRadialGauge(
            axes: <RadialAxis>[
              RadialAxis(
                minimum: 0,
                maximum: 8,
                startAngle: 110,
                endAngle: 70,
                interval: 1,
                radiusFactor: 0.92,
                showLastLabel: true,
                axisLineStyle: const AxisLineStyle(
                  thickness: 10,
                  color: Color(0x1AFFFFFF), // track tipis abu-abu untuk full scale
                  thicknessUnit: GaugeSizeUnit.logicalPixel,
                  cornerStyle: CornerStyle.bothCurve,
                ),
                majorTickStyle: const MajorTickStyle(
                  length: 12,
                  thickness: 2,
                  color: Colors.white70,
                ),
                minorTickStyle: const MinorTickStyle(
                  length: 6,
                  thickness: 1,
                  color: Colors.white30,
                ),
                minorTicksPerInterval: 4,
                axisLabelStyle: const GaugeTextStyle(
                  color: Colors.white,
                  fontSize: 16,
                  fontWeight: FontWeight.w500,
                ),
                labelOffset: 22,
                // RangePointer = arc terisi (palette biru-cyan elegant)
                pointers: <GaugePointer>[
                  RangePointer(
                    value: _rpmK,
                    width: 10,
                    cornerStyle: CornerStyle.bothCurve,
                    enableAnimation: true,
                    animationType: AnimationType.ease,
                    gradient: const SweepGradient(
                      colors: [
                        Color(0xFF3B82F6), // blue
                        Color(0xFF4DA3FF), // medium blue
                        Color(0xFF5BB6FF), // light blue
                        Color(0xFF7BD3F7), // sky blue
                        Color(0xFFB5E8FF), // pale cyan
                      ],
                      stops: [0.0, 0.25, 0.5, 0.75, 1.0],
                    ),
                  ),
                ],
                annotations: <GaugeAnnotation>[
                  const GaugeAnnotation(
                    positionFactor: 0.32,
                    angle: 270,
                    widget: Text(
                      'x1000 rpm',
                      style: TextStyle(
                        color: Colors.white54,
                        fontSize: 12,
                        letterSpacing: 1.5,
                      ),
                    ),
                  ),
                  GaugeAnnotation(
                    positionFactor: 0,
                    widget: Text(
                      "$_gear",
                      style: const TextStyle(
                        color: Colors.white,
                        fontSize: 84,
                        fontWeight: FontWeight.w500,
                        height: 1,
                      ),
                    ),
                  ),
                  const GaugeAnnotation(
                    positionFactor: 0.32,
                    angle: 90, // bottom
                    widget: Text(
                      "GEAR",
                      style: TextStyle(
                        color: Colors.white60,
                        fontSize: 12,
                        letterSpacing: 2.5,
                      ),
                    ),
                  ),
                ],
              ),
            ],
          ),
    );
  }
}

// SPEED GAUGE (KANAN)
class _SpeedGauge extends StatelessWidget {
  final double speed;
  final double size;

  const _SpeedGauge({required this.speed, required this.size});

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: size,
      height: size,
      child: SfRadialGauge(
            axes: <RadialAxis>[
              RadialAxis(
                minimum: 0,
                maximum: 240,
                startAngle: 110,
                endAngle: 70,
                interval: 20,
                radiusFactor: 0.92,
                showLastLabel: true,
                axisLineStyle: const AxisLineStyle(
                  thickness: 10,
                  color: Color(0x1AFFFFFF), // track tipis abu-abu untuk full scale
                  thicknessUnit: GaugeSizeUnit.logicalPixel,
                  cornerStyle: CornerStyle.bothCurve,
                ),
                majorTickStyle: const MajorTickStyle(
                  length: 12,
                  thickness: 2,
                  color: Colors.white70,
                ),
                minorTickStyle: const MinorTickStyle(
                  length: 6,
                  thickness: 1,
                  color: Colors.white30,
                ),
                minorTicksPerInterval: 4,
                axisLabelStyle: const GaugeTextStyle(
                  color: Colors.white,
                  fontSize: 14,
                  fontWeight: FontWeight.w500,
                ),
                labelOffset: 20,
                // RangePointer = arc terisi (palette biru-cyan elegant)
                pointers: <GaugePointer>[
                  RangePointer(
                    value: speed,
                    width: 10,
                    cornerStyle: CornerStyle.bothCurve,
                    enableAnimation: true,
                    animationType: AnimationType.ease,
                    gradient: const SweepGradient(
                      colors: [
                        Color(0xFF3B82F6), // blue
                        Color(0xFF4DA3FF), // medium blue
                        Color(0xFF5BB6FF), // light blue
                        Color(0xFF7BD3F7), // sky blue
                        Color(0xFFB5E8FF), // pale cyan
                      ],
                      stops: [0.0, 0.25, 0.5, 0.75, 1.0],
                    ),
                  ),
                ],
                annotations: <GaugeAnnotation>[
                  const GaugeAnnotation(
                    positionFactor: 0.32,
                    angle: 270,
                    widget: Text(
                      'km/h',
                      style: TextStyle(
                        color: Colors.white60,
                        fontSize: 13,
                        letterSpacing: 1.5,
                      ),
                    ),
                  ),
                  GaugeAnnotation(
                    positionFactor: 0,
                    widget: Text(
                      speed.toStringAsFixed(0),
                      style: const TextStyle(
                        color: Colors.white,
                        fontSize: 84,
                        fontWeight: FontWeight.w500,
                        height: 1,
                      ),
                    ),
                  ),
                  // Eco icon at bottom
                  const GaugeAnnotation(
                    positionFactor: 0.65,
                    angle: 60, // lower-right
                    widget: Icon(
                      Icons.eco_rounded,
                      color: Color(0xFF4ADE80),
                      size: 26,
                    ),
                  ),
                ],
              ),
            ],
          ),
    );
  }
}

// BOTTOM BAR (fuel | range | coolant temp)
class _BottomBar extends StatelessWidget {
  final double tempC;
  final double fuelLevelPct;
  const _BottomBar({required this.tempC, required this.fuelLevelPct});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 50, vertical: 40),
      child: Row(
        children: [
          Expanded(
            child: _FuelBar(fuel: (fuelLevelPct / 100).clamp(0.0, 1.0)),
          ),
          // COOLANT TEMP (dari ego.engineTempC)
          Expanded(
            child: _CoolantBar(tempC: tempC),
          ),
        ],
      ),
    );
  }
}

class _FuelBar extends StatelessWidget {
  final double fuel; // 0.0 - 1.0
  const _FuelBar({required this.fuel});

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.start,
      children: [
        const Icon(Icons.local_gas_station_rounded, color: Colors.white60, size: 22),
        const SizedBox(width: 8),
        const Text('E', style: TextStyle(color: Colors.white60, fontSize: 12)),
        const SizedBox(width: 6),
        SizedBox(
          width: 140,
          height: 8,
          child: Row(
            children: List.generate(10, (i) {
              final filled = (i / 10) < fuel;
              return Expanded(
                child: Container(
                  margin: const EdgeInsets.symmetric(horizontal: 1.5),
                  decoration: BoxDecoration(
                    color: filled
                        ? const Color(0xFF4DA3FF)
                        : Colors.white12,
                    borderRadius: BorderRadius.circular(2),
                    boxShadow: filled
                        ? [
                            BoxShadow(
                              color: const Color(0xFF4DA3FF)
                                  .withValues(alpha: 0.5),
                              blurRadius: 4,
                            ),
                          ]
                        : null,
                  ),
                ),
              );
            }),
          ),
        ),
        const SizedBox(width: 6),
        const Text('F', style: TextStyle(color: Colors.white60, fontSize: 12)),
      ],
    );
  }
}

class _CoolantBar extends StatelessWidget {
  final double tempC;
  const _CoolantBar({required this.tempC});

  @override
  Widget build(BuildContext context) {
    final normalized = ((tempC - 60) / 60).clamp(0.0, 1.0);
    return Row(
      mainAxisAlignment: MainAxisAlignment.end,
      children: [
        const Icon(Icons.water_drop_outlined, color: Colors.white60, size: 22),
        const SizedBox(width: 8),
        const Text('C', style: TextStyle(color: Colors.white60, fontSize: 12)),
        const SizedBox(width: 6),
        SizedBox(
          width: 140,
          height: 8,
          child: Stack(
            children: [
              ClipRRect(
                borderRadius: BorderRadius.circular(4),
                child: Container(
                  decoration: const BoxDecoration(
                    gradient: LinearGradient(
                      colors: [
                        Color(0xFF3B82F6), // blue
                        Color(0xFF5BB6FF), // light blue
                        Color(0xFFB5E8FF), // pale cyan
                      ],
                    ),
                  ),
                ),
              ),
              Positioned(
                left: (140 * normalized).clamp(0.0, 134.0),
                top: -2,
                child: Container(
                  width: 6,
                  height: 12,
                  decoration: BoxDecoration(
                    color: Colors.white,
                    borderRadius: BorderRadius.circular(2),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.white.withValues(alpha: 0.6),
                        blurRadius: 6,
                      ),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
        const SizedBox(width: 6),
        const Text('H', style: TextStyle(color: Colors.white60, fontSize: 12)),
      ],
    );
  }
}

// CONNECTION BADGE — class disimpan untuk dipakai saat SerialDataSource aktif.
// Saat itu hapus baris `ignore` di bawah dan uncomment penggunaannya di SensorView.
// ignore: unused_element
class _ConnectionBadge extends StatelessWidget {
  final ConnectionStatus status;
  const _ConnectionBadge({required this.status});

  Color get _color {
    switch (status.level) {
      case ConnectionLevel.ok:
        return const Color(0xFF4ADE80);
      case ConnectionLevel.mock:
        return const Color(0xFFFFB020);
      case ConnectionLevel.stale:
        return const Color(0xFFFFCC00);
      case ConnectionLevel.error:
        return const Color(0xFFFF3B30);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
      decoration: BoxDecoration(
        color: const Color(0xFF0F1330).withValues(alpha: 0.7),
        borderRadius: BorderRadius.circular(20),
        border: Border.all(color: _color.withValues(alpha: 0.4), width: 1),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 7,
            height: 7,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: _color,
              boxShadow: [BoxShadow(color: _color, blurRadius: 5)],
            ),
          ),
          const SizedBox(width: 7),
          Text(
            status.label,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 10,
              fontWeight: FontWeight.w500,
              letterSpacing: 1.0,
            ),
          ),
        ],
      ),
    );
  }
}

// LANE PAINTER (garis perspektif jalan)
class _LanePainter extends CustomPainter {
  static const double _bottomHalfWidth = 200;
  static const double _topHalfWidth = 50;
  static const double _topYFactor = 0.45;
  static const double _bottomYFactor = 0.82;

  static const double _xOffset = 0;

  @override
  void paint(Canvas canvas, Size size) {
    final cx = size.width / 2 + _xOffset;
    final bottom = size.height * _bottomYFactor;
    final topY = size.height * _topYFactor;

    final paint = Paint()
      ..color = Colors.white.withValues(alpha: 0.35)
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round;

    final glowPurple = Paint()
      ..color = const Color(0xFF4DA3FF).withValues(alpha: 0.18)
      ..strokeWidth = 14
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 6);

    final glowBlue = Paint()
      ..color = const Color(0xFF7BD3F7).withValues(alpha: 0.12)
      ..strokeWidth = 22
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 10);

    final leftPath = Path()
      ..moveTo(cx - _bottomHalfWidth, bottom)
      ..lineTo(cx - _topHalfWidth, topY);

    final rightPath = Path()
      ..moveTo(cx + _bottomHalfWidth, bottom)
      ..lineTo(cx + _topHalfWidth, topY);

    // Layer: glow biru (paling luar) → glow ungu → garis utama
    canvas.drawPath(leftPath, glowBlue);
    canvas.drawPath(rightPath, glowBlue);
    canvas.drawPath(leftPath, glowPurple);
    canvas.drawPath(rightPath, glowPurple);
    canvas.drawPath(leftPath, paint);
    canvas.drawPath(rightPath, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}

// GLOW PAINTER
class _GlowPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height * 0.75);
    final rect = Rect.fromCircle(center: center, radius: size.width * 0.42);

    final gradient = RadialGradient(
      colors: [
        const Color(0xFF4DA3FF).withValues(alpha: 0.22),
        const Color(0xFF7BD3F7).withValues(alpha: 0.08),
        Colors.transparent,
      ],
      stops: const [0.0, 0.5, 1.0],
    );

    final paint = Paint()..shader = gradient.createShader(rect);
    canvas.drawOval(rect, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => false;
}
