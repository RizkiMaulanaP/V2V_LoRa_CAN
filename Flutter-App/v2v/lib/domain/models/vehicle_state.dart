class VehicleState {
  final String id;
  final double x;
  final double y;
  final double speed;
  final double heading;
  final double lat;
  final double lng;

  VehicleState({
    required this.id,
    required this.x,
    required this.y,
    required this.speed,
    required this.heading,
    required this.lat,
    required this.lng,
  });

  factory VehicleState.fromMap(Map<String, dynamic> data) {
  final lat = double.tryParse(data['lat'].toString()) ?? 0;
  final lng = double.tryParse(data['lon'].toString()) ?? 0;

  final x = double.tryParse(data['pos_east_m'].toString()) ?? 0;
  final y = double.tryParse(data['pos_north_m'].toString()) ?? 0;

  return VehicleState(
    id: "ego",
    x: x,
    y: y,
    speed: double.tryParse(data['speed_kmh'].toString()) ?? 0,
    heading: double.tryParse(data['yaw_deg'].toString()) ?? 0,
    lat: lat,
    lng: lng,
  );
}
}
