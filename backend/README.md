# Backend - Surface Level Mapping

## Role
The backend is the executable core of the project. It integrates sensors/actuators, derives 3D surface-point variables, and logs them to CSV for downstream terrain mapping.

## Entry points
- `python3 -m backend.main` - recommended
- `python3 -m backend.data.main` - direct orchestrator module
- `python3 -m backend.data.main --simulate` - no-hardware simulation (20 samples, progress printed)
- Flags: `--simulate`, `--cycles N`, `--output PATH`, `--reflector-height METRES`
- Real mode performs hardware preflight on the Pi: BCM pin validation, GPIO pin read test, device read test, then acquisition.
- Simulation prints `t, x, y, d, theta_v, theta_h, yaw, h_ref, z` per sample and writes all fields to CSV; `h_ref` defaults to 1.5 m and odometry is simulated so `x/y` are populated.

## OOP architecture
```text
SurfaceMappingSystem (backend/data/main.py)
  ├─ ActuatorReadings (backend/actuators/readings.py)
  │   ├─ QuadratureEncoder (pan)
  │   └─ QuadratureEncoder (tilt)
  ├─ PanTiltTurret (backend/actuators/turret.py)
  │   ├─ CycloidalActuator (pan)
  │   └─ CycloidalActuator (tilt)
  ├─ SensorEncoder (backend/sensors/encoder_readings.py)
  ├─ ImuOrientation (backend/actuators/imu_rpy.py)
  ├─ LidarSensor (backend/sensors/lidar_sensor.py)
  ├─ CameraTracker (backend/sensors/camera_tracker.py)
  ├─ PlatformPositionEstimator (backend/data/main.py)
  ├─ combine_readings() (backend/data/processing.py)
  ├─ calculate_robot_elevation()/calculate_coordinates() (backend/data/coordinates.py)
  └─ SurfaceMapDataLogger (backend/data/main.py)
```

## Pseudocode (integrated loop)
```text
system.initialise_hardware()
logger.open()
WHILE running:
    m = collect_measurements()
    m = process_measurement(m)   # validate + derive point_x/y/z
    IF m is valid: logger.write(m)
    sleep(sampling_interval)
close all resources
```

## Terrain-ready output variables
Reflector-elevation mode (matches description): `timestamp, platform_x/y, lidar_distance(d), pan_angle(theta_h), tilt_angle(theta_v), roll/pitch/yaw, reflector_height_m(H_ref), z_robot_m/point_z, reference tracking errors`.  
Legacy beam-projection mode: also includes `beam_x/y/z, point_x/y/z` from beam geometry.

Primary formula: `z_robot = H_ref - d*sin(theta_v)` (`backend/data/coordinates.py`).

## Calibration required
- Encoder counts per revolution
- Wheel circumference / odometry model
- Sensor mounting offsets and axis conventions
- LiDAR/camera alignment to platform frame

Without real platform localization, absolute `point_x/y/z` cannot be trusted for terrain reconstruction.
