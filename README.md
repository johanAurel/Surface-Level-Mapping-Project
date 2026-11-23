# Surface Level Mapping Project

## Overview
This project implements the described reflector-based field elevation survey concept in software structure: a wheeled robot with camera+LiDAR+IMU+2-axis turret tracks a fixed reflector, logs measurements, and derives robot elevation/terrain-ready variables. It does not generate a DEM/mesh.

**Important:** this repository code derives terrain-ready data (`point_x/y/z`, beam vectors, pose, range, tracking errors). It does **not** generate a terrain mesh/DEM by itself.

## What is integrated (OOP)
The main integration path is object-oriented and centered on `SurfaceMappingSystem`:

- `backend/data/main.py` - `MappingConfig`, `PlatformPositionEstimator`, `SurfaceMapDataLogger`, `SurfaceMappingSystem`
- `backend/actuators/cycloidal_actuator.py` - `CycloidalActuator` (pan/tilt cycloidal axis abstraction)
- `backend/actuators/turret.py` - `PanTiltTurret` (2-axis cycloidal turret tracking from camera error)
- `backend/actuators/readings.py` - `ActuatorReadings` (pan/tilt encoders)
- `backend/encoders/quadrature.py` - `QuadratureEncoder`
- `backend/sensors/encoder_readings.py` - `SensorEncoder` (external/wheel encoder)
- `backend/actuators/imu_rpy.py` - `ImuOrientation` / `ImuReading`
- `backend/sensors/lidar_sensor.py` - `LidarSensor` / `LidarReading`
- `backend/sensors/camera_tracker.py` - `CameraTracker` / `CameraTrackingReading`
- `backend/data/processing.py` - `combine_readings()` (measurement fusion/flattening)
- `backend/data/coordinates.py` - `calculate_robot_elevation()` / `calculate_coordinates()`

Standalone demos/utilities (kept, not the main integrated loop): `backend/sensors/camera/*.py`, `backend/sensors/lidar/main.py`, `backend/actuators/2_axis_turret.py`.


## Alignment with described system (image description)
- **Camera**: detects/tracks reflector/fiducial (`CameraTracker`, `reference_detected`, pixel errors).
- **Single-point LiDAR**: measures distance `d` (`lidar_distance`).
- **IMU**: provides roll/pitch/yaw (`ImuOrientation`).
- **2-axis turret**: pan/tilt angles `theta_h/theta_v` from actuator encoders; `PanTiltTurret` + `CycloidalActuator` provide integrated cycloidal pan/tilt tracking commands from camera pixel error.
- **Raspberry Pi/controller**: `SurfaceMappingSystem` orchestrates read/track/compute/log.
- **Primary elevation formula**: `z_robot = H_ref - d*sin(theta_v)` implemented in `backend/data/coordinates.py:calculate_robot_elevation()` and used by `SurfaceMappingSystem` in `reflector_elevation` mode.
- **Terrain-ready outputs**: timestamped `x,y,d,theta_v,theta_h,yaw,z_robot`/`point_z` CSV rows for field height map/DEM generation downstream, plus tracking vars (`tracking_active`, target/delta/clamped).

## Derived variables for terrain mapping
Per measurement cycle, the system can derive/log:
- **Time**: `timestamp`
- **Actuator geometry**: `pan_counts`, `pan_angle`, `tilt_counts`, `tilt_angle`
- **Platform motion input**: `sensor_encoder_counts`, `sensor_encoder_degrees`, `platform_travel_m`, `odometry_available`
- **Orientation**: `roll`, `pitch`, `yaw`, `temperature_c`
- **Range observation**: `lidar_distance`, `lidar_strength`, `lidar_quality`, `lidar_valid`
- **Camera reference tracking**: `reference_detected`, `reference_x_px`, `reference_y_px`, `x_error_px`, `y_error_px`, `area_px`, `bbox/frame`
- **Platform pose**: `platform_x`, `platform_y`, `platform_z` (requires calibration/localization)
- **Terrain-ready 3D outputs**: `beam_x/y/z`, `point_x/y/z`

These variables are sufficient inputs for downstream point-cloud/surface reconstruction when platform pose is properly calibrated.

## Data flow (pseudocode)
```text
INITIALIZE hardware and CSV logger
LOOP every sampling_interval:
  actuators = read pan/tilt encoders
  encoder   = read external encoder
  imu       = read roll/pitch/yaw
  lidar     = read distance frame
  camera    = detect reference + pixel error
  pose      = estimate platform position
  record    = combine_readings(all of above)
  IF lidar valid AND pose available:
      point = calculate_coordinates(record)
      record += beam_x/y/z, point_x/y/z
  WRITE record to CSV
CLOSE all resources
```

## How to run
```bash
cd "Surface Level Mapping Project"
python3 -m backend.main
```

Output CSV default: `backend/data/surface_measurements.csv`.

## Configuration / calibration notes
Edit `MappingConfig` in `backend/data/main.py`:
- GPIO pins and encoder counts/rev
- `wheel_circumference_m` for odometry
- sampling interval and output file
- `require_reference_detection` if camera lock is mandatory

For accurate terrain coordinates, provide real platform localization and mounting calibration; `PlatformPositionEstimator` is currently a simple odometry placeholder.

## Repository layout
- `backend/` - integrated backend, sensor/actuator modules, demos
- `backend/data/` - orchestration, fusion, coordinate transforms
- `backend/sensors/` - LiDAR/camera/encoder interfaces
- `backend/actuators/` - IMU and actuator interfaces
- `backend/encoders/` - quadrature encoder abstraction

## Status / limitations
- Core Python modules compile (`python3 -m py_compile` on backend `.py` files).
- Hardware execution requires Raspberry Pi dependencies/devices (GPIO, serial, I2C, camera).
- I cannot inspect `/mnt/c/Users/user/Downloads/machine.png` in this environment (image input unsupported by current model), so comparison to that diagram must be done manually against the derived-variable list above.
