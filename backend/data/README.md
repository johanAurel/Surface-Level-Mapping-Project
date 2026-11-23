# Data Module

## Purpose
Contains the integration/orchestration and data transforms that turn raw sensor readings into terrain-ready 3D point variables.

## Files
- `main.py` - `SurfaceMappingSystem` orchestration, config, logging, odometry placeholder, and cycloidal turret tracking integration.
- `processing.py` - `combine_readings()` flattens component readings into one record.
- `coordinates.py` - `calculate_robot_elevation()` for reflector-based `z_robot = H_ref - d*sin(theta_v)`; `calculate_coordinates()` for legacy beam projection.

## OOP/pseudocode
```text
SurfaceMappingSystem.run():
  init hardware + logger
  for each cycle:
    record = combine_readings(
      timestamp,
      read_actuators(),
      read_sensor_encoder(),
      read_imu(),
      read_lidar(),
      read_camera(),
      read_platform_position()
    )
    record = process_measurement(record)
    if record: save(record)
  close()
```

## Derived variables
- Reflector mode (primary): `z_robot_m = H_ref - d*sin(theta_v)`, plus `distance_to_reflector_m`, `theta_h_deg`, `theta_v_deg`, horizontal/reflector-local components.
- Tracking vars: `pan/tilt_target_angle`, `tracking_active`, `pan/tilt_delta_deg`, `pan/tilt_command_clamped`.
- Legacy beam mode: `beam_x, beam_y, beam_z`, `point_x, point_y, point_z`.
- Validity depends on LiDAR validity; reflector mode also requires configured `reflector_height_m`.

## Note
No terrain mesh/DEM is generated here; output is structured measurement data for downstream mapping.
