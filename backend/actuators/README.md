# Actuators Module

## Purpose
Actuator interfaces for the scanning head: pan/tilt cycloidal actuation, encoder readings, and IMU orientation.

## Files
- `cycloidal_actuator.py` - `CycloidalActuator`, `ActuatorCommand`: single-axis cycloidal actuator abstraction (encoder feedback + driver command interface, angle limits, stop).
- `turret.py` - `PanTiltTurret`: 2-axis pan/tilt turret integrating two cycloidal actuators; computes tracking commands from camera pixel error.
- `readings.py` - `ActuatorReadings`: combines pan and tilt `QuadratureEncoder` instances.
- `imu_rpy.py` - `ImuOrientation`, `ImuReading`: reads BNO055 roll/pitch/yaw/temperature.
- `2_axis_turret.py` - standalone CAN servo control GUI/utility (legacy/manual utility, not main integrated loop).

## Pseudocode
```text
CycloidalActuator.move_to_angle(target):
  clamp target to min/max
  driver.move_to_angle(target)
  return command + actual angle

PanTiltTurret.track_reference(x_error_px, y_error_px):
  if reference not detected: return tracking_active=false
  if error outside deadband: pan/tilt delta = error * gain
  command pan/tilt cycloidal actuators
  return tracking deltas/targets

ActuatorReadings.read(): pan/tilt counts+degrees
ImuOrientation.read(): roll, pitch, yaw, temperature
```

## Integration note
`SurfaceMappingSystem` initializes `PanTiltTurret` with two cycloidal actuators and calls `track_reference()` during `collect_measurements()`. Real physical motion requires injecting a hardware driver into `CycloidalActuator(driver=...)` and calibrating gains/deadband/limits.
