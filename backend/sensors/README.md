# Sensors Module

## Purpose
Sensor interfaces and sensor demos for the Surface Level Mapping project.

## Integrated sensor classes
- `encoder_readings.py` - `SensorEncoder`: external/wheel encoder counts+degrees.
- `lidar_sensor.py` - `LidarSensor`, `LidarReading`: TFmini serial frame parsing and distance/strength.
- `camera_tracker.py` - `CameraTracker`, `CameraTrackingReading`: HSV reference detection and pixel error from frame center.

## Demos / legacy scripts
- `camera/basic_picamera.py` - camera bring-up demo.
- `camera/hsv_color_filter.py` - HSV mask demo.
- `camera/color_detection_trackbars.py` - interactive HSV tuning demo.
- `camera/pan_tilt_tracking.py` - demo combining camera detection + servo movement.
- `lidar/main.py` - basic TFmini serial read demo.

## Pseudocode
```text
SensorEncoder.read(): counts, degrees
LidarSensor.read(): parse TFmini frame -> distance_m, strength, valid
CameraTracker.read(): capture frame -> HSV mask -> largest contour -> center/error
```

## Integration note
`SurfaceMappingSystem` uses the OOP classes above; demo scripts are useful for hardware validation but are not the primary data pipeline.
