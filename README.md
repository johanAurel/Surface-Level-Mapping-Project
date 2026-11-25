# Surface Level Mapping Project

## Overview
This project implements the described reflector-based field elevation survey concept in software structure: a wheeled robot with camera+LiDAR+IMU+2-axis turret tracks a fixed reflector, logs measurements, and derives robot elevation/terrain-ready variables. A separate simulation renders incremental terrain plots from the same reflector geometry.

**Important:** `SurfaceMappingSystem` derives terrain-ready data (`point_x/y/z`, beam vectors, pose, range, tracking errors) and, in manual block mode, plots accepted scan points. It does **not** generate a continuous terrain mesh/DEM. Terrain simulation plots are handled by `backend/data/terrain_simulation.py`.

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
- `backend/data/scan_area.py` - shared field/block and calibration configuration
- `backend/data/manual_scan.py` - manual per-block scan controller
- `backend/data/terrain_plots.py` - shared measured-height/coverage plotting

Standalone demos/utilities (kept, not the main integrated loop): `backend/sensors/camera/*.py`, `backend/sensors/lidar/main.py`, `backend/actuators/2_axis_turret.py`.


## Alignment with described system (image description)
- **Camera**: detects/tracks reflector/fiducial (`CameraTracker`, `reference_detected`, pixel errors).
- **Single-point LiDAR**: measures distance `d` (`lidar_distance`).
- **IMU**: provides roll/pitch/yaw (`ImuOrientation`).
- **2-axis turret**: pan/tilt angles `theta_h/theta_v` from actuator encoders; `PanTiltTurret` + `CycloidalActuator` provide integrated cycloidal pan/tilt tracking commands from camera pixel error.
- **Raspberry Pi/controller**: `SurfaceMappingSystem` orchestrates read/track/compute/log.
- **Primary elevation formula**: `z_robot = H_ref - d*sin(theta_v)`, with `theta_v = turret_tilt + IMU_pitch + tilt_mount_offset_deg`, implemented in `backend/data/coordinates.py:calculate_robot_elevation()` and used by `SurfaceMappingSystem` in `reflector_elevation` mode.
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
- **Scan workflow**: `block_row`, `block_col`, `block_scan_index`, `accepted_for_scan`, `calibration_version`, `field_x_m`, `field_y_m`, `block_size_m`, `scan_session`, `pose_source`
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

Without `--simulate` (run this on the Pi): the program requires a Raspberry Pi, validates the configured BCM pins for duplicates/range, tests each GPIO pin, constructs the devices, reads every sensor once, then starts the logging loop. Any failed check aborts with an `error:` message.

On a machine without Raspberry Pi hardware (e.g. WSL), use simulation mode:
```bash
python3 -m backend.data.main --simulate --reflector-height 1.5
```

Simulation defaults to `h_ref = 1.5 m` (override with `--reflector-height`) and simulates odometry so `x/y` are populated. Each sample prints `t, x, y, d, theta_v` with its tilt/pitch/offset breakdown, `theta_h, yaw, h_ref, z`; the CSV contains all configured fields:
```text
[simulate]   1 t=2026-11-23T15:35:32.601349+00:00 x=0.000 y=-0.000 d=2.482 theta_v=45.900 (tilt=45.000 pitch=0.900 offset=0.000) theta_h=0.000 yaw=-3.203 h_ref=1.500 z=-0.269
```
Simulation runs 20 samples and prints progress by default; pass `--cycles N` to change it (use `--cycles 0` for a continuous stream, stopped with Ctrl+C). Real hardware mode runs continuously unless `--cycles N` is given.

Useful flags: `--simulate`, `--cycles N`, `--output PATH`, `--reflector-height METRES`, `--show` to simulate a grid, plot the written CSV and save a PNG.

Output CSV default for `backend/data/main.py`: `backend/data/surface_measurements.csv`.

To simulate values into CSV, then plot that CSV and store the chart without the frontend:

```bash
python3 -m backend.data.main --simulate --show
```

In this mode the simulator writes a full grid to CSV (default 70 m x 100 m field, 5 m spacing, seed 14, noise off), then reads the CSV back to plot measured height in 3D/2D and measurement error as three panels. The PNG is saved under `backend/data/scan_plots/` as `csv_plot_TIMESTAMP.png` and displayed. Use `--grid-step METRES`, `--sensor-height METRES`, `--noise-on`, `--noise-scale`, `--seed`, `--output PATH` and `--scan-plots-dir PATH` to change the simulation and outputs; `--cycles` applies only to continuous sample simulation without `--show`.

## Manual block scanning
`backend/data/main.py --manual-scan` runs reference-tracked scans block by block. Set field size and block size before starting; a 100 x 100 m field with 5 m blocks becomes a 20 x 20 block grid. Each block asks for a configured number of accepted scans, then saves a plot to `backend/data/scan_plots/` before moving to the next block.

```bash
python3 -m backend.data.main --simulate --manual-scan --x-length 10 --y-length 10 --block-size 5 --points-per-block 2 --reflector-height 1.5 --no-prompts
```

Hardware mode still requires a Raspberry Pi. In normal manual mode, prompts let you change scan area and calibration before each block, then `Enter` measures, `s` skips the block, and `q` quits. `--no-prompts` auto-accepts samples for simulation/testing. Use `--points-grid` for per-block counts and `--scan-plots-dir PATH` to change plot output.

Pass `--session-id ID` to stream accepted points into a frontend scan session. The ID is shown in the frontend header; select the same session there to watch the field fill live.

## Frontend scan console
A dependency-free console lives in `frontend/`. Start its local API server and open the printed URL:

```bash
python3 -m backend.web.server
```

Then open `http://127.0.0.1:8765`. The console lets you:
- record and edit field scale, block size, scan counts, reflector height, wheel circumference, and initial pose;
- watch the field fill block by block, with live point dots and depth heat colors;
- click, shift-click, ctrl-click, or drag to select one block or a block group;
- mark selected blocks for rescan (consumed by the next backend run), clear their points, add demo scans for testing, and edit or delete individual variables;
- view depth-by-block and selected-block depth charts;
- save drafts, resume unfinished sessions, or start a completely new scan.

Sessions are persisted as JSON under `backend/data/sessions/`. Use `Add demo scans` to verify the UI without hardware. For live hardware/simulator integration from Python:

```bash
python3 -m backend.web.server
python3 -m backend.data.main --simulate --manual-scan --session-id <id-from-frontend> --x-length 10 --y-length 10 --block-size 5 --points-per-block 2 --no-prompts
```

## Terrain scan simulation
`backend/data/terrain_simulation.py` uses the reflector geometry to simulate scan points and plots the terrain after every full scan, where one full scan means one square of the field is completed.

```bash
python3 -m backend.data.terrain_simulation --x-length 120 --y-length 56 --square-size 8 --points-per-square 4 --seed 14
```

Defaults: 120 m x 56 m field, 8 m squares (15 x 7), 4 points per square, seed 14, noise off. PNGs are written to `backend/data/terrain_scans/scan_NN_rRR_cCC.png`. Use `--show` for live plots and `--noise-on --noise-scale 1` for noisy measurements. For different counts per square, use `--points-grid` with one row per square row; for example, add `--x-length 16 --y-length 16 --points-grid "4 4;4 4"` for a 2x2 grid. The square size must divide both field lengths exactly; for manual surveys choose the field, square grid, point counts, and keep the seed fixed so runs are repeatable.

## Configuration / calibration notes
Edit `MappingConfig` in `backend/data/main.py`, or pass the matching CLI flags:
- GPIO pins and encoder counts/rev
- `wheel_circumference_m` for odometry scale; editable before each manual block
- reflector x/y/z and initial machine x/y/yaw for field-frame pose
- `tilt_mount_offset_deg` for fixed LiDAR/tilt mounting error (IMU pitch is added automatically)
- sampling interval and output file
- `require_reference_detection` if camera lock is mandatory

`PlatformPositionEstimator` uses calibrated initial pose, wheel travel, and IMU yaw for horizontal x/y. IMU pitch is used for elevation/tilt correction; it does not by itself determine horizontal position. Without real localization, odometry-derived x/y remains approximate.

## Repository layout
- `backend/` - integrated backend, sensor/actuator modules, demos
- `backend/data/` - orchestration, fusion, coordinate transforms
- `backend/sensors/` - LiDAR/camera/encoder interfaces
- `backend/actuators/` - IMU and actuator interfaces
- `backend/encoders/` - quadrature encoder abstraction
- `backend/web/` - local HTTP/SSE API for the frontend
- `backend/data/sessions/` - persisted scan sessions
- `frontend/` - dependency-free scan console UI

## Status / limitations
- Core Python modules compile (`python3 -m py_compile` on backend `.py` files).
- Hardware execution requires Raspberry Pi dependencies/devices (GPIO, serial, I2C, camera); use `--simulate` elsewhere.
- I cannot inspect `/mnt/c/Users/user/Downloads/machine.png` in this environment (image input unsupported by current model), so comparison to that diagram must be done manually against the derived-variable list above.
