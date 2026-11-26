# Data Module

## Purpose
Contains the integration/orchestration and data transforms that turn raw sensor readings into terrain-ready 3D point variables.

## Files
- `main.py` - `SurfaceMappingSystem` orchestration, config, logging, odometry placeholder, and cycloidal turret tracking integration.
- `processing.py` - `combine_readings()` flattens component readings into one record.
- `coordinates.py` - `calculate_robot_elevation()` for reflector-based `z_robot = H_ref - d*sin(theta_v)` where `theta_v = tilt + IMU pitch + mounting offset`; `calculate_coordinates()` for legacy beam projection.
- `scan_area.py` - shared field/block sizes, per-block scan counts, and editable calibration.
- `manual_scan.py` - manual reference-tracked block progression and scan acceptance.
- `terrain_plots.py` - shared measured-height/coverage plotting for simulated and real scans.
- `terrain_simulation.py` - configurable field/square scan simulation; plots measured terrain after each completed square and saves one PNG per scan.
- `session_store.py` - persisted scan sessions with blocks, points, partial progress, and frontend-compatible JSON.

## OOP/pseudocode
```text
SurfaceMappingSystem.run():
  init hardware + logger
  if manual_scan:
    for each block:
      optionally edit scan area and calibration
      for each configured scan:
        record = acquire + process_measurement()
        accept when reference, lidar, and odometry are valid
        save(record + block/calibration metadata)
      plot accepted points for the completed block
  else:
    for each cycle:
      record = combine_readings(all sensors)
      record = process_measurement(record)
      if record: save(record)
  close()
```

## Derived variables
- Reflector mode (primary): `z_robot_m = H_ref - d*sin(theta_v)`, plus `distance_to_reflector_m`, `theta_h_deg`, `theta_v_deg` (world-referenced), `theta_v_mechanical_deg`, `imu_pitch_deg`, `tilt_mount_offset_deg`, horizontal/reflector-local components.
- Tracking vars: `pan/tilt_target_angle`, `tracking_active`, `pan/tilt_delta_deg`, `pan/tilt_command_clamped`.
- Legacy beam mode: `beam_x, beam_y, beam_z`, `point_x, point_y, point_z`.
- Manual scan tags: `block_row`, `block_col`, `block_scan_index`, `accepted_for_scan`, `calibration_version`, `field_x_m`, `field_y_m`, `block_size_m`, `scan_session`, `pose_source`.
- Validity depends on LiDAR validity; reflector mode also requires configured `reflector_height_m`.

## Note
The acquisition path generates no continuous terrain mesh/DEM; its output is structured measurement data for downstream mapping. `python3 -m backend.data.main --simulate --show` writes a simulated grid (default 70 m x 100 m, 5 m spacing, seed 14, sensor noise on) to CSV, then reads `point_x/point_y`, `measured_height_mm` and `error_mm` back to plot three panels and saves the chart as `backend/data/scan_plots/csv_plot_TIMESTAMP.png`. The plotted Z axis is reconstructed height in millimetres derived from the pitch/elevation angle and slant range; the angle itself is used internally and is not plotted. Manual block mode plots accepted scan points after each completed block and can persist them to `backend/data/sessions/` with `--session-id` for the frontend. `terrain_simulation.py` generates incremental terrain plots from a simulated true surface plus measured scan points.

## Manual block scanning
```bash
python3 -m backend.data.main --simulate --manual-scan --x-length 10 --y-length 10 --block-size 5 --points-per-block 2 --reflector-height 1.5 --no-prompts
```

Field size, block size, and per-block scans are set before the run. Prompts before each block can change scan area and calibration; `Enter` measures, `s` skips the block, and `q` quits. `--no-prompts` auto-accepts samples for simulation/testing. Hardware mode still requires a Raspberry Pi.

## Terrain simulation
```bash
python3 -m backend.data.terrain_simulation --x-length 120 --y-length 56 --square-size 8 --points-per-square 4 --seed 14
```

A full scan is one completed field square. Use `--points-grid "4 4;4 4"` with a 2x2 square grid for per-square counts, `--show` to display each plot, and `--noise-on` to include sensor noise.
