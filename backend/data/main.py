"""OOP orchestration for the Surface Level Mapping project.

PSEUDOCODE (system):
    CONFIGURE hardware pins, calibration, output CSV
    INITIALIZE actuators, IMU, LiDAR, camera, encoder, logger
    LOOP at fixed sampling interval:
        timestamp = now()
        actuator_data = read pan/tilt
        encoder_data = read external encoder
        imu_data = read roll/pitch/yaw
        lidar_data = read distance
        camera_data = read reference detection/tracking error
        position_data = estimate platform x/y/z
        measurement = combine_all(...)
        point = calculate_coordinates(measurement) if valid
        WRITE measurement+point to CSV
    ON EXIT: close all devices/files

This derives terrain-ready variables, plots each completed manual block, and can persist accepted points into a scan session for the frontend.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from backend.data.coordinates import calculate_coordinates, calculate_robot_elevation
from backend.data.manual_scan import ManualScanController
from backend.data.processing import combine_readings
from backend.data.scan_area import CalibrationConfig, ScanAreaConfig
from backend.data import session_store
from backend.data.terrain_plots import TerrainPlotter
from backend.data.terrain_simulation import ScanConfig as SimGridConfig, TerrainScanSimulator


def _fmt(value: Any) -> str:
    if value is None:
        return "n/a"
    try:
        return f"{float(value):.3f}"
    except (TypeError, ValueError):
        return str(value)


@dataclass
class MappingConfig:
    pan_a: int = 17
    pan_b: int = 27
    tilt_a: int = 22
    tilt_b: int = 23
    sensor_a: int = 5
    sensor_b: int = 6
    pan_counts_per_rev: int = 1024
    tilt_counts_per_rev: int = 1024
    sensor_counts_per_rev: int = 1024
    wheel_circumference_m: float = 0.0  # set from physical wheel for odometry
    sampling_interval_s: float = 0.1
    output_file: Path = Path("backend/data/surface_measurements.csv")
    require_reference_detection: bool = False
    reflector_height_m: float = 0.0  # H_ref, known surveyed reflector height
    mode: str = "reflector_elevation"  # reflector_elevation | beam_projection
    enable_turret_tracking: bool = True
    tracking_deadband_px: float = 20.0
    pan_gain_deg_per_px: float = 0.01
    tilt_gain_deg_per_px: float = 0.01
    tilt_mount_offset_deg: float = 0.0
    simulate: bool = False
    manual_scan: bool = False
    area: ScanAreaConfig = field(default_factory=ScanAreaConfig)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    scan_plots_dir: Path | None = Path("backend/data/scan_plots")
    show_plots: bool = False
    sim_grid_step_m: float = 5.0
    sim_sensor_height_m: float = 1.2
    sim_noise_on: bool = False
    sim_noise_scale: float = 1.0
    sim_seed: int | None = 14
    calibration_prompts: bool = True
    accept_prompts: bool = True
    session_id: str | None = None


@dataclass
class _SimEncoder:
    counts_per_rev: int
    phase: float = 0.0
    _start: float = field(default_factory=time.monotonic)

    def read_counts(self) -> int:
        degrees = math.sin((time.monotonic() - self._start) * 0.5 + self.phase) * 180.0
        return int(degrees / 360.0 * self.counts_per_rev)

    def read_degrees(self) -> float:
        return self.read_counts() / self.counts_per_rev * 360.0

    def close(self) -> None:
        pass


@dataclass
class _SimActuatorReadings:
    pan_encoder: _SimEncoder
    tilt_encoder: _SimEncoder

    def read(self) -> dict[str, Any]:
        return {
            "pan_counts": self.pan_encoder.read_counts(),
            "pan_degrees": self.pan_encoder.read_degrees(),
            "tilt_counts": self.tilt_encoder.read_counts(),
            "tilt_degrees": self.tilt_encoder.read_degrees(),
        }

    def close(self) -> None:
        self.pan_encoder.close()
        self.tilt_encoder.close()


@dataclass
class _SimSensorEncoder:
    encoder: _SimEncoder

    def read(self) -> dict[str, Any]:
        return {
            "sensor_encoder_counts": self.encoder.read_counts(),
            "sensor_encoder_degrees": self.encoder.read_degrees(),
        }

    def close(self) -> None:
        self.encoder.close()


@dataclass
class _SimImuReading:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    temperature_c: float


class _SimImu:
    def read(self) -> _SimImuReading:
        t = time.monotonic()
        return _SimImuReading(
            roll_deg=math.sin(t * 0.3) * 2.0,
            pitch_deg=math.cos(t * 0.3) * 2.0,
            yaw_deg=math.sin(t * 0.1) * 10.0,
            temperature_c=25.0,
        )

    def close(self) -> None:
        pass


@dataclass
class _SimLidarReading:
    distance_m: float | None
    strength: int
    quality: int
    valid: bool


class _SimLidar:
    def read(self) -> _SimLidarReading:
        distance = 2.0 + math.sin(time.monotonic() * 0.4) * 0.5
        return _SimLidarReading(distance_m=distance, strength=500, quality=500, valid=True)

    def close(self) -> None:
        pass


class _SimCamera:
    def read(self) -> dict[str, Any]:
        t = time.monotonic()
        x_error = math.sin(t) * 40.0
        y_error = math.cos(t) * 30.0
        return {
            "reference_detected": True,
            "reference_x_px": 360.0 + x_error,
            "reference_y_px": 180.0 + y_error,
            "x_error_px": x_error,
            "y_error_px": y_error,
            "area_px": 500.0,
            "frame_width": 720,
            "frame_height": 360,
        }

    def close(self) -> None:
        pass


@dataclass
class PlatformPositionEstimator:
    """Calibrated odometry placeholder using wheel travel and IMU yaw."""

    wheel_circumference_m: float = 0.0
    start_x_m: float = 0.0
    start_y_m: float = 0.0
    start_yaw_deg: float = 0.0
    _start_degrees: float | None = None

    def apply_pose(self, calibration: CalibrationConfig) -> None:
        self.start_x_m = calibration.start_x_m
        self.start_y_m = calibration.start_y_m
        self.start_yaw_deg = calibration.start_yaw_deg

    def apply_calibration(self, calibration: CalibrationConfig) -> None:
        calibration.validate(require_reflector=False)
        self.apply_pose(calibration)
        self.wheel_circumference_m = calibration.wheel_circumference_m
        self._start_degrees = None

    def read(self, sensor_encoder_degrees: float | None, imu: dict[str, Any]) -> dict[str, Any]:
        degrees = 0.0 if sensor_encoder_degrees is None else float(sensor_encoder_degrees)
        if self._start_degrees is None:
            self._start_degrees = degrees
        delta_degrees = degrees - self._start_degrees
        revolutions = delta_degrees / 360.0
        travel_m = revolutions * self.wheel_circumference_m if self.wheel_circumference_m > 0 else None

        yaw = imu.get("yaw")
        yaw_rad = math.radians(float(yaw) - self.start_yaw_deg) if yaw is not None else None
        platform_x = (
            self.start_x_m + travel_m * math.cos(yaw_rad)
            if travel_m is not None and yaw_rad is not None
            else None
        )
        platform_y = (
            self.start_y_m + travel_m * math.sin(yaw_rad)
            if travel_m is not None and yaw_rad is not None
            else None
        )

        return {
            "platform_x": platform_x,
            "platform_y": platform_y,
            "platform_z": 0.0,
            "platform_travel_m": travel_m,
            "odometry_available": travel_m is not None,
        }


@dataclass
class SurfaceMapDataLogger:
    output_file: Path
    fields: list[str]
    _file: Any = None
    _writer: csv.DictWriter | None = None

    def open(self) -> None:
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_file.open("w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fields, extrasaction="ignore")
        self._writer.writeheader()

    def write(self, measurement: dict[str, Any]) -> None:
        if self._writer is None or self._file is None:
            raise RuntimeError("Logger not open")
        self._writer.writerow(measurement)
        self._file.flush()

    def close(self) -> None:
        if self._file:
            self._file.close()
        self._file = None
        self._writer = None


class SurfaceMappingSystem:
    """High-level integration object for one scanning session."""

    def __init__(self, config: MappingConfig | None = None):
        self.config = config or MappingConfig()
        self.actuators: Any = None
        self.sensor_encoder: Any = None
        self.imu: Any = None
        self.lidar: Any = None
        self.camera: Any = None
        self.turret: Any = None
        self.calibration = self.config.calibration
        self.manual_scan: ManualScanController | None = None
        self.plotter: TerrainPlotter | None = None
        wheel_circumference = self.config.wheel_circumference_m
        if self.config.simulate and wheel_circumference <= 0:
            wheel_circumference = 1.0
        self.position_estimator = PlatformPositionEstimator(wheel_circumference)
        self.logger = SurfaceMapDataLogger(self.config.output_file, self.output_fields())
        self._running = False

    @staticmethod
    def output_fields() -> list[str]:
        return [
            "timestamp",
            "pan_counts",
            "pan_angle",
            "tilt_counts",
            "tilt_angle",
            "pan_target_angle",
            "tilt_target_angle",
            "tracking_active",
            "pan_delta_deg",
            "tilt_delta_deg",
            "pan_command_clamped",
            "tilt_command_clamped",
            "sensor_encoder_counts",
            "sensor_encoder_degrees",
            "roll",
            "pitch",
            "yaw",
            "temperature_c",
            "lidar_distance",
            "lidar_quality",
            "lidar_strength",
            "lidar_valid",
            "reference_detected",
            "reference_x_px",
            "reference_y_px",
            "x_error_px",
            "y_error_px",
            "area_px",
            "platform_x",
            "platform_y",
            "platform_z",
            "platform_travel_m",
            "odometry_available",
            "reflector_height_m",
            "distance_to_reflector_m",
            "theta_v_deg",
            "theta_h_deg",
            "theta_v_mechanical_deg",
            "imu_pitch_deg",
            "tilt_mount_offset_deg",
            "horizontal_distance_to_reflector_m",
            "reflector_dx_local_m",
            "reflector_dy_local_m",
            "z_robot_m",
            "beam_x",
            "beam_y",
            "beam_z",
            "point_x",
            "point_y",
            "point_z",
            "terrain_height_mm",
            "measured_height_mm",
            "error_mm",
            "block_row",
            "block_col",
            "block_scan_index",
            "accepted_for_scan",
            "calibration_version",
            "field_x_m",
            "field_y_m",
            "block_size_m",
            "grid_step_m",
            "scan_session",
            "pose_source",
            "reflector_x_m",
            "reflector_y_m",
            "start_x_m",
            "start_y_m",
            "start_yaw_deg",
            "wheel_circumference_m",
        ]

    @staticmethod
    def require_raspberry_pi() -> None:
        model = Path("/proc/device-tree/model")
        if model.exists():
            try:
                if "Raspberry Pi" in model.read_text(errors="replace"):
                    return
            except OSError:
                pass
        raise RuntimeError("Real hardware mode requires a Raspberry Pi; use --simulate on this machine.")

    def gpio_pins(self) -> dict[str, int]:
        return {
            "pan_a": self.config.pan_a,
            "pan_b": self.config.pan_b,
            "tilt_a": self.config.tilt_a,
            "tilt_b": self.config.tilt_b,
            "sensor_a": self.config.sensor_a,
            "sensor_b": self.config.sensor_b,
        }

    def validate_pin_config(self) -> None:
        used: dict[int, str] = {}
        for name, pin in self.gpio_pins().items():
            if not isinstance(pin, int) or not 0 <= pin <= 27:
                raise ValueError(f"invalid BCM GPIO pin for {name}: {pin!r}")
            if pin in used:
                raise ValueError(f"BCM GPIO {pin} assigned to both {used[pin]} and {name}")
            used[pin] = name
        print(f"[hardware] pin config OK: {used}")

    def verify_gpio_pins(self) -> None:
        from gpiozero import InputDevice

        for name, pin in self.gpio_pins().items():
            device = InputDevice(pin, pull_up=True)
            try:
                device.value
            finally:
                device.close()
            print(f"[hardware] BCM GPIO {pin} ({name}): OK")

    def verify_devices(self) -> None:
        checks = (
            ("pan/tilt encoders", self.actuators.read),
            ("platform encoder", self.sensor_encoder.read),
            ("IMU", self.imu.read),
            ("LiDAR", self.lidar.read),
            ("camera", self.camera.read),
        )
        for name, read in checks:
            try:
                value = read()
            except Exception as exc:
                raise RuntimeError(f"hardware check failed for {name}: {exc}") from exc
            print(f"[hardware] {name}: OK ({value})")

    def initialise_hardware(self) -> None:
        # PSEUDOCODE: lazy-import hardware adapters, then construct with configured pins/calibration.
        from backend.actuators.cycloidal_actuator import CycloidalActuator
        from backend.actuators.turret import PanTiltTurret

        if self.config.simulate:
            self.actuators = _SimActuatorReadings(
                pan_encoder=_SimEncoder(self.config.pan_counts_per_rev),
                tilt_encoder=_SimEncoder(self.config.tilt_counts_per_rev, math.asin(0.25)),
            )
            self.sensor_encoder = _SimSensorEncoder(_SimEncoder(self.config.sensor_counts_per_rev))
            self.imu = _SimImu()
            self.lidar = _SimLidar()
            self.camera = _SimCamera()
        else:
            from backend.actuators.imu_rpy import ImuOrientation
            from backend.actuators.readings import ActuatorReadings
            from backend.sensors.camera_tracker import CameraTracker
            from backend.sensors.encoder_readings import SensorEncoder
            from backend.sensors.lidar_sensor import LidarSensor

            self.actuators = ActuatorReadings(
                pan_pin_a=self.config.pan_a,
                pan_pin_b=self.config.pan_b,
                tilt_pin_a=self.config.tilt_a,
                tilt_pin_b=self.config.tilt_b,
                pan_counts_per_rev=self.config.pan_counts_per_rev,
                tilt_counts_per_rev=self.config.tilt_counts_per_rev,
            )
            self.sensor_encoder = SensorEncoder(
                pin_a=self.config.sensor_a,
                pin_b=self.config.sensor_b,
                counts_per_rev=self.config.sensor_counts_per_rev,
            )
            self.imu = ImuOrientation()
            self.lidar = LidarSensor()
            self.camera = CameraTracker()

        # PSEUDOCODE: integrate 2-axis cycloidal turret; drivers can be injected later.
        pan_axis = CycloidalActuator("pan", encoder=getattr(self.actuators, "pan_encoder", None), min_angle_deg=-180, max_angle_deg=180)
        tilt_axis = CycloidalActuator("tilt", encoder=getattr(self.actuators, "tilt_encoder", None), min_angle_deg=-30, max_angle_deg=90)
        self.turret = PanTiltTurret(
            pan_axis,
            tilt_axis,
            pan_gain_deg_per_px=self.config.pan_gain_deg_per_px,
            tilt_gain_deg_per_px=self.config.tilt_gain_deg_per_px,
            deadband_px=self.config.tracking_deadband_px,
        )

    def read_actuators(self) -> dict[str, Any]:
        # PSEUDOCODE: sample pan/tilt encoders and normalize keys for fusion.
        assert self.actuators is not None
        data = self.actuators.read()
        return {
            "pan_counts": data["pan_counts"],
            "pan_angle": data["pan_degrees"],
            "tilt_counts": data["tilt_counts"],
            "tilt_angle": data["tilt_degrees"],
        }

    def read_sensor_encoder(self) -> dict[str, Any]:
        # PSEUDOCODE: sample external encoder for platform movement/odometry input.
        assert self.sensor_encoder is not None
        return self.sensor_encoder.read()

    def read_imu(self) -> dict[str, Any]:
        # PSEUDOCODE: read fused orientation and temperature from IMU.
        assert self.imu is not None
        reading = self.imu.read()
        return {
            "roll": reading.roll_deg,
            "pitch": reading.pitch_deg,
            "yaw": reading.yaw_deg,
            "temperature_c": reading.temperature_c,
        }

    def read_lidar(self) -> dict[str, Any]:
        # PSEUDOCODE: read one distance frame and expose validity/quality.
        assert self.lidar is not None
        reading = self.lidar.read()
        return {
            "lidar_distance": reading.distance_m,
            "lidar_quality": reading.quality,
            "lidar_strength": reading.strength,
            "lidar_valid": reading.valid,
        }

    def read_camera(self) -> dict[str, Any]:
        # PSEUDOCODE: detect reference target and compute pixel tracking error.
        assert self.camera is not None
        return self.camera.read()

    def read_platform_position(self, encoder_data: dict[str, Any], imu_data: dict[str, Any]) -> dict[str, Any]:
        return self.position_estimator.read(encoder_data.get("sensor_encoder_degrees"), imu_data)

    def collect_measurements(self) -> dict[str, Any]:
        # PSEUDOCODE: acquire sensors, track reference with turret, estimate pose, flatten into one record.
        timestamp = datetime.now(timezone.utc).isoformat()
        actuator_data = self.read_actuators()
        encoder_data = self.read_sensor_encoder()
        imu_data = self.read_imu()
        lidar_data = self.read_lidar()
        camera_data = self.read_camera()
        tracking_data: dict[str, Any] = {"tracking_active": False}
        if self.config.enable_turret_tracking and self.turret is not None:
            tracking_data = self.turret.track_reference(camera_data.get("x_error_px"), camera_data.get("y_error_px"))
            # PSEUDOCODE: refresh angles after command; turret/command values fill missing encoder feedback.
            actuator_data = {**self.turret.read(), **self.read_actuators(), **self.turret.read()}
            # Ensure tilt/pan targets are always present for downstream logging/derivation.
            actuator_data.setdefault("pan_target_angle", tracking_data.get("pan_command_target_deg"))
            actuator_data.setdefault("tilt_target_angle", tracking_data.get("tilt_command_target_deg"))
        elif self.turret is not None:
            actuator_data = {**actuator_data, **self.turret.read()}
        position_data = self.read_platform_position(encoder_data, imu_data)
        return combine_readings(
            timestamp=timestamp,
            actuators=actuator_data,
            encoder=encoder_data,
            imu=imu_data,
            lidar=lidar_data,
            camera=camera_data,
            position=position_data | tracking_data,
        )

    def process_measurement(self, measurement: dict[str, Any]) -> dict[str, Any] | None:
        # PSEUDOCODE: validate measurement, compute elevation/point variables by configured mode.
        if not measurement.get("lidar_valid") or measurement.get("lidar_distance") is None:
            return None
        if self.config.require_reference_detection and not measurement.get("reference_detected"):
            return None

        # PSEUDOCODE: primary mode matches described reflector-based field elevation survey.
        if self.config.mode == "reflector_elevation":
            if self.config.reflector_height_m <= 0:
                # Keep schema stable; elevation cannot be derived without H_ref.
                measurement.update({
                    "reflector_height_m": self.config.reflector_height_m,
                    "distance_to_reflector_m": measurement.get("lidar_distance"),
                    "theta_v_deg": measurement.get("tilt_angle"),  # tilt accounted for
                    "theta_h_deg": measurement.get("pan_angle"),
                    "theta_v_mechanical_deg": measurement.get("tilt_angle"),
                    "imu_pitch_deg": measurement.get("pitch"),
                    "tilt_mount_offset_deg": self.config.tilt_mount_offset_deg,
                    "z_robot_m": None,
                })
                return measurement

            elev = calculate_robot_elevation(
                measurement,
                self.config.reflector_height_m,
                imu_pitch_deg=measurement.get("pitch"),
                tilt_mount_offset_deg=self.config.tilt_mount_offset_deg,
            )
            measurement.update(elev)
            # Explicitly keep tilt/pan used for derivation visible in the record.
            measurement["theta_v_deg"] = elev["theta_v_deg"]
            measurement["theta_h_deg"] = elev["theta_h_deg"]
            # Map to common output names for terrain/elevation map generation.
            measurement["point_x"] = measurement.get("platform_x")
            measurement["point_y"] = measurement.get("platform_y")
            measurement["point_z"] = elev["z_robot_m"]
            return measurement

        # PSEUDOCODE: legacy beam-projection mode (scan beam to 3D point).
        if any(measurement.get(k) is None for k in ("platform_x", "platform_y", "platform_z")):
            measurement.update({"beam_x": None, "beam_y": None, "beam_z": None, "point_x": None, "point_y": None, "point_z": None})
            return measurement
        point = calculate_coordinates(measurement)
        measurement.update(point)
        measurement["point_x"] = point["x"]
        measurement["point_y"] = point["y"]
        measurement["point_z"] = point["z"]
        return measurement

    def close(self) -> None:
        # PSEUDOCODE: stop motion first, then release sensors and logger.
        if self.turret is not None:
            try:
                self.turret.stop()
            except Exception:
                pass
        for component in (self.actuators, self.sensor_encoder, self.imu, self.lidar, self.camera):
            close = getattr(component, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass
        self.logger.close()

    @staticmethod
    def print_summary(index: int, measurement: dict[str, Any]) -> None:
        print(
            f"[simulate] {index:>3} "
            f"t={measurement.get('timestamp')} "
            f"x={_fmt(measurement.get('platform_x'))} "
            f"y={_fmt(measurement.get('platform_y'))} "
            f"d={_fmt(measurement.get('lidar_distance'))} "
            f"theta_v={_fmt(measurement.get('theta_v_deg'))} "
            f"(tilt={_fmt(measurement.get('theta_v_mechanical_deg'))} "
            f"pitch={_fmt(measurement.get('imu_pitch_deg'))} "
            f"offset={_fmt(measurement.get('tilt_mount_offset_deg'))}) "
            f"theta_h={_fmt(measurement.get('theta_h_deg'))} "
            f"yaw={_fmt(measurement.get('yaw'))} "
            f"h_ref={_fmt(measurement.get('reflector_height_m'))} "
            f"z={_fmt(measurement.get('z_robot_m'))}"
        )

    def apply_calibration(
        self, calibration: CalibrationConfig, require_reflector: bool = True
    ) -> None:
        calibration.validate(require_reflector=require_reflector)
        self.calibration = calibration
        self.config.reflector_height_m = calibration.reflector_z_m
        self.config.tilt_mount_offset_deg = calibration.tilt_mount_offset_deg
        self.position_estimator.apply_calibration(calibration)

    def session_config(self) -> dict[str, Any]:
        area = self.config.area
        calibration = self.calibration
        counts = area.points_per_block
        return {
            "x_length_m": area.x_length_m,
            "y_length_m": area.y_length_m,
            "block_size_m": area.block_size_m,
            "points_per_block": counts.tolist() if isinstance(counts, np.ndarray) else int(counts),
            "reflector_height_m": calibration.reflector_z_m,
            "wheel_circumference_m": calibration.wheel_circumference_m,
            "start_x_m": calibration.start_x_m,
            "start_y_m": calibration.start_y_m,
            "start_yaw_deg": calibration.start_yaw_deg,
            "tilt_mount_offset_deg": calibration.tilt_mount_offset_deg,
        }

    def _ensure_session(self) -> None:
        if not self.config.manual_scan or not self.config.session_id:
            return
        config = self.session_config()
        try:
            session_store.load_session(self.config.session_id)
        except FileNotFoundError:
            session_store.create_session(
                config, name=f"Field scan {self.config.session_id}", session_id=self.config.session_id
            )
        else:
            session_store.reconfigure_session(self.config.session_id, config)

    def _prepare_run(self, max_cycles: int | None) -> int | None:
        if not self.config.simulate:
            self.require_raspberry_pi()
            self.validate_pin_config()
            self.verify_gpio_pins()
        if max_cycles is not None and max_cycles <= 0:
            max_cycles = None
        elif self.config.simulate and not self.config.manual_scan and max_cycles is None:
            max_cycles = 20
        self.initialise_hardware()
        self._running = True
        if self.calibration.wheel_circumference_m > 0:
            self.apply_calibration(self.calibration, require_reflector=False)
        else:
            self.position_estimator.apply_pose(self.calibration)
        if self.config.manual_scan:
            self._ensure_session()
            if self.config.session_id:
                session_store.consume_rescan_requests(self.config.session_id)
            self.manual_scan = ManualScanController(
                area=self.config.area,
                calibration=self.calibration,
                calibration_prompts=self.config.calibration_prompts,
                accept_prompts=self.config.accept_prompts,
            )
            self.plotter = TerrainPlotter(
                self.config.area,
                save_dir=self.config.scan_plots_dir,
                show=False,
                pause_s=0.0,
                height_limit_mm=1.0,
                z_axis_limit_mm=2.0,
                box_z_aspect=25.0,
            )
        self.logger.open()
        return max_cycles

    def _plot_current_block(self) -> None:
        if self.manual_scan is None or self.plotter is None:
            return
        points = self.manual_scan.current_block_points
        if not points:
            return
        target = self.manual_scan.current_target()
        x = np.asarray([point["x"] for point in points], dtype=float)
        y = np.asarray([point["y"] for point in points], dtype=float)
        z = np.asarray([point["z_mm"] for point in points], dtype=float)
        block_number = target["row"] * self.manual_scan.area.columns + target["col"] + 1
        output_path = self.plotter.plot(
            x,
            y,
            z,
            row=target["row"],
            col=target["col"],
            scan_index=block_number,
            total_scans=self.manual_scan.area.rows * self.manual_scan.area.columns,
            title="Block",
            filename=(
                f"block_{block_number:03d}_r{target['row']:02d}_c{target['col']:02d}"
                f"_v{self.calibration.version:02d}.png"
            ),
            reflector_xy=(self.calibration.reflector_x_m, self.calibration.reflector_y_m),
        )
        destination = f" -> {output_path}" if output_path is not None else ""
        print(f"[scan] block ({target['row']}, {target['col']}) plotted{destination}")

    def _run_simulated_grid(self) -> None:
        cfg = self.config
        area = cfg.area
        base_x = cfg.calibration.reflector_x_m if cfg.calibration.reflector_x_m > 0 else None
        sim_config = SimGridConfig(
            x_length_m=area.x_length_m,
            y_length_m=area.y_length_m,
            block_size_m=area.block_size_m,
            points_per_block=1,
            seed=cfg.sim_seed,
            noise_on=cfg.sim_noise_on,
            noise_scale=cfg.sim_noise_scale,
            target_height_m=cfg.reflector_height_m,
            sensor_height_m=cfg.sim_sensor_height_m,
            base_x_m=base_x,
            base_y_m=cfg.calibration.reflector_y_m,
            grid_step_m=cfg.sim_grid_step_m,
            save_dir=None,
            show=False,
            pause_s=0.0,
        )
        simulator = TerrainScanSimulator(sim_config)
        xs = np.arange(0.0, area.x_length_m + 1e-9, cfg.sim_grid_step_m)
        ys = np.arange(0.0, area.y_length_m + 1e-9, cfg.sim_grid_step_m)
        x_mesh, y_mesh = np.meshgrid(xs, ys)
        terrain_mm = simulator.terrain.height_mm(x_mesh, y_mesh)
        measured_mm, error_mm = simulator.measure(x_mesh, y_mesh, noise_seed=cfg.sim_seed)
        timestamp = datetime.now(timezone.utc).isoformat()
        for row_index, y_value in enumerate(ys):
            for col_index, x_value in enumerate(xs):
                point_x = float(x_value)
                point_y = float(y_value)
                measured = float(measured_mm[row_index, col_index])
                self.logger.write(
                    {
                        "timestamp": timestamp,
                        "reflector_height_m": cfg.reflector_height_m,
                        "theta_v_deg": None,
                        "theta_h_deg": None,
                        "platform_x": point_x,
                        "platform_y": point_y,
                        "platform_z": 0.0,
                        "pose_source": "simulated_grid",
                        "z_robot_m": measured / 1000.0,
                        "point_x": point_x,
                        "point_y": point_y,
                        "point_z": measured / 1000.0,
                        "terrain_height_mm": float(terrain_mm[row_index, col_index]),
                        "measured_height_mm": measured,
                        "error_mm": float(error_mm[row_index, col_index]),
                        "block_row": min(
                            area.rows - 1, int(math.floor(point_y / area.block_size_m))
                        ),
                        "block_col": min(
                            area.columns - 1, int(math.floor(point_x / area.block_size_m))
                        ),
                        "block_scan_index": row_index * len(xs) + col_index + 1,
                        "accepted_for_scan": True,
                        "calibration_version": self.calibration.version,
                        "field_x_m": area.x_length_m,
                        "field_y_m": area.y_length_m,
                        "block_size_m": area.block_size_m,
                        "grid_step_m": cfg.sim_grid_step_m,
                        "scan_session": cfg.session_id or "",
                        "reflector_x_m": sim_config.base_x,
                        "reflector_y_m": sim_config.base_y_m,
                        "start_x_m": self.calibration.start_x_m,
                        "start_y_m": self.calibration.start_y_m,
                        "start_yaw_deg": self.calibration.start_yaw_deg,
                        "wheel_circumference_m": cfg.wheel_circumference_m,
                    }
                )
        max_error = float(np.max(np.abs(error_mm)))
        print(
            f"[simulate] grid={len(xs)}x{len(ys)} points={measured_mm.size} "
            f"max|error|={max_error:.4f} mm -> {cfg.output_file}"
        )

    @staticmethod
    def _csv_number(row: dict[str, str], key: str) -> float | None:
        value = row.get(key)
        if value is None or not str(value).strip():
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    def _plot_csv_results(self) -> None:
        try:
            with self.config.output_file.open("r", newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
        except (OSError, csv.Error) as exc:
            print(f"[plot] could not read CSV {self.config.output_file}: {exc}")
            return

        area = self.config.area
        save_dir = self.config.scan_plots_dir or Path("backend/data/scan_plots")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")

        grid_rows: list[tuple[float, float, float, float, float]] = []
        for row in rows:
            x = self._csv_number(row, "point_x")
            y = self._csv_number(row, "point_y")
            measured = self._csv_number(row, "measured_height_mm")
            terrain = self._csv_number(row, "terrain_height_mm")
            error = self._csv_number(row, "error_mm")
            if None not in (x, y, measured, terrain, error):
                grid_rows.append((x, y, measured, terrain, error))

        x_values = sorted({round(point[0], 9) for point in grid_rows})
        y_values = sorted({round(point[1], 9) for point in grid_rows})
        if len(grid_rows) == len(x_values) * len(y_values) and len(x_values) > 1 and len(y_values) > 1:
            x_index = {value: index for index, value in enumerate(x_values)}
            y_index = {value: index for index, value in enumerate(y_values)}
            measured_grid = np.full((len(y_values), len(x_values)), np.nan, dtype=float)
            terrain_grid = np.full_like(measured_grid, np.nan)
            error_grid = np.full_like(measured_grid, np.nan)
            for x, y, measured, terrain, error in grid_rows:
                row_index = y_index[round(y, 9)]
                col_index = x_index[round(x, 9)]
                measured_grid[row_index, col_index] = measured
                terrain_grid[row_index, col_index] = terrain
                error_grid[row_index, col_index] = error
            if np.all(np.isfinite(measured_grid)):
                x_mesh, y_mesh = np.meshgrid(
                    np.asarray(x_values, dtype=float), np.asarray(y_values, dtype=float)
                )
                base_x = (
                    self.calibration.reflector_x_m
                    if self.calibration.reflector_x_m > 0
                    else area.y_length_m / 2.0
                )
                plotter = TerrainPlotter(
                    area,
                    save_dir=save_dir,
                    show=True,
                    pause_s=0.0,
                    block=True,
                    height_limit_mm=1.0,
                    z_axis_limit_mm=2.0,
                    box_z_aspect=25.0,
                )
                output_path = plotter.plot(
                    np.empty(0),
                    np.empty(0),
                    np.empty(0),
                    row=0,
                    col=0,
                    scan_index=1,
                    total_scans=1,
                    title="CSV grid",
                    filename=f"csv_plot_{stamp}.png",
                    true_surface=(x_mesh, y_mesh, terrain_grid),
                    measured_surface=(x_mesh, y_mesh, measured_grid),
                    error_surface=(x_mesh, y_mesh, error_grid),
                    reflector_xy=(base_x, self.calibration.reflector_y_m),
                )
                print(f"[plot] chart from CSV saved -> {output_path}")
                return

        x_values = []
        y_values = []
        z_values = []
        row_value = 0
        col_value = 0
        for row in rows:
            if str(row.get("accepted_for_scan", "")).strip().lower() in {"false", "0"}:
                continue
            x = self._csv_number(row, "point_x")
            y = self._csv_number(row, "point_y")
            z = self._csv_number(row, "point_z")
            if x is None or y is None or z is None:
                continue
            x_values.append(x)
            y_values.append(y)
            z_values.append(z * 1000.0)
            row_value = max(0, min(area.rows - 1, int(math.floor(y / area.block_size_m))))
            col_value = max(0, min(area.columns - 1, int(math.floor(x / area.block_size_m))))

        if not x_values:
            print(f"[plot] no plottable point_x/point_y/point_z rows in {self.config.output_file}")
            return

        plotter = TerrainPlotter(
            area,
            save_dir=save_dir,
            show=True,
            pause_s=0.0,
            block=True,
            auto_limits=True,
        )
        output_path = plotter.plot(
            np.asarray(x_values, dtype=float),
            np.asarray(y_values, dtype=float),
            np.asarray(z_values, dtype=float),
            row=row_value,
            col=col_value,
            scan_index=1,
            total_scans=1,
            title="CSV scan",
            filename=f"csv_plot_{stamp}.png",
            reflector_xy=(self.calibration.reflector_x_m, self.calibration.reflector_y_m),
        )
        print(f"[plot] chart from CSV saved -> {output_path}")

    def _run_continuous(self, max_cycles: int | None) -> None:
        cycles = 0
        while self._running:
            measurement = self.process_measurement(self.collect_measurements())
            if measurement is not None:
                self.logger.write(measurement)
                if self.config.simulate:
                    self.print_summary(cycles + 1, measurement)
            cycles += 1
            if max_cycles is not None and cycles >= max_cycles:
                break
            time.sleep(self.config.sampling_interval_s)
        if self.config.simulate:
            print(f"[simulate] finished: {cycles} samples -> {self.config.output_file} (all fields written to CSV)")

    def _run_manual_scan(self, max_cycles: int | None) -> None:
        if self.manual_scan is None:
            return
        controller = self.manual_scan
        cycles = 0
        applied_version = self.calibration.version
        applied_area = controller.area
        calibration_applied = (
            self.calibration.wheel_circumference_m > 0
            and self.calibration.reflector_z_m > 0
        )
        while self._running and not controller.completed:
            controller.prompt_before_block()
            area_changed = (
                controller.area.x_length_m != applied_area.x_length_m
                or controller.area.y_length_m != applied_area.y_length_m
                or controller.area.block_size_m != applied_area.block_size_m
                or not np.array_equal(controller.area.point_counts, applied_area.point_counts)
            )
            self.config.area = controller.area
            if self.plotter is not None:
                self.plotter.area = controller.area
            session_changed = area_changed
            if not calibration_applied or controller.calibration.version != applied_version:
                self.apply_calibration(controller.calibration)
                calibration_applied = True
                applied_version = controller.calibration.version
                session_changed = True
            if session_changed:
                applied_area = controller.area
                self._ensure_session()
            target = controller.current_target()
            print(
                f"[scan] block ({target['row']}, {target['col']}) "
                f"needs {target['count']} reference-tracked scans"
            )
            while self._running and not controller.block_complete():
                action = controller.prompt_action()
                if action == "q":
                    self._running = False
                    break
                if action == "s":
                    controller.skip_block()
                    print(f"[scan] block ({target['row']}, {target['col']}) skipped")
                    break

                target = controller.current_target()
                raw = self.collect_measurements()
                measurement = self.process_measurement(raw)
                cycles += 1
                if measurement is None:
                    raw.update(
                        controller.metadata(self.calibration, target["scan_index"], False)
                    )
                    self.logger.write(raw)
                    print(f"[scan] sample {target['scan_index']} rejected by processing")
                else:
                    accepted = controller.accept(measurement)
                    measurement.update(
                        controller.metadata(self.calibration, target["scan_index"], accepted)
                    )
                    self.logger.write(measurement)
                    if accepted:
                        if self.config.session_id:
                            session_store.record_measurement(self.config.session_id, measurement)
                        if self.config.simulate:
                            self.print_summary(cycles, measurement)
                    else:
                        reason = controller.can_accept(measurement)
                        print(f"[scan] sample {target['scan_index']} not accepted: {reason}")

                if max_cycles is not None and cycles >= max_cycles:
                    self._running = False
                    break
                time.sleep(self.config.sampling_interval_s)

            if not self._running:
                break
            self._plot_current_block()
            if not controller.advance_block():
                break
        if self.config.simulate:
            print(
                f"[simulate] manual scan finished: {len(controller.all_points)} accepted points "
                f"-> {self.config.output_file}"
            )

    def run(self, max_cycles: int | None = None) -> None:
        max_cycles = self._prepare_run(max_cycles)
        try:
            if not self.config.simulate:
                self.verify_devices()
            if self.config.manual_scan:
                self._run_manual_scan(max_cycles)
            elif self.config.simulate and self.config.show_plots:
                self._run_simulated_grid()
            else:
                self._run_continuous(max_cycles)
        finally:
            self.close()
            if self.config.show_plots and self.config.simulate and not self.config.manual_scan:
                self._plot_csv_results()


def main() -> None:
    parser = argparse.ArgumentParser(description="Surface Level Mapping data acquisition.")
    parser.add_argument("--simulate", action="store_true", help="use simulated sensors instead of Raspberry Pi hardware")
    parser.add_argument("--show", action="store_true", help="with --simulate, write a grid CSV then plot its surfaces and save the PNG")
    parser.add_argument("--cycles", type=int, default=None, help="stop after this many samples (continuous simulation defaults to 20)")
    parser.add_argument("--output", type=Path, default=None, help="CSV output path")
    parser.add_argument("--reflector-height", type=float, default=None, help="known reflector height H_ref in metres (simulation defaults to 1.5)")
    parser.add_argument("--manual-scan", action="store_true", help="run manual reference-tracked block scanning")
    parser.add_argument("--session-id", default=None, help="frontend scan session id to update while scanning")
    parser.add_argument("--x-length", type=float, default=None, help="field X length in metres (grid --show defaults to 70)")
    parser.add_argument("--y-length", type=float, default=None, help="field Y length in metres (grid --show defaults to 100)")
    parser.add_argument("--block-size", type=float, default=None, help="block side length in metres (grid --show defaults to 5)")
    parser.add_argument("--grid-step", type=float, default=5.0, help="simulated grid spacing in metres for --show")
    parser.add_argument("--sensor-height", type=float, default=1.2, help="simulated sensor height H_true in metres")
    parser.add_argument("--noise-on", action="store_true", help="enable simulated sensor noise")
    parser.add_argument("--noise-scale", type=float, default=1.0, help="simulated sensor noise scale")
    parser.add_argument("--seed", type=int, default=14, help="terrain random seed")
    parser.add_argument("--points-per-block", type=int, default=4, help="scans per block")
    parser.add_argument("--points-grid", type=str, default=None, help="semicolon-separated per-block scan counts")
    parser.add_argument("--scan-plots-dir", type=Path, default=Path("backend/data/scan_plots"))
    parser.add_argument("--no-prompts", action="store_true", help="disable calibration and accept prompts")
    parser.add_argument("--reflector-x", type=float, default=0.0, help="reflector X in field coordinates")
    parser.add_argument("--reflector-y", type=float, default=0.0, help="reflector Y in field coordinates")
    parser.add_argument("--start-x", type=float, default=0.0, help="initial machine X in field coordinates")
    parser.add_argument("--start-y", type=float, default=0.0, help="initial machine Y in field coordinates")
    parser.add_argument("--start-yaw", type=float, default=0.0, help="initial machine yaw in degrees")
    parser.add_argument("--wheel-circumference", type=float, default=None, help="calibrated wheel circumference in metres (simulation defaults to 1.0)")
    parser.add_argument("--tilt-mount-offset", type=float, default=0.0, help="fixed LiDAR/tilt mount offset in degrees")
    args = parser.parse_args()

    try:
        reflector_height = 1.5 if args.reflector_height is None and args.simulate else args.reflector_height or 0.0
        wheel_circumference = 1.0 if args.wheel_circumference is None and args.simulate else args.wheel_circumference or 0.0
        grid_show = args.simulate and args.show and not args.manual_scan
        x_length = 70.0 if grid_show and args.x_length is None else (120.0 if args.x_length is None else args.x_length)
        y_length = 100.0 if grid_show and args.y_length is None else (56.0 if args.y_length is None else args.y_length)
        block_size = 5.0 if grid_show and args.block_size is None else (8.0 if args.block_size is None else args.block_size)
        points_per_block: int | np.ndarray = args.points_per_block
        if args.points_grid is not None:
            rows = []
            for line in args.points_grid.split(";"):
                rows.append([int(value) for value in line.split()])
            points_per_block = np.asarray(rows, dtype=int)

        area = ScanAreaConfig(
            x_length_m=x_length,
            y_length_m=y_length,
            block_size_m=block_size,
            points_per_block=points_per_block,
        )
        calibration = CalibrationConfig(
            reflector_x_m=args.reflector_x,
            reflector_y_m=args.reflector_y,
            reflector_z_m=reflector_height,
            start_x_m=args.start_x,
            start_y_m=args.start_y,
            start_yaw_deg=args.start_yaw,
            wheel_circumference_m=wheel_circumference,
            tilt_mount_offset_deg=args.tilt_mount_offset,
            version=1,
        )
        config = MappingConfig(
            simulate=args.simulate,
            reflector_height_m=reflector_height,
            wheel_circumference_m=wheel_circumference,
            tilt_mount_offset_deg=args.tilt_mount_offset,
            manual_scan=args.manual_scan,
            area=area,
            calibration=calibration,
            scan_plots_dir=args.scan_plots_dir,
            show_plots=args.show,
            sim_grid_step_m=args.grid_step,
            sim_sensor_height_m=args.sensor_height,
            sim_noise_on=args.noise_on,
            sim_noise_scale=args.noise_scale,
            sim_seed=args.seed,
            calibration_prompts=not args.no_prompts,
            accept_prompts=not args.no_prompts,
            session_id=args.session_id,
        )
        if args.output is not None:
            config.output_file = args.output
        system = SurfaceMappingSystem(config)
        system.run(max_cycles=args.cycles)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
