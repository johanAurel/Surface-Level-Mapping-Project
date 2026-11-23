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

This derives terrain-ready variables; it does not render terrain.
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

from backend.data.coordinates import calculate_coordinates, calculate_robot_elevation
from backend.data.processing import combine_readings


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
    """Simple odometry placeholder.

    Without heading/wheel calibration it cannot produce absolute X/Y; it exposes
    travel distance and placeholders so the dataset schema remains terrain-ready.
    """

    wheel_circumference_m: float = 0.0
    _start_degrees: float | None = None

    def read(self, sensor_encoder_degrees: float | None, imu: dict[str, Any]) -> dict[str, Any]:
        degrees = 0.0 if sensor_encoder_degrees is None else float(sensor_encoder_degrees)
        if self._start_degrees is None:
            self._start_degrees = degrees
        delta_degrees = degrees - self._start_degrees
        revolutions = delta_degrees / 360.0
        travel_m = revolutions * self.wheel_circumference_m if self.wheel_circumference_m > 0 else None

        yaw = imu.get("yaw")
        yaw_rad = math.radians(float(yaw)) if yaw is not None else None
        platform_x = travel_m * math.cos(yaw_rad) if travel_m is not None and yaw_rad is not None else None
        platform_y = travel_m * math.sin(yaw_rad) if travel_m is not None and yaw_rad is not None else None

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

    def run(self, max_cycles: int | None = None) -> None:
        # PSEUDOCODE: main acquisition loop with deterministic logging and safe shutdown.
        if not self.config.simulate:
            self.require_raspberry_pi()
            self.validate_pin_config()
            self.verify_gpio_pins()
        if max_cycles is not None and max_cycles <= 0:
            max_cycles = None
        elif self.config.simulate and max_cycles is None:
            max_cycles = 20
        self.initialise_hardware()
        self._running = True
        cycles = 0
        try:
            if not self.config.simulate:
                self.verify_devices()
            self.logger.open()
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
        finally:
            self.close()
        if self.config.simulate:
            print(f"[simulate] finished: {cycles} samples -> {self.config.output_file} (all fields written to CSV)")


def main() -> None:
    parser = argparse.ArgumentParser(description="Surface Level Mapping data acquisition.")
    parser.add_argument("--simulate", action="store_true", help="use simulated sensors instead of Raspberry Pi hardware")
    parser.add_argument("--cycles", type=int, default=None, help="stop after this many samples (simulation defaults to 20)")
    parser.add_argument("--output", type=Path, default=None, help="CSV output path")
    parser.add_argument("--reflector-height", type=float, default=None, help="known reflector height H_ref in metres (simulation defaults to 1.5)")
    args = parser.parse_args()

    if args.reflector_height is None:
        reflector_height = 1.5 if args.simulate else 0.0
    else:
        reflector_height = args.reflector_height
    config = MappingConfig(simulate=args.simulate, reflector_height_m=reflector_height)
    if args.output is not None:
        config.output_file = args.output
    system = SurfaceMappingSystem(config)
    try:
        system.run(max_cycles=args.cycles)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
