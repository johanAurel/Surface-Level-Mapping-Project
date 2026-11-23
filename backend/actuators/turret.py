"""2-axis pan/tilt turret driven by cycloidal actuators.

PSEUDOCODE:
    CLASS PanTiltTurret(pan_actuator, tilt_actuator):
        read():
            RETURN pan/tilt actual angles and targets

        move_to(pan_deg, tilt_deg):
            COMMAND pan actuator
            COMMAND tilt actuator
            RETURN command/result dict

        track_reference(camera_error_x_px, camera_error_y_px):
            CONVERT pixel error to angle delta (simple proportional gain)
            APPLY deadband
            COMMAND new pan/tilt targets
"""

from __future__ import annotations

from typing import Any

from backend.actuators.cycloidal_actuator import CycloidalActuator


class PanTiltTurret:
    """Pan/tilt head integrating two cycloidal actuators."""

    def __init__(self, pan_actuator: CycloidalActuator, tilt_actuator: CycloidalActuator, pan_gain_deg_per_px: float = 0.01, tilt_gain_deg_per_px: float = 0.01, deadband_px: float = 20.0):
        # PSEUDOCODE: store mechanical axes and tracking control parameters.
        self.pan_actuator = pan_actuator
        self.tilt_actuator = tilt_actuator
        self.pan_gain_deg_per_px = pan_gain_deg_per_px
        self.tilt_gain_deg_per_px = tilt_gain_deg_per_px
        self.deadband_px = deadband_px

    def read(self) -> dict[str, Any]:
        # PSEUDOCODE: report current axis angles/targets; omit unknown actuals to avoid clobbering encoder data.
        pan = self.pan_actuator.read_angle_deg()
        tilt = self.tilt_actuator.read_angle_deg()
        data: dict[str, Any] = {
            "pan_target_angle": self.pan_actuator.target_angle_deg,
            "tilt_target_angle": self.tilt_actuator.target_angle_deg,
        }
        if pan is not None:
            data["pan_angle"] = pan
        if tilt is not None:
            data["tilt_angle"] = tilt
        return data

    def move_to(self, pan_deg: float, tilt_deg: float) -> dict[str, Any]:
        # PSEUDOCODE: command both axes.
        pan_cmd = self.pan_actuator.move_to_angle(pan_deg)
        tilt_cmd = self.tilt_actuator.move_to_angle(tilt_deg)
        return {
            "pan_command_target_deg": pan_cmd.target_angle_deg,
            "tilt_command_target_deg": tilt_cmd.target_angle_deg,
            "pan_command_clamped": pan_cmd.clamped,
            "tilt_command_clamped": tilt_cmd.clamped,
        }

    def track_reference(self, x_error_px: float | None, y_error_px: float | None) -> dict[str, Any]:
        # PSEUDOCODE: if reference visible, move turret to reduce camera pixel error.
        if x_error_px is None or y_error_px is None:
            return {"tracking_active": False, "reason": "reference_not_detected"}

        pan_read = self.pan_actuator.read_angle_deg()
        tilt_read = self.tilt_actuator.read_angle_deg()
        pan_current = self.pan_actuator.target_angle_deg if pan_read is None else pan_read
        tilt_current = self.tilt_actuator.target_angle_deg if tilt_read is None else tilt_read

        pan_delta = 0.0
        tilt_delta = 0.0
        if abs(x_error_px) > self.deadband_px:
            pan_delta = x_error_px * self.pan_gain_deg_per_px
        if abs(y_error_px) > self.deadband_px:
            tilt_delta = -y_error_px * self.tilt_gain_deg_per_px  # image y+ down -> tilt up negative/adjust as calibrated

        cmd = self.move_to(pan_current + pan_delta, tilt_current + tilt_delta)
        cmd.update({
            "tracking_active": True,
            "pan_delta_deg": pan_delta,
            "tilt_delta_deg": tilt_delta,
            "x_error_px": x_error_px,
            "y_error_px": y_error_px,
        })
        return cmd

    def stop(self) -> None:
        # PSEUDOCODE: stop both actuators.
        self.pan_actuator.stop()
        self.tilt_actuator.stop()
