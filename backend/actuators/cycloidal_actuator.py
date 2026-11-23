"""Cycloidal actuator abstraction.

PSEUDOCODE:
    CLASS CycloidalActuator(name, driver, encoder):
        INIT:
            STORE identity, driver, encoder, angle limits

        read_angle_deg():
            RETURN encoder angle (or driver feedback)

        move_to_angle(target_deg):
            CLAMP target to limits
            SEND target to motor driver
            RETURN commanded target

        stop():
            SEND stop/hold command
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional


@dataclass
class ActuatorCommand:
    name: str
    target_angle_deg: float
    actual_angle_deg: Optional[float]
    clamped: bool = False


class CycloidalActuator:
    """Single-axis cycloidal actuator with encoder feedback and driver command."""

    def __init__(self, name: str, encoder: Any = None, driver: Any = None, min_angle_deg: float = -180.0, max_angle_deg: float = 180.0):
        # PSEUDOCODE: bind feedback/command interfaces for one mechanical axis.
        self.name = name
        self.encoder = encoder
        self.driver = driver
        self.min_angle_deg = min_angle_deg
        self.max_angle_deg = max_angle_deg
        self.target_angle_deg = 0.0

    def read_angle_deg(self) -> Optional[float]:
        # PSEUDOCODE: prefer encoder feedback; fall back to driver feedback; else unknown.
        if self.encoder is not None and hasattr(self.encoder, "read_degrees"):
            try:
                return float(self.encoder.read_degrees())
            except Exception:
                pass
        if self.driver is not None and hasattr(self.driver, "read_angle_deg"):
            try:
                return float(self.driver.read_angle_deg())
            except Exception:
                pass
        return None

    def move_to_angle(self, target_deg: float) -> ActuatorCommand:
        # PSEUDOCODE: clamp to safe mechanical range, command driver, record target.
        clamped_target = max(self.min_angle_deg, min(self.max_angle_deg, float(target_deg)))
        clamped = clamped_target != float(target_deg)
        self.target_angle_deg = clamped_target
        if self.driver is not None and hasattr(self.driver, "move_to_angle"):
            try:
                self.driver.move_to_angle(clamped_target)
            except Exception:
                # PSEUDOCODE: keep software state; hardware fault should be surfaced by caller/logging.
                pass
        actual = self.read_angle_deg()
        # PSEUDOCODE: if no physical feedback/driver, expose commanded target for simulation/logging continuity.
        if actual is None:
            actual = clamped_target
        return ActuatorCommand(
            name=self.name,
            target_angle_deg=clamped_target,
            actual_angle_deg=actual,
            clamped=clamped,
        )

    def stop(self) -> None:
        # PSEUDOCODE: request driver hold/stop if supported.
        if self.driver is not None and hasattr(self.driver, "stop"):
            try:
                self.driver.stop()
            except Exception:
                pass
