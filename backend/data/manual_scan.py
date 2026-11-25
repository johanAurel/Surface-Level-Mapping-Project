"""Manual reference-tracked scan controller."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from backend.data.scan_area import CalibrationConfig, ScanAreaConfig


@dataclass
class ManualScanController:
    area: ScanAreaConfig
    calibration: CalibrationConfig
    calibration_prompts: bool = True
    accept_prompts: bool = True
    row: int = field(init=False, default=0)
    col: int = field(init=False, default=0)
    scan_index: int = field(init=False, default=0)
    completed: bool = field(init=False, default=False)
    session: int = field(init=False, default=1)
    current_block_points: list[dict[str, float]] = field(init=False, default_factory=list)
    all_points: list[dict[str, float]] = field(init=False, default_factory=list)

    def current_target(self) -> dict[str, int]:
        count = int(self.area.point_counts[self.row, self.col])
        return {
            "row": self.row,
            "col": self.col,
            "count": count,
            "scan_index": self.scan_index + 1,
            "remaining": max(0, count - self.scan_index),
        }

    def block_complete(self) -> bool:
        count = int(self.area.point_counts[self.row, self.col])
        return self.scan_index >= count

    def can_accept(self, measurement: dict[str, Any]) -> str | None:
        if not measurement.get("reference_detected"):
            return "reference not detected"
        if not measurement.get("lidar_valid"):
            return "lidar invalid"
        if measurement.get("z_robot_m") is None:
            return "z_robot_m unavailable"
        if measurement.get("platform_x") is None or measurement.get("platform_y") is None:
            return "odometry unavailable"
        return None

    def accept(self, measurement: dict[str, Any]) -> bool:
        if self.can_accept(measurement) is not None:
            return False
        point = {
            "x": float(measurement["point_x"]),
            "y": float(measurement["point_y"]),
            "z_mm": float(measurement["z_robot_m"]) * 1000.0,
        }
        self.current_block_points.append(point)
        self.all_points.append(point)
        self.scan_index += 1
        return True

    def skip_block(self) -> None:
        self.scan_index = int(self.area.point_counts[self.row, self.col])
        self.current_block_points = []

    def advance_block(self) -> bool:
        if not self.block_complete():
            return False
        self.col += 1
        if self.col >= self.area.columns:
            self.col = 0
            self.row += 1
        if self.row >= self.area.rows:
            self.completed = True
            return False
        self.scan_index = 0
        self.current_block_points = []
        return True

    def update_area(self, area: ScanAreaConfig) -> None:
        self.area = area
        self.row = 0
        self.col = 0
        self.scan_index = 0
        self.completed = False
        self.session += 1
        self.current_block_points = []
        self.all_points = []

    def prompt_before_block(self) -> None:
        if not self.calibration_prompts:
            return
        area_answer = input("Change scan area before this block? [y/N]: ").strip().lower()
        if area_answer in {"y", "yes"}:
            try:
                updated_area = self.area.prompt_update()
                changed = (
                    updated_area.x_length_m != self.area.x_length_m
                    or updated_area.y_length_m != self.area.y_length_m
                    or updated_area.block_size_m != self.area.block_size_m
                    or not np.array_equal(updated_area.point_counts, self.area.point_counts)
                )
                if changed:
                    self.update_area(updated_area)
                    print(f"scan area restarted as session {self.session}")
                else:
                    self.area = updated_area
            except (ValueError, EOFError) as exc:
                print(f"error: {exc}; keeping previous scan area")

        calibration_answer = input("Change calibration before this block? [y/N]: ").strip().lower()
        if calibration_answer in {"y", "yes"}:
            try:
                self.calibration = self.calibration.prompt_update()
            except (ValueError, EOFError) as exc:
                print(f"error: {exc}; keeping previous calibration")

    def prompt_action(self) -> str:
        target = self.current_target()
        if not self.accept_prompts:
            return "measure"
        prompt = (
            f"Block ({target['row']}, {target['col']}) scan "
            f"{target['scan_index']}/{target['count']}: "
            "Enter=measure, s=skip block, q=quit: "
        )
        return input(prompt).strip().lower()

    def metadata(
        self, calibration: CalibrationConfig, scan_index: int, accepted: bool
    ) -> dict[str, float | int | bool | str]:
        data: dict[str, float | int | bool | str] = {
            "block_row": self.row,
            "block_col": self.col,
            "block_scan_index": scan_index,
            "accepted_for_scan": accepted,
            "field_x_m": self.area.x_length_m,
            "field_y_m": self.area.y_length_m,
            "block_size_m": self.area.block_size_m,
            "scan_session": self.session,
            "pose_source": "odometry",
        }
        data.update(calibration.metadata())
        return data
