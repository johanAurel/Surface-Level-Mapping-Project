"""Shared scan area and calibration configuration."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Sequence

import numpy as np


def _format_counts(value: int | Sequence[Sequence[int]] | np.ndarray) -> str:
    array = np.asarray(value)
    if array.ndim == 0:
        return str(int(array))
    return "; ".join(" ".join(str(int(item)) for item in row) for row in array)


def _parse_counts(raw: str) -> int | np.ndarray:
    if ";" in raw or " " in raw:
        rows = []
        for line in raw.split(";"):
            rows.append([int(item) for item in line.split()])
        return np.asarray(rows, dtype=int)
    return int(raw)


def _prompt_float(label: str, current: float) -> float:
    while True:
        raw = input(f"  {label} [{current}]: ").strip()
        if not raw:
            return float(current)
        try:
            return float(raw)
        except ValueError:
            print("  invalid number; try again")


@dataclass
class ScanAreaConfig:
    x_length_m: float = 120.0
    y_length_m: float = 56.0
    block_size_m: float = 8.0
    points_per_block: int | Sequence[Sequence[int]] | np.ndarray = 4
    rows: int = field(init=False)
    columns: int = field(init=False)
    point_counts: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        if self.x_length_m <= 0 or self.y_length_m <= 0:
            raise ValueError("field lengths must be positive")
        if self.block_size_m <= 0:
            raise ValueError("block_size_m must be positive")

        self.columns = int(round(self.x_length_m / self.block_size_m))
        self.rows = int(round(self.y_length_m / self.block_size_m))
        if self.columns < 1 or self.rows < 1:
            raise ValueError("field must contain at least one block")
        if not math.isclose(self.columns * self.block_size_m, self.x_length_m, abs_tol=1e-9):
            raise ValueError("block_size_m must divide x_length_m exactly")
        if not math.isclose(self.rows * self.block_size_m, self.y_length_m, abs_tol=1e-9):
            raise ValueError("block_size_m must divide y_length_m exactly")

        counts = np.asarray(self.points_per_block, dtype=int)
        if counts.ndim == 0:
            value = int(counts)
            if value < 1:
                raise ValueError("points_per_block must be at least 1")
            self.point_counts = np.full((self.rows, self.columns), value, dtype=int)
        elif counts.shape == (self.rows, self.columns):
            if np.any(counts < 0):
                raise ValueError("block point counts cannot be negative")
            self.point_counts = counts
        else:
            raise ValueError(
                f"points_per_block must be a scalar or a {self.rows}x{self.columns} array"
            )

    @property
    def square_size_m(self) -> float:
        return self.block_size_m

    @property
    def points_per_square(self) -> int | np.ndarray:
        return self.points_per_block

    def prompt_update(self) -> ScanAreaConfig:
        print("Scan area calibration (Enter keeps current value):")
        values = {
            "x_length_m": _prompt_float("x_length_m", self.x_length_m),
            "y_length_m": _prompt_float("y_length_m", self.y_length_m),
            "block_size_m": _prompt_float("block_size_m", self.block_size_m),
        }
        raw = input(f"  points_per_block [{_format_counts(self.points_per_block)}]: ").strip()
        if raw:
            values["points_per_block"] = _parse_counts(raw)
        updated = replace(self, **values)
        return updated


@dataclass
class CalibrationConfig:
    reflector_x_m: float = 0.0
    reflector_y_m: float = 0.0
    reflector_z_m: float = 1.5
    start_x_m: float = 0.0
    start_y_m: float = 0.0
    start_yaw_deg: float = 0.0
    wheel_circumference_m: float = 0.0
    tilt_mount_offset_deg: float = 0.0
    version: int = 0

    def validate(self, require_reflector: bool = True) -> None:
        if self.wheel_circumference_m <= 0:
            raise ValueError("wheel_circumference_m must be positive")
        if require_reflector and self.reflector_z_m <= 0:
            raise ValueError("reflector_z_m must be positive")

    def prompt_update(self) -> CalibrationConfig:
        print("Block calibration (Enter keeps current value):")
        values = {
            "reflector_x_m": _prompt_float("reflector_x_m", self.reflector_x_m),
            "reflector_y_m": _prompt_float("reflector_y_m", self.reflector_y_m),
            "reflector_z_m": _prompt_float("reflector_z_m", self.reflector_z_m),
            "start_x_m": _prompt_float("start_x_m", self.start_x_m),
            "start_y_m": _prompt_float("start_y_m", self.start_y_m),
            "start_yaw_deg": _prompt_float("start_yaw_deg", self.start_yaw_deg),
            "wheel_circumference_m": _prompt_float(
                "wheel_circumference_m", self.wheel_circumference_m
            ),
            "tilt_mount_offset_deg": _prompt_float(
                "tilt_mount_offset_deg", self.tilt_mount_offset_deg
            ),
        }
        updated = replace(self, **values, version=self.version + 1)
        updated.validate()
        return updated

    def metadata(self) -> dict[str, float | int]:
        return {
            "calibration_version": self.version,
            "reflector_x_m": self.reflector_x_m,
            "reflector_y_m": self.reflector_y_m,
            "reflector_height_m": self.reflector_z_m,
            "start_x_m": self.start_x_m,
            "start_y_m": self.start_y_m,
            "start_yaw_deg": self.start_yaw_deg,
            "wheel_circumference_m": self.wheel_circumference_m,
            "tilt_mount_offset_deg": self.tilt_mount_offset_deg,
        }
