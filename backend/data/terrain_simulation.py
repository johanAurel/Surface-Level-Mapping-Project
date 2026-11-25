"""Incremental terrain scan simulation and plotting.

A full scan means one field square has been completed. The simulator keeps all
measurements collected so far and renders the terrain after every square.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np

from backend.data.scan_area import ScanAreaConfig
from backend.data.terrain_plots import TerrainPlotter


@dataclass(frozen=True)
class Terrain:
    wavelength: np.ndarray
    angle: np.ndarray
    phase: np.ndarray
    amplitude: np.ndarray

    def height_mm(self, x: np.ndarray | float, y: np.ndarray | float) -> np.ndarray:
        x_grid, y_grid = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
        height = np.zeros_like(x_grid, dtype=float)
        for wavelength, angle, phase, amplitude in zip(
            self.wavelength, self.angle, self.phase, self.amplitude
        ):
            projection = x_grid * math.cos(angle) + y_grid * math.sin(angle)
            height += amplitude * np.sin(2 * math.pi * projection / wavelength + phase)
        return height


@dataclass
class ScanConfig(ScanAreaConfig):
    seed: int | None = 14
    noise_on: bool = False
    noise_scale: float = 1.0
    target_height_m: float = 1.5
    sensor_height_m: float = 1.2
    base_x_m: float | None = None
    base_y_m: float = 0.0
    grid_step_m: float = 1.0
    save_dir: Path | None = Path("backend/data/terrain_scans")
    show: bool = False
    pause_s: float = 0.5
    base_x: float = field(init=False)

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.grid_step_m <= 0:
            raise ValueError("grid_step_m must be positive")
        self.base_x = self.y_length_m / 2 if self.base_x_m is None else float(self.base_x_m)
        if self.save_dir is not None:
            self.save_dir = Path(self.save_dir)


@dataclass
class TerrainScanSimulator:
    config: ScanConfig = field(default_factory=ScanConfig)
    terrain: Terrain = field(init=False)
    x: np.ndarray = field(init=False)
    y: np.ndarray = field(init=False)
    z: np.ndarray = field(init=False)
    error: np.ndarray = field(init=False)
    scan_index: int = field(init=False, default=0)
    scans: list[dict[str, int]] = field(init=False, default_factory=list)
    plotter: TerrainPlotter = field(init=False)

    def __post_init__(self) -> None:
        rng = np.random.default_rng(self.config.seed)
        layer_count = 4
        self.terrain = Terrain(
            wavelength=rng.uniform(20, 80, layer_count),
            angle=rng.uniform(0, np.pi, layer_count),
            phase=rng.uniform(0, 2 * np.pi, layer_count),
            amplitude=rng.uniform(0.2, 0.6, layer_count),
        )
        self.x = np.empty(0, dtype=float)
        self.y = np.empty(0, dtype=float)
        self.z = np.empty(0, dtype=float)
        self.error = np.empty(0, dtype=float)
        self.plotter = TerrainPlotter(
            self.config,
            save_dir=self.config.save_dir,
            show=self.config.show,
            pause_s=self.config.pause_s,
            grid_step_m=self.config.grid_step_m,
            height_limit_mm=1.0,
            z_axis_limit_mm=2.0,
            box_z_aspect=25.0,
        )

    def _noise_seed(self, offset: int, row: int, col: int, point: int) -> int | None:
        if self.config.seed is None:
            return None
        return self.config.seed + offset + row * 100_000 + col * 10_000 + point

    def _positions_in_square(self, row: int, col: int, count: int) -> tuple[np.ndarray, np.ndarray]:
        if count == 0:
            empty = np.empty(0, dtype=float)
            return empty, empty.copy()

        side = int(math.ceil(math.sqrt(count)))
        cell_size = self.config.square_size_m / side
        cells = [(i, j) for j in range(side) for i in range(side)]
        rng = np.random.default_rng(self._noise_seed(10_000, row, col, 0))
        rng.shuffle(cells)

        x0 = col * self.config.square_size_m
        y0 = row * self.config.square_size_m
        xs: list[float] = []
        ys: list[float] = []
        for i, j in cells[:count]:
            xs.append(x0 + (i + rng.uniform(0.2, 0.8)) * cell_size)
            ys.append(y0 + (j + rng.uniform(0.2, 0.8)) * cell_size)
        return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)

    def measure(
        self, x: np.ndarray | float, y: np.ndarray | float, noise_seed: int | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        x_grid, y_grid = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
        terrain_mm = self.terrain.height_mm(x_grid, y_grid)
        ground_m = terrain_mm / 1000.0
        pivot_m = ground_m + self.config.sensor_height_m

        horizontal_distance = np.hypot(x_grid - self.config.base_x, y_grid - self.config.base_y_m)
        vertical_distance = self.config.target_height_m - pivot_m
        theta_true = np.arctan2(vertical_distance, horizontal_distance)
        slant_true = np.hypot(horizontal_distance, vertical_distance)

        theta_meas = theta_true
        slant_meas = slant_true
        height_meas = self.config.sensor_height_m
        if self.config.noise_on and self.config.noise_scale > 0:
            rng = np.random.default_rng(noise_seed)
            theta_meas = theta_true + self.config.noise_scale * rng.normal(
                0, np.deg2rad(0.0005), x_grid.shape
            )
            slant_meas = slant_true + self.config.noise_scale * rng.normal(
                0, 0.02, x_grid.shape
            )
            height_meas = self.config.sensor_height_m + self.config.noise_scale * rng.normal(
                0, 0.0003, x_grid.shape
            )

        pivot_est = self.config.target_height_m - slant_meas * np.sin(theta_meas)
        measured_mm = (pivot_est - height_meas) * 1000.0
        error_mm = measured_mm - terrain_mm
        return measured_mm, error_mm

    def _plot(self, row: int, col: int) -> Path | None:
        x_points = max(
            2, int(round(self.config.x_length_m / self.config.grid_step_m)) + 1
        )
        y_points = max(
            2, int(round(self.config.y_length_m / self.config.grid_step_m)) + 1
        )
        x_values = np.linspace(0, self.config.x_length_m, x_points)
        y_values = np.linspace(0, self.config.y_length_m, y_points)
        x_mesh, y_mesh = np.meshgrid(x_values, y_values)
        true_surface = (x_mesh, y_mesh, self.terrain.height_mm(x_mesh, y_mesh))
        return self.plotter.plot(
            self.x,
            self.y,
            self.z,
            row=row,
            col=col,
            scan_index=self.scan_index,
            total_scans=self.config.rows * self.config.columns,
            title="Scan",
            filename=f"scan_{self.scan_index:02d}_r{row:02d}_c{col:02d}.png",
            error=self.error,
            true_surface=true_surface,
            reflector_xy=(self.config.base_x, self.config.base_y_m),
        )

    def scan_square(self, row: int, col: int) -> dict[str, int]:
        if not (0 <= row < self.config.rows and 0 <= col < self.config.columns):
            raise ValueError(f"invalid square index: row={row}, col={col}")

        count = int(self.config.point_counts[row, col])
        xs, ys = self._positions_in_square(row, col, count)
        zs: list[float] = []
        errors: list[float] = []
        for point, (px, py) in enumerate(zip(xs, ys)):
            measured_mm, error_mm = self.measure(
                px, py, self._noise_seed(1_000_000, row, col, point)
            )
            zs.append(float(measured_mm))
            errors.append(float(error_mm))

        self.x = np.concatenate((self.x, xs))
        self.y = np.concatenate((self.y, ys))
        self.z = np.concatenate((self.z, np.asarray(zs, dtype=float)))
        self.error = np.concatenate((self.error, np.asarray(errors, dtype=float)))
        self.scan_index += 1

        scan = {"index": self.scan_index, "row": row, "col": col, "points": count}
        self.scans.append(scan)
        output_path = self._plot(row, col)
        max_error = float(np.abs(self.error).max()) if self.error.size else 0.0
        destination = f" -> {output_path}" if output_path is not None else ""
        print(
            f"[scan {self.scan_index:02d}/{self.config.rows * self.config.columns}] "
            f"square=({row}, {col}) points={count} collected={self.x.size} "
            f"max|error|={max_error:.4f} mm{destination}"
        )
        return scan

    def run(self) -> dict[str, np.ndarray]:
        for row in range(self.config.rows):
            for col in range(self.config.columns):
                self.scan_square(row, col)
        return {"x": self.x, "y": self.y, "z": self.z, "error": self.error}


def main() -> None:
    parser = argparse.ArgumentParser(description="Incremental terrain scan simulator.")
    parser.add_argument("--x-length", type=float, default=120.0, help="field X length in metres")
    parser.add_argument("--y-length", type=float, default=56.0, help="field Y length in metres")
    parser.add_argument("--square-size", type=float, default=8.0, help="square side length in metres")
    parser.add_argument("--points-per-square", type=int, default=4, help="measurement points per square")
    parser.add_argument(
        "--points-grid",
        type=str,
        default=None,
        help="semicolon-separated rows of per-square counts, e.g. '4 4;4 4' for a 2x2 square grid",
    )
    parser.add_argument("--seed", type=int, default=14, help="random seed for repeatable terrain and scans")
    parser.add_argument("--noise-on", action="store_true", help="enable sensor noise")
    parser.add_argument("--noise-scale", type=float, default=1.0, help="sensor noise scale")
    parser.add_argument("--target-height", type=float, default=1.5, help="reflector height in metres")
    parser.add_argument("--sensor-height", type=float, default=1.2, help="nominal sensor height in metres")
    parser.add_argument("--grid-step", type=float, default=1.0, help="rendering grid resolution in metres")
    parser.add_argument("--save-dir", type=Path, default=Path("backend/data/terrain_scans"))
    parser.add_argument("--show", action="store_true", help="show each plot while running")
    parser.add_argument("--pause", type=float, default=0.5, help="seconds to keep each live plot open")
    args = parser.parse_args()

    try:
        points_per_square: int | np.ndarray = args.points_per_square
        if args.points_grid is not None:
            rows = []
            for line in args.points_grid.split(";"):
                rows.append([int(value) for value in line.split()])
            points_per_square = np.asarray(rows, dtype=int)

        config = ScanConfig(
            x_length_m=args.x_length,
            y_length_m=args.y_length,
            block_size_m=args.square_size,
            points_per_block=points_per_square,
            seed=args.seed,
            noise_on=args.noise_on,
            noise_scale=args.noise_scale,
            target_height_m=args.target_height,
            sensor_height_m=args.sensor_height,
            grid_step_m=args.grid_step,
            save_dir=args.save_dir,
            show=args.show,
            pause_s=args.pause,
        )
        simulator = TerrainScanSimulator(config)
        simulator.run()
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f"error: {exc}")
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
