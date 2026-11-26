"""Shared terrain plotting for simulated and real scan points."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcsetup
from matplotlib.patches import Rectangle

from backend.data.scan_area import ScanAreaConfig


@dataclass
class TerrainPlotter:
    area: ScanAreaConfig
    save_dir: Path | None = None
    show: bool = False
    pause_s: float = 0.5
    grid_step_m: float = 1.0
    block: bool = False
    auto_limits: bool = False
    height_limit_mm: float | None = None
    z_axis_limit_mm: float | None = None
    box_z_aspect: float | None = None

    def __post_init__(self) -> None:
        if self.grid_step_m <= 0:
            raise ValueError("grid_step_m must be positive")
        if self.save_dir is not None:
            self.save_dir = Path(self.save_dir)

    def _grid(self) -> tuple[np.ndarray, np.ndarray]:
        x_points = max(2, int(round(self.area.x_length_m / self.grid_step_m)) + 1)
        y_points = max(2, int(round(self.area.y_length_m / self.grid_step_m)) + 1)
        return (
            np.linspace(0, self.area.x_length_m, x_points),
            np.linspace(0, self.area.y_length_m, y_points),
        )

    def _add_outlines(self, ax: plt.Axes, row: int, col: int) -> None:
        for r in range(self.area.rows):
            for c in range(self.area.columns):
                is_current = r == row and c == col
                ax.add_patch(
                    Rectangle(
                        (c * self.area.block_size_m, r * self.area.block_size_m),
                        self.area.block_size_m,
                        self.area.block_size_m,
                        fill=False,
                        edgecolor="yellow" if is_current else "white",
                        linewidth=1.6 if is_current else 0.5,
                        alpha=1.0 if is_current else 0.5,
                        zorder=2,
                    )
                )

    def plot(
        self,
        x: np.ndarray,
        y: np.ndarray,
        z: np.ndarray,
        *,
        row: int,
        col: int,
        scan_index: int,
        total_scans: int,
        title: str = "Scan",
        filename: str | None = None,
        error: np.ndarray | None = None,
        true_surface: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
        measured_surface: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
        error_surface: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
        reflector_xy: tuple[float, float] | None = None,
    ) -> Path | None:
        x_arr = np.asarray(x, dtype=float)
        y_arr = np.asarray(y, dtype=float)
        z_arr = np.asarray(z, dtype=float)
        error_arr = None if error is None else np.asarray(error, dtype=float)
        mask = np.isfinite(x_arr) & np.isfinite(y_arr) & np.isfinite(z_arr)
        error_mask = mask & np.isfinite(error_arr) if error_arr is not None else np.zeros_like(mask)

        x_plot = x_arr[mask]
        y_plot = y_arr[mask]
        z_plot = z_arr[mask]
        error_plot = error_arr[error_mask] if error_arr is not None else None
        error_x = x_arr[error_mask]
        error_y = y_arr[error_mask]

        has_points = x_plot.size > 0
        height_surface = measured_surface if measured_surface is not None else true_surface
        scale_parts = [z_plot] if z_plot.size else []
        if height_surface is not None:
            scale_parts.append(np.asarray(height_surface[2], dtype=float).ravel())
        if scale_parts:
            combined = np.concatenate(scale_parts)
            finite = combined[np.isfinite(combined)]
            z_limit = max(1.0, float(np.max(np.abs(finite)))) if finite.size else 1.0
        else:
            z_limit = 1.0
        if self.height_limit_mm is not None:
            z_limit = self.height_limit_mm
        z_axis_limit = self.z_axis_limit_mm if self.z_axis_limit_mm is not None else z_limit
        x_limits = None
        y_limits = None
        if self.auto_limits and has_points:
            x_min, x_max = float(np.min(x_plot)), float(np.max(x_plot))
            y_min, y_max = float(np.min(y_plot)), float(np.max(y_plot))
            x_span = x_max - x_min
            y_span = y_max - y_min
            x_pad = max(x_span * 0.15, 0.25)
            y_pad = max(y_span * 0.15, 0.25)
            x_limits = (x_min - x_pad, x_max + x_pad)
            y_limits = (y_min - y_pad, y_max + y_pad)

        fig = plt.figure(figsize=(16, 5))

        ax_3d = fig.add_subplot(1, 3, 1, projection="3d")
        surface = None
        if height_surface is not None:
            x_mesh, y_mesh, height_mm = height_surface
            surface = ax_3d.plot_surface(
                x_mesh, y_mesh, height_mm, cmap="coolwarm", vmin=-z_limit, vmax=z_limit, alpha=0.7
            )
        if has_points:
            ax_3d.scatter(
                x_plot, y_plot, z_plot, c=z_plot, cmap="coolwarm", vmin=-z_limit, vmax=z_limit,
                s=18, edgecolor="k", linewidth=0.2,
            )
        box_x = x_limits[1] - x_limits[0] if x_limits is not None else self.area.x_length_m
        box_y = y_limits[1] - y_limits[0] if y_limits is not None else self.area.y_length_m
        box_z = self.box_z_aspect if self.box_z_aspect is not None else max(2 * z_limit / 1000.0, 1e-3)
        ax_3d.set_box_aspect((box_x, box_y, box_z))
        ax_3d.set_zlim(-z_axis_limit, z_axis_limit)
        if x_limits is not None and y_limits is not None:
            ax_3d.set_xlim(*x_limits)
            ax_3d.set_ylim(*y_limits)
        ax_3d.set_xlabel("X (m)")
        ax_3d.set_ylabel("Y (m)")
        ax_3d.set_zlabel("Derived height Z (mm)")
        ax_3d.set_title("Measured Z from pitch/range")
        if surface is not None:
            fig.colorbar(surface, ax=ax_3d, shrink=0.6, label="Derived Z (mm)")

        ax_height = fig.add_subplot(1, 3, 2)
        height_mesh = None
        height_points = None
        if height_surface is not None:
            x_mesh, y_mesh, height_mm = height_surface
            height_mesh = ax_height.pcolormesh(
                x_mesh, y_mesh, height_mm, cmap="coolwarm", shading="auto", vmin=-z_limit, vmax=z_limit
            )
        else:
            ax_height.set_facecolor("#f2f2f2")
        if has_points:
            height_points = ax_height.scatter(
                x_plot, y_plot, c=z_plot, cmap="coolwarm", vmin=-z_limit, vmax=z_limit,
                s=18, edgecolor="k", linewidth=0.2, zorder=3,
            )
        self._add_outlines(ax_height, row, col)
        if reflector_xy is not None:
            ax_height.plot(reflector_xy[0], reflector_xy[1], "k^", markersize=10)
        ax_height.set_xlim(*(x_limits if x_limits is not None else (0, self.area.x_length_m)))
        ax_height.set_ylim(*(y_limits if y_limits is not None else (0, self.area.y_length_m)))
        ax_height.set_aspect("equal")
        ax_height.set_xlabel("X (m)")
        ax_height.set_ylabel("Y (m)")
        ax_height.set_title("Measured Z (derived from pitch)")
        if height_points is not None or height_mesh is not None:
            fig.colorbar(height_points or height_mesh, ax=ax_height, label="Derived Z (mm)")

        ax_summary = fig.add_subplot(1, 3, 3)
        error_points = None
        error_mesh = None
        max_error = 0.0
        error_limit = 1.0
        error_data_available = error_surface is not None or (error_plot is not None and error_plot.size > 0)
        if error_surface is not None:
            x_mesh, y_mesh, error_grid = error_surface
            finite_error = np.asarray(error_grid, dtype=float)[np.isfinite(error_grid)]
            max_error = float(np.max(np.abs(finite_error))) if finite_error.size else 0.0
            if max_error > 1e-9:
                error_limit = max(max_error * 1.05, 1e-6)
                error_mesh = ax_summary.pcolormesh(
                    x_mesh, y_mesh, error_grid, cmap="coolwarm", shading="auto",
                    vmin=-error_limit, vmax=error_limit,
                )
            else:
                ax_summary.set_facecolor("#efefef")
        elif error_plot is not None and error_plot.size:
            finite_error = error_plot[np.isfinite(error_plot)]
            max_error = float(np.max(np.abs(finite_error))) if finite_error.size else 0.0
            if max_error > 1e-9:
                error_limit = max(max_error * 1.05, 1e-6)
                error_points = ax_summary.scatter(
                    error_x, error_y, c=error_plot, cmap="coolwarm",
                    vmin=-error_limit, vmax=error_limit, s=18,
                    edgecolor="k", linewidth=0.2, zorder=3,
                )
            elif has_points:
                ax_summary.scatter(x_plot, y_plot, c="tab:blue", s=18, zorder=3)
        elif has_points:
            ax_summary.scatter(x_plot, y_plot, c="tab:blue", s=18, zorder=3)
        self._add_outlines(ax_summary, row, col)
        if reflector_xy is not None:
            ax_summary.plot(reflector_xy[0], reflector_xy[1], "k^", markersize=10)
        ax_summary.set_xlim(*(x_limits if x_limits is not None else (0, self.area.x_length_m)))
        ax_summary.set_ylim(*(y_limits if y_limits is not None else (0, self.area.y_length_m)))
        ax_summary.set_aspect("equal")
        ax_summary.set_xlabel("X (m)")
        ax_summary.set_ylabel("Y (m)")
        if error_data_available:
            ax_summary.set_title(
                "Measurement error" if error_points is not None or error_mesh is not None else "Measurement error (zero)"
            )
            ax_summary.text(
                0.02, 0.02, f"max |error| = {max_error:.4f} mm",
                transform=ax_summary.transAxes, fontsize=9,
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.8},
            )
        else:
            ax_summary.set_title("Coverage")
        if error_points is not None:
            fig.colorbar(error_points, ax=ax_summary, label="Error (mm)")
        elif error_mesh is not None:
            fig.colorbar(error_mesh, ax=ax_summary, label="Error (mm)")

        fig.suptitle(
            f"{title} {scan_index}/{total_scans} | block ({row}, {col}) | "
            f"points: {self.area.point_counts[row, col]} | collected: {x_plot.size}"
        )
        fig.tight_layout(rect=(0, 0, 1, 0.94))

        output_path: Path | None = None
        if self.save_dir is not None:
            self.save_dir.mkdir(parents=True, exist_ok=True)
            output_path = self.save_dir / (filename or f"scan_{scan_index:03d}.png")
            fig.savefig(output_path, dpi=150)

        if self.show and plt.get_backend().lower() not in {
            name.lower() for name in rcsetup.non_interactive_bk
        }:
            plt.show(block=self.block)
            if not self.block and self.pause_s > 0:
                plt.pause(self.pause_s)
        plt.close(fig)
        return output_path
