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
        reflector_xy: tuple[float, float] | None = None,
    ) -> Path | None:
        x_arr = np.asarray(x, dtype=float)
        y_arr = np.asarray(y, dtype=float)
        z_arr = np.asarray(z, dtype=float)
        error_arr = None if error is None else np.asarray(error, dtype=float)
        mask = np.isfinite(x_arr) & np.isfinite(y_arr) & np.isfinite(z_arr)
        if error_arr is not None:
            mask &= np.isfinite(error_arr)

        x_plot = x_arr[mask]
        y_plot = y_arr[mask]
        z_plot = z_arr[mask]
        error_plot = None if error_arr is None else error_arr[mask]
        has_points = x_plot.size > 0

        fig = plt.figure(figsize=(16, 5))

        ax_3d = fig.add_subplot(1, 3, 1, projection="3d")
        surface = None
        if true_surface is not None:
            x_mesh, y_mesh, true_mm = true_surface
            surface = ax_3d.plot_surface(
                x_mesh, y_mesh, true_mm, cmap="coolwarm", vmin=-1, vmax=1, alpha=0.7
            )
        if has_points:
            ax_3d.scatter(
                x_plot, y_plot, z_plot, c=z_plot, cmap="coolwarm", vmin=-1, vmax=1,
                s=18, edgecolor="k", linewidth=0.2,
            )
        ax_3d.set_box_aspect((self.area.x_length_m, self.area.y_length_m, 25))
        ax_3d.set_zlim(-2, 2)
        ax_3d.set_xlabel("X (m)")
        ax_3d.set_ylabel("Y (m)")
        ax_3d.set_zlabel("Height (mm)")
        ax_3d.set_title("Terrain + measured points")
        if surface is not None:
            fig.colorbar(surface, ax=ax_3d, shrink=0.6, label="Height (mm)")

        ax_height = fig.add_subplot(1, 3, 2)
        height_mesh = None
        height_points = None
        if true_surface is not None:
            x_mesh, y_mesh, true_mm = true_surface
            height_mesh = ax_height.pcolormesh(
                x_mesh, y_mesh, true_mm, cmap="coolwarm", shading="auto", vmin=-1, vmax=1
            )
        else:
            ax_height.set_facecolor("#f2f2f2")
        if has_points:
            height_points = ax_height.scatter(
                x_plot, y_plot, c=z_plot, cmap="coolwarm", vmin=-1, vmax=1,
                s=18, edgecolor="k", linewidth=0.2, zorder=3,
            )
        self._add_outlines(ax_height, row, col)
        if reflector_xy is not None:
            ax_height.plot(reflector_xy[0], reflector_xy[1], "k^", markersize=10)
        ax_height.set_xlim(0, self.area.x_length_m)
        ax_height.set_ylim(0, self.area.y_length_m)
        ax_height.set_aspect("equal")
        ax_height.set_xlabel("X (m)")
        ax_height.set_ylabel("Y (m)")
        ax_height.set_title("Measured height")
        if height_points is not None or height_mesh is not None:
            fig.colorbar(height_points or height_mesh, ax=ax_height, label="Height (mm)")

        ax_summary = fig.add_subplot(1, 3, 3)
        error_points = None
        if error_plot is not None and error_plot.size:
            error_limit = max(1.0, float(np.nanmax(np.abs(error_plot))))
            error_points = ax_summary.scatter(
                x_plot, y_plot, c=error_plot, cmap="coolwarm",
                vmin=-error_limit, vmax=error_limit, s=18,
                edgecolor="k", linewidth=0.2, zorder=3,
            )
        elif has_points:
            ax_summary.scatter(x_plot, y_plot, c="tab:blue", s=18, zorder=3)
        self._add_outlines(ax_summary, row, col)
        if reflector_xy is not None:
            ax_summary.plot(reflector_xy[0], reflector_xy[1], "k^", markersize=10)
        ax_summary.set_xlim(0, self.area.x_length_m)
        ax_summary.set_ylim(0, self.area.y_length_m)
        ax_summary.set_aspect("equal")
        ax_summary.set_xlabel("X (m)")
        ax_summary.set_ylabel("Y (m)")
        ax_summary.set_title("Error" if error_points is not None else "Coverage")
        if error_points is not None:
            fig.colorbar(error_points, ax=ax_summary, label="Error (mm)")

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
            plt.show(block=False)
            if self.pause_s > 0:
                plt.pause(self.pause_s)
        plt.close(fig)
        return output_path
