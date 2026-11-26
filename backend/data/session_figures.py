"""Stored figure artifacts for scan sessions."""

from __future__ import annotations

import csv
import json
import os
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg", force=True)
import numpy as np

from backend.data import session_store
from backend.data.demo_simulation import _terrain_height_mm
from backend.data.scan_area import ScanAreaConfig
from backend.data.terrain_plots import TerrainPlotter

FIGURES_DIR = Path(__file__).parent / "session_figures"
EXPORTS_DIR = Path(__file__).parent / "session_exports"
REPO_ROOT = Path(__file__).resolve().parents[2]
MAX_FIGURE_POINTS = 20_000
MAX_TERRAIN_GRID_CELLS = 20_000
_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")
CSV_FIELDS = (
    "id",
    "block_row",
    "block_col",
    "scan_index",
    "timestamp",
    "x_m",
    "y_m",
    "depth_mm",
    "lidar_distance_m",
    "theta_v_deg",
    "theta_h_deg",
    "imu_pitch_deg",
    "yaw_deg",
    "platform_x_m",
    "platform_y_m",
    "reflector_height_m",
    "accepted",
    "source",
    "raw",
)


def _validate_session_id(value: Any) -> str:
    if not isinstance(value, str) or not value or not _ID_RE.fullmatch(value) or len(value) > 128:
        raise ValueError("invalid session id")
    return value


def figure_path_for(session_id: str) -> Path:
    return FIGURES_DIR / _validate_session_id(session_id) / "session_figure.png"


def csv_path_for(session_id: str) -> Path:
    return EXPORTS_DIR / _validate_session_id(session_id) / "scan_points.csv"


def _relative_path(path: Path) -> str:
    resolved = Path(path).resolve()
    try:
        return str(resolved.relative_to(REPO_ROOT))
    except ValueError:
        return str(resolved)


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if np.isfinite(result) else None


def _point_error(point: dict[str, Any]) -> float | None:
    raw = point.get("raw")
    if not isinstance(raw, dict):
        return None
    for key in ("measurement_error_mm", "error_mm", "true_error_mm"):
        value = _number(raw.get(key))
        if value is not None:
            return value
    return None


def _accepted_points(session: dict[str, Any]) -> list[dict[str, Any]]:
    points = session.get("points")
    if not isinstance(points, list):
        return []
    accepted = []
    for point in points:
        if not isinstance(point, dict) or point.get("accepted") is False:
            continue
        if _number(point.get("x_m")) is None or _number(point.get("y_m")) is None:
            continue
        if _number(point.get("depth_mm")) is None:
            continue
        accepted.append(point)
    return accepted


def _sample_points(points: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
    if len(points) <= limit:
        return points
    if limit <= 1:
        return points[-1:]
    return [points[round(index * (len(points) - 1) / (limit - 1))] for index in range(limit)]


def _write_csv(path: Path, points: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(f".{path.name}.tmp")
    try:
        with temp_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
            writer.writeheader()
            for point in points:
                row = {field: point.get(field, "") for field in CSV_FIELDS}
                raw = point.get("raw")
                if isinstance(raw, (dict, list)):
                    row["raw"] = json.dumps(raw, ensure_ascii=False)
                elif raw is None:
                    row["raw"] = ""
                else:
                    row["raw"] = str(raw)
                writer.writerow(row)
        os.replace(temp_path, path)
    finally:
        if temp_path.exists():
            temp_path.unlink()


def _area_for_session(session: dict[str, Any]) -> ScanAreaConfig:
    config = session.get("config")
    if not isinstance(config, dict):
        raise ValueError("session config is missing")
    x_length = _number(config.get("x_length_m"))
    y_length = _number(config.get("y_length_m"))
    block_size = _number(config.get("block_size_m"))
    if None in (x_length, y_length, block_size):
        raise ValueError("session config is incomplete")
    points_per_block = config.get("points_per_block")
    if isinstance(points_per_block, (list, tuple)):
        counts = points_per_block
    else:
        value = _number(points_per_block)
        if value is None:
            raise ValueError("session config is incomplete")
        counts = int(value)
    return ScanAreaConfig(
        x_length_m=x_length,
        y_length_m=y_length,
        block_size_m=block_size,
        points_per_block=counts,
    )


def _point_terrain(point: dict[str, Any]) -> float | None:
    raw = point.get("raw")
    if not isinstance(raw, dict):
        return None
    for key in ("terrain_height_mm", "true_terrain_mm", "terrain_mm"):
        value = _number(raw.get(key))
        if value is not None:
            return value
    return None


def _grid_values(area: ScanAreaConfig, grid_step_m: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
    x_points = max(2, int(round(area.x_length_m / grid_step_m)) + 1)
    y_points = max(2, int(round(area.y_length_m / grid_step_m)) + 1)
    while x_points * y_points > MAX_TERRAIN_GRID_CELLS:
        grid_step_m *= 1.25
        x_points = max(2, int(round(area.x_length_m / grid_step_m)) + 1)
        y_points = max(2, int(round(area.y_length_m / grid_step_m)) + 1)
    return (
        np.linspace(0, area.x_length_m, x_points),
        np.linspace(0, area.y_length_m, y_points),
    )


def _terrain_surface(
    points: list[dict[str, Any]], area: ScanAreaConfig
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    x_values, y_values = _grid_values(area)
    x_mesh, y_mesh = np.meshgrid(x_values, y_values)
    if all(point.get("source") == "demo" for point in points):
        terrain = np.array(
            [[_terrain_height_mm(float(x), float(y)) for x in x_values] for y in y_values],
            dtype=float,
        )
        return x_mesh, y_mesh, terrain

    terrain_values = np.array(
        [value for point in points if (value := _point_terrain(point)) is not None], dtype=float
    )
    point_x = np.array(
        [
            value
            for point in points
            if _point_terrain(point) is not None and (value := _number(point.get("x_m"))) is not None
        ],
        dtype=float,
    )
    point_y = np.array(
        [
            value
            for point in points
            if _point_terrain(point) is not None and (value := _number(point.get("y_m"))) is not None
        ],
        dtype=float,
    )
    if terrain_values.size == 0 or point_x.size != terrain_values.size or point_y.size != terrain_values.size:
        return None

    terrain = np.empty_like(x_mesh, dtype=float)
    for row in range(y_mesh.shape[0]):
        for col in range(x_mesh.shape[1]):
            dx = point_x - x_mesh[row, col]
            dy = point_y - y_mesh[row, col]
            terrain[row, col] = terrain_values[int(np.argmin(dx * dx + dy * dy))]
    return x_mesh, y_mesh, terrain


def _reflector_xy(points: list[dict[str, Any]], area: ScanAreaConfig) -> tuple[float, float]:
    for point in points:
        raw = point.get("raw")
        if not isinstance(raw, dict):
            continue
        value = raw.get("reflector_xy_m")
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            x = _number(value[0])
            y = _number(value[1])
            if x is not None and y is not None:
                return x, y
        x = _number(raw.get("reflector_x_m"))
        y = _number(raw.get("reflector_y_m"))
        if x is not None and y is not None:
            return x, y
    return area.x_length_m / 2.0, 0.0


def _last_block(session: dict[str, Any], area: ScanAreaConfig) -> tuple[int, int]:
    blocks = session.get("blocks")
    if isinstance(blocks, list):
        for block in reversed(blocks):
            if not isinstance(block, dict) or block.get("status") != "complete":
                continue
            row = _number(block.get("row"))
            col = _number(block.get("col"))
            if row is not None and col is not None:
                return min(area.rows - 1, max(0, int(row))), min(area.columns - 1, max(0, int(col)))
    return area.rows - 1, area.columns - 1


def build_session_artifacts(session: dict[str, Any]) -> dict[str, Any]:
    session_id = _validate_session_id(session.get("id"))
    points = _accepted_points(session)
    if not points:
        raise ValueError("no accepted points to plot")

    area = _area_for_session(session)
    figure_path = figure_path_for(session_id)
    csv_path = csv_path_for(session_id)
    figure_path.parent.mkdir(parents=True, exist_ok=True)
    _write_csv(csv_path, points)

    plotted = _sample_points(points, MAX_FIGURE_POINTS)
    xs = np.array([_number(point.get("x_m")) for point in plotted], dtype=float)
    ys = np.array([_number(point.get("y_m")) for point in plotted], dtype=float)
    zs = np.array([_number(point.get("depth_mm")) for point in plotted], dtype=float)
    errors = np.array([_point_error(point) for point in plotted], dtype=float)
    row, col = _last_block(session, area)
    total_scans = area.rows * area.columns

    temp_figure_path = figure_path.with_name(f".{figure_path.name}.tmp.png")
    plotter = TerrainPlotter(
        area,
        save_dir=figure_path.parent,
        show=False,
        pause_s=0.0,
        grid_step_m=1.0,
        height_limit_mm=1.0,
        z_axis_limit_mm=2.0,
        box_z_aspect=25.0,
    )
    try:
        plotter.plot(
            xs,
            ys,
            zs,
            row=row,
            col=col,
            scan_index=total_scans,
            total_scans=total_scans,
            title="Scan",
            filename=temp_figure_path.name,
            error=errors,
            true_surface=_terrain_surface(plotted, area),
            reflector_xy=_reflector_xy(plotted, area),
        )
        if not temp_figure_path.is_file():
            raise RuntimeError("figure was not written")
        os.replace(temp_figure_path, figure_path)
    finally:
        if temp_figure_path.exists():
            temp_figure_path.unlink()

    generated_at = datetime.now(timezone.utc).isoformat()
    session_data_path = session_store.session_path(session_id)
    return {
        "schema_version": 1,
        "generated_at": generated_at,
        "point_count": len(points),
        "source_fingerprint": session_store.session_fingerprint(session),
        "data_path": _relative_path(session_data_path),
        "data_absolute_path": str(session_data_path.resolve()),
        "csv_path": _relative_path(csv_path),
        "csv_absolute_path": str(csv_path.resolve()),
        "figure_path": _relative_path(figure_path),
        "figure_absolute_path": str(figure_path.resolve()),
        "figure_url": f"/api/sessions/{session_id}/figure.png",
        "csv_url": f"/api/sessions/{session_id}/points.csv",
    }


def generate_session_artifacts(session_id: str) -> dict[str, Any]:
    session_id = _validate_session_id(session_id)
    with session_store._session_lock(session_id):
        session = session_store.load_session(session_id)
        current_status = session.get("status")
        artifact = build_session_artifacts(session)

        def mutate(work: dict[str, Any]) -> dict[str, Any]:
            artifacts = work.get("artifacts")
            if not isinstance(artifacts, dict):
                artifacts = {}
            artifacts["figure"] = artifact
            artifacts["updated_at"] = artifact["generated_at"]
            artifacts["source_fingerprint"] = artifact["source_fingerprint"]
            work["artifacts"] = artifacts
            if current_status in {"draft", "active", "complete"}:
                work["_force_status"] = current_status
            return work

        return session_store._write_session(mutate(session))
