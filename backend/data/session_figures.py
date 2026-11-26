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
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

from backend.data import session_store

FIGURES_DIR = Path(__file__).parent / "session_figures"
EXPORTS_DIR = Path(__file__).parent / "session_exports"
REPO_ROOT = Path(__file__).resolve().parents[2]
MAX_FIGURE_POINTS = 20_000
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


def _draw_blocks(ax: plt.Axes, session: dict[str, Any]) -> None:
    config = session.get("config")
    if not isinstance(config, dict):
        return
    block_size = _number(config.get("block_size_m"))
    if block_size is None or block_size <= 0:
        return
    blocks = session.get("blocks")
    if not isinstance(blocks, list):
        return
    for block in blocks:
        if not isinstance(block, dict):
            continue
        row = _number(block.get("row"))
        col = _number(block.get("col"))
        if row is None or col is None:
            continue
        status = block.get("status")
        color = "#4ade80" if status == "complete" else "#fbbf24" if status == "scanning" else "#334155"
        ax.add_patch(
            Rectangle(
                (col * block_size, row * block_size),
                block_size,
                block_size,
                fill=False,
                edgecolor=color,
                linewidth=0.8,
                alpha=0.7,
            )
        )


def build_session_artifacts(session: dict[str, Any]) -> dict[str, Any]:
    session_id = _validate_session_id(session.get("id"))
    points = _accepted_points(session)
    if not points:
        raise ValueError("no accepted points to plot")

    figure_path = figure_path_for(session_id)
    csv_path = csv_path_for(session_id)
    figure_path.parent.mkdir(parents=True, exist_ok=True)
    _write_csv(csv_path, points)

    plotted = _sample_points(points, MAX_FIGURE_POINTS)
    xs = np.array([_number(point.get("x_m")) for point in plotted], dtype=float)
    ys = np.array([_number(point.get("y_m")) for point in plotted], dtype=float)
    zs = np.array([_number(point.get("depth_mm")) for point in plotted], dtype=float)
    z_limit = max(1.0, float(np.max(np.abs(zs))))

    fig = plt.figure(figsize=(16, 5))

    ax_3d = fig.add_subplot(1, 3, 1, projection="3d")
    scatter_3d = ax_3d.scatter(xs, ys, zs, c=zs, cmap="coolwarm", vmin=-z_limit, vmax=z_limit, s=9)
    ax_3d.set_xlabel("X (m)")
    ax_3d.set_ylabel("Y (m)")
    ax_3d.set_zlabel("Depth (mm)")
    ax_3d.set_title("Measured surface (3D)")
    fig.colorbar(scatter_3d, ax=ax_3d, shrink=0.6, label="Depth (mm)")

    ax_depth = fig.add_subplot(1, 3, 2)
    scatter_depth = ax_depth.scatter(
        xs, ys, c=zs, cmap="coolwarm", vmin=-z_limit, vmax=z_limit, s=12, edgecolor="k", linewidth=0.2
    )
    _draw_blocks(ax_depth, session)
    ax_depth.set_aspect("equal")
    ax_depth.set_xlabel("X (m)")
    ax_depth.set_ylabel("Y (m)")
    ax_depth.set_title("Measured depth")
    fig.colorbar(scatter_depth, ax=ax_depth, label="Depth (mm)")

    ax_error = fig.add_subplot(1, 3, 3)
    error_values = [_point_error(point) for point in plotted]
    error_x = np.array([x for x, value in zip(xs, error_values) if value is not None], dtype=float)
    error_y = np.array([y for y, value in zip(ys, error_values) if value is not None], dtype=float)
    error_z = np.array([value for value in error_values if value is not None], dtype=float)
    if error_z.size:
        error_limit = max(1e-6, float(np.max(np.abs(error_z))) * 1.05)
        scatter_error = ax_error.scatter(
            error_x,
            error_y,
            c=error_z,
            cmap="coolwarm",
            vmin=-error_limit,
            vmax=error_limit,
            s=12,
            edgecolor="k",
            linewidth=0.2,
        )
        ax_error.set_title("Measurement error")
        fig.colorbar(scatter_error, ax=ax_error, label="Error (mm)")
    else:
        ax_error.scatter(xs, ys, c="#48cae4", s=12)
        ax_error.set_title("Coverage")
    _draw_blocks(ax_error, session)
    ax_error.set_aspect("equal")
    ax_error.set_xlabel("X (m)")
    ax_error.set_ylabel("Y (m)")

    fig.suptitle(f"Scan {session_id} | {len(points)} points", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    temp_figure_path = figure_path.with_name(f".{figure_path.name}.tmp.png")
    try:
        fig.savefig(temp_figure_path, dpi=140)
        os.replace(temp_figure_path, figure_path)
    finally:
        if temp_figure_path.exists():
            temp_figure_path.unlink()
        plt.close(fig)

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
