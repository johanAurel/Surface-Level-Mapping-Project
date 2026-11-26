from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
import tempfile
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone
from functools import wraps
from pathlib import Path
from typing import Any

try:
    import fcntl
except ImportError:
    fcntl = None

SESSIONS_DIR = Path(__file__).parent / "sessions"

_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")
_SESSION_STATUSES = ("draft", "active", "complete")
_SOURCES = ("hardware", "manual", "demo")
_NUMERIC_CONFIG_KEYS = (
    "x_length_m",
    "y_length_m",
    "block_size_m",
    "reflector_height_m",
    "wheel_circumference_m",
    "start_x_m",
    "start_y_m",
    "start_yaw_deg",
    "tilt_mount_offset_deg",
)
_DEFAULT_CONFIG: dict[str, Any] = {
    "x_length_m": 10.0,
    "y_length_m": 10.0,
    "block_size_m": 5.0,
    "points_per_block": 2,
    "reflector_height_m": 1.5,
    "wheel_circumference_m": 1.0,
    "start_x_m": 0.0,
    "start_y_m": 0.0,
    "start_yaw_deg": 0.0,
    "tilt_mount_offset_deg": 0.0,
}
_POINT_NUMERIC_FIELDS = (
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
)


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def session_fingerprint(session: dict[str, Any]) -> str:
    payload = {
        "id": session.get("id"),
        "config": session.get("config"),
        "blocks": session.get("blocks"),
        "points": session.get("points"),
    }
    encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _validate_id(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or not _ID_RE.fullmatch(value) or len(value) > 128:
        raise ValueError(f"invalid {label}")
    return value


def sessions_dir() -> Path:
    SESSIONS_DIR.mkdir(parents=True, exist_ok=True)
    return SESSIONS_DIR


def session_path(session_id: str) -> Path:
    return sessions_dir() / f"{_validate_id(session_id, 'session id')}.json"


def new_session_id() -> str:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f")
    return f"{stamp}-{uuid.uuid4().hex[:8]}"


def _locked_session_write(func):
    @wraps(func)
    def wrapper(session_id: str, *args: Any, **kwargs: Any):
        with _session_lock(session_id):
            return func(session_id, *args, **kwargs)
    return wrapper


@contextmanager
def _session_lock(session_id: str):
    resolved_id = _validate_id(session_id, "session id")
    sessions_dir()
    lock_path = SESSIONS_DIR / f".{resolved_id}.lock"
    with lock_path.open("a+") as handle:
        if fcntl is None:
            raise RuntimeError("session locking requires fcntl; run the backend on Linux")
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)


def _strict_float(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _strict_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be an integer")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be an integer") from exc
    if not math.isfinite(number) or not number.is_integer():
        raise ValueError(f"{label} must be an integer")
    return int(number)


def _as_float(value: Any, default: float = 0.0) -> float:
    if value is None or isinstance(value, bool):
        return float(default)
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not math.isfinite(result):
        return float(default)
    return result


def _as_int(value: Any, default: int = 0) -> int:
    if value is None or isinstance(value, bool):
        return int(default)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return int(default)
    if not math.isfinite(number) or not number.is_integer():
        return int(default)
    return int(number)


def _as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "y", "on"}:
            return True
        if lowered in {"0", "false", "no", "n", "off", ""}:
            return False
    return bool(default)


def _normalize_config(config: Any) -> tuple[dict[str, Any], list[list[int]], int, int]:
    if not isinstance(config, dict):
        raise ValueError("config must be an object")
    numeric = {key: _strict_float(config.get(key, default), key) for key, default in _DEFAULT_CONFIG.items() if key in _NUMERIC_CONFIG_KEYS}
    x_length = numeric["x_length_m"]
    y_length = numeric["y_length_m"]
    block_size = numeric["block_size_m"]
    if x_length <= 0 or y_length <= 0 or block_size <= 0:
        raise ValueError("field lengths and block_size_m must be positive")
    cols = int(round(x_length / block_size))
    rows = int(round(y_length / block_size))
    if rows < 1 or cols < 1:
        raise ValueError("field must contain at least one block")
    if not math.isclose(cols * block_size, x_length, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("block_size_m must divide x_length_m exactly")
    if not math.isclose(rows * block_size, y_length, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("block_size_m must divide y_length_m exactly")

    raw_counts = config.get("points_per_block", _DEFAULT_CONFIG["points_per_block"])
    if isinstance(raw_counts, (list, tuple)):
        if len(raw_counts) != rows or any(not isinstance(row, (list, tuple)) or len(row) != cols for row in raw_counts):
            raise ValueError(f"points_per_block must be a scalar or a {rows}x{cols} matrix")
        counts = [[_strict_int(value, "points_per_block") for value in row] for row in raw_counts]
        if any(value < 0 for row in counts for value in row):
            raise ValueError("block point counts cannot be negative")
        stored_counts: int | list[list[int]] = counts
    else:
        scalar = _strict_int(raw_counts, "points_per_block")
        if scalar < 1:
            raise ValueError("points_per_block must be at least 1")
        counts = [[scalar for _ in range(cols)] for _ in range(rows)]
        stored_counts = scalar

    ordered = {
        "x_length_m": x_length,
        "y_length_m": y_length,
        "block_size_m": block_size,
        "points_per_block": stored_counts,
        "reflector_height_m": numeric["reflector_height_m"],
        "wheel_circumference_m": numeric["wheel_circumference_m"],
        "start_x_m": numeric["start_x_m"],
        "start_y_m": numeric["start_y_m"],
        "start_yaw_deg": numeric["start_yaw_deg"],
        "tilt_mount_offset_deg": numeric["tilt_mount_offset_deg"],
    }
    return ordered, counts, rows, cols


def block_targets(config: Any) -> tuple[list[list[int]], int, int]:
    _, counts, rows, cols = _normalize_config(config)
    return counts, rows, cols


def _clamp(value: int, upper: int) -> int:
    return max(0, min(upper, value))


def _block_from_xy(x_m: float, y_m: float, config: dict[str, Any], rows: int, cols: int) -> tuple[int, int]:
    block_size = config["block_size_m"]
    row = _clamp(int(math.floor(y_m / block_size)), rows - 1)
    col = _clamp(int(math.floor(x_m / block_size)), cols - 1)
    return row, col


def _build_blocks(
    config: dict[str, Any],
    counts: list[list[int]],
    rows: int,
    cols: int,
    previous: Any,
    timestamp: str,
) -> list[dict[str, Any]]:
    old: dict[tuple[int, int], dict[str, Any]] = {}
    if isinstance(previous, list):
        for block in previous:
            if not isinstance(block, dict):
                continue
            key = (_as_int(block.get("row"), -1), _as_int(block.get("col"), -1))
            old[key] = block
    blocks: list[dict[str, Any]] = []
    for row in range(rows):
        for col in range(cols):
            existing = old.get((row, col), {})
            blocks.append(
                {
                    "row": row,
                    "col": col,
                    "target_count": counts[row][col],
                    "status": existing.get("status", "pending") if isinstance(existing.get("status"), str) else "pending",
                    "point_ids": [],
                    "demo_simulated_count": max(0, _as_int(existing.get("demo_simulated_count"), 0)),
                    "demo_next_index": max(0, _as_int(existing.get("demo_next_index"), 0)),
                    "updated_at": existing.get("updated_at") if isinstance(existing.get("updated_at"), str) else timestamp,
                }
            )
    return blocks


def _make_point(
    point: Any,
    config: dict[str, Any],
    rows: int,
    cols: int,
    existing_ids: set[str],
    timestamp: str,
    default_source: str = "manual",
    default_scan_index: int | None = None,
) -> dict[str, Any]:
    if not isinstance(point, dict):
        raise ValueError("point must be an object")
    point_id = point.get("id")
    if not isinstance(point_id, str) or not _ID_RE.fullmatch(point_id) or point_id in existing_ids:
        point_id = f"p-{uuid.uuid4().hex[:8]}"
        while point_id in existing_ids:
            point_id = f"p-{uuid.uuid4().hex[:8]}"

    values = {field: _as_float(point.get(field), 0.0) for field in _POINT_NUMERIC_FIELDS}
    if point.get("reflector_height_m") is None:
        values["reflector_height_m"] = _as_float(config["reflector_height_m"], 0.0)

    computed_row, computed_col = _block_from_xy(values["x_m"], values["y_m"], config, rows, cols)
    row = computed_row if point.get("block_row") is None else _as_int(point.get("block_row"), computed_row)
    col = computed_col if point.get("block_col") is None else _as_int(point.get("block_col"), computed_col)
    row = _clamp(row, rows - 1)
    col = _clamp(col, cols - 1)

    scan_index = _as_int(point.get("scan_index"), default_scan_index or 1)
    if scan_index < 1:
        scan_index = 1
    point_timestamp = point.get("timestamp")
    if not isinstance(point_timestamp, str) or not point_timestamp:
        point_timestamp = timestamp

    source = point.get("source", default_source)
    if not isinstance(source, str) or source.lower() not in _SOURCES:
        source = default_source if default_source in _SOURCES else "manual"
    else:
        source = source.lower()

    raw = point.get("raw")
    return {
        "id": point_id,
        "block_row": row,
        "block_col": col,
        "scan_index": scan_index,
        "timestamp": point_timestamp,
        "x_m": values["x_m"],
        "y_m": values["y_m"],
        "depth_mm": values["depth_mm"],
        "lidar_distance_m": values["lidar_distance_m"],
        "theta_v_deg": values["theta_v_deg"],
        "theta_h_deg": values["theta_h_deg"],
        "imu_pitch_deg": values["imu_pitch_deg"],
        "yaw_deg": values["yaw_deg"],
        "platform_x_m": values["platform_x_m"],
        "platform_y_m": values["platform_y_m"],
        "reflector_height_m": values["reflector_height_m"],
        "accepted": _as_bool(point.get("accepted"), True),
        "source": source,
        "raw": dict(raw) if isinstance(raw, dict) else {},
    }


def _finalize(session: Any) -> dict[str, Any]:
    if not isinstance(session, dict):
        raise ValueError("session must be an object")
    work = copy.deepcopy(session)
    session_id = _validate_id(work.get("id"), "session id")
    timestamp = _now()
    config, counts, rows, cols = _normalize_config(work.get("config"))
    forced_status = work.pop("_force_status", None)

    blocks = _build_blocks(config, counts, rows, cols, work.get("blocks"), timestamp)
    block_map = {(block["row"], block["col"]): block for block in blocks}

    raw_points = work.get("points")
    if not isinstance(raw_points, list):
        raise ValueError("points must be a list")
    points: list[dict[str, Any]] = []
    existing_ids: set[str] = set()
    for raw_point in raw_points:
        point = _make_point(raw_point, config, rows, cols, existing_ids, timestamp)
        existing_ids.add(point["id"])
        points.append(point)

    points.sort(
        key=lambda point: (
            point["block_row"],
            point["block_col"],
            point["scan_index"],
            str(point["timestamp"]),
            point["id"],
        )
    )
    accepted_points = [point for point in points if point["accepted"]]
    counters: dict[tuple[int, int], int] = {}
    for point in points:
        key = (point["block_row"], point["block_col"])
        counters[key] = counters.get(key, 0) + 1
        point["scan_index"] = counters[key]
        if point["accepted"]:
            block_map[key]["point_ids"].append(point["id"])

    for block in blocks:
        count = len(block["point_ids"]) + _as_int(block.get("demo_simulated_count"), 0)
        target = block["target_count"]
        previous = block["status"]
        if previous == "needs_rescan" and count == 0:
            status = "needs_rescan"
        elif count >= target:
            status = "complete"
        elif count > 0:
            status = "scanning"
        elif target > 0:
            status = "pending"
        else:
            status = "complete"
        if status != previous:
            block["updated_at"] = timestamp
        block["status"] = status

    if not accepted_points:
        overall_status = "draft"
    elif all(block["status"] == "complete" for block in blocks):
        overall_status = "complete"
    else:
        overall_status = "active"

    created_at = work.get("created_at")
    if not isinstance(created_at, str) or not created_at:
        created_at = timestamp
    name = work.get("name")
    if not isinstance(name, str) or not name:
        name = "Field scan"

    result = {
        "schema_version": 1,
        "id": session_id,
        "name": name,
        "created_at": created_at,
        "updated_at": timestamp,
        "status": overall_status,
        "config": config,
        "blocks": blocks,
        "points": points,
    }
    artifacts = work.get("artifacts")
    fingerprint = session_fingerprint(result)
    if isinstance(artifacts, dict):
        figure_artifact = artifacts.get("figure")
        nested_fingerprint = figure_artifact.get("source_fingerprint") if isinstance(figure_artifact, dict) else None
        if artifacts.get("source_fingerprint") == fingerprint or nested_fingerprint == fingerprint:
            preserved = copy.deepcopy(artifacts)
            preserved["source_fingerprint"] = fingerprint
            if isinstance(preserved.get("figure"), dict):
                preserved["figure"]["source_fingerprint"] = fingerprint
            result["artifacts"] = preserved
    if forced_status in _SESSION_STATUSES:
        result["status"] = forced_status
    return result


def _atomic_write(path: Path, session: dict[str, Any]) -> None:
    fd, temp_name = tempfile.mkstemp(dir=path.parent, prefix=f".{path.stem}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(session, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp_name, path)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)


def _write_session(session: dict) -> dict:
    finalized = _finalize(session)
    _atomic_write(session_path(finalized["id"]), finalized)
    return finalized


def save_session(session: dict) -> dict:
    session_id = _validate_id(session.get("id") if isinstance(session, dict) else None, "session id")
    with _session_lock(session_id):
        return _write_session(session)


def update_session(session_id: str, mutator: Any) -> dict:
    with _session_lock(session_id):
        session = load_session(session_id)
        updated = mutator(session)
        return _write_session(updated if isinstance(updated, dict) else session)


def load_session(session_id: str) -> dict:
    path = session_path(session_id)
    try:
        with path.open("r", encoding="utf-8") as handle:
            session = json.load(handle)
    except FileNotFoundError:
        raise
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid session file: {session_id}") from exc
    if not isinstance(session, dict):
        raise ValueError(f"invalid session file: {session_id}")
    if session.get("id") != session_id:
        raise ValueError("session id mismatch")
    return session


def session_summary(session: dict) -> dict:
    blocks = session.get("blocks") if isinstance(session.get("blocks"), list) else []
    points = session.get("points") if isinstance(session.get("points"), list) else []
    target = sum(_as_int(block.get("target_count"), 0) for block in blocks if isinstance(block, dict))
    blocks_total = len(blocks)
    blocks_complete = sum(
        1 for block in blocks if isinstance(block, dict) and block.get("status") == "complete"
    )
    accepted_points = sum(1 for point in points if isinstance(point, dict) and point.get("accepted") is not False)
    accepted_points += sum(
        max(0, _as_int(block.get("demo_simulated_count"), 0))
        for block in blocks
        if isinstance(block, dict)
    )
    completion = {
        "points": accepted_points,
        "target_points": target,
        "blocks_complete": blocks_complete,
        "blocks_total": blocks_total,
        "percent": round(min(100.0, 100.0 * accepted_points / target), 1) if target > 0 else 0.0,
    }
    config = session.get("config")
    return {
        "id": session.get("id"),
        "name": session.get("name", "Field scan"),
        "updated_at": session.get("updated_at"),
        "status": session.get("status", "draft"),
        "config": dict(config) if isinstance(config, dict) else {},
        "completion": completion,
    }


def list_sessions() -> list[dict]:
    summaries: list[dict] = []
    for path in sessions_dir().glob("*.json"):
        try:
            with path.open("r", encoding="utf-8") as handle:
                session = json.load(handle)
            if isinstance(session, dict):
                summaries.append(session_summary(session))
        except (OSError, ValueError, json.JSONDecodeError):
            continue
    summaries.sort(key=lambda item: str(item.get("updated_at") or ""), reverse=True)
    return summaries


def _new_session(session_id: str, config: dict, name: str = "Field scan") -> dict[str, Any]:
    timestamp = _now()
    return {
        "schema_version": 1,
        "id": _validate_id(session_id, "session id"),
        "name": name if isinstance(name, str) and name else "Field scan",
        "created_at": timestamp,
        "updated_at": timestamp,
        "status": "draft",
        "config": config,
        "blocks": [],
        "points": [],
    }


def create_session(config: dict, name: str = "Field scan", session_id: str | None = None) -> dict:
    resolved_id = _validate_id(session_id, "session id") if session_id is not None else None
    for _ in range(1 if resolved_id else 10):
        candidate = resolved_id or new_session_id()
        with _session_lock(candidate):
            if session_path(candidate).exists():
                if resolved_id is not None:
                    raise ValueError(f"session already exists: {resolved_id}")
                continue
            return _write_session(_new_session(candidate, config, name))
    raise RuntimeError("could not allocate a session id")


def _add_point_to_session(
    session: dict,
    point: dict,
    default_source: str = "manual",
    requested_index: Any = None,
) -> dict:
    config, _, rows, cols = _normalize_config(session.get("config"))
    timestamp = _now()
    existing_ids = {
        item["id"]
        for item in session.get("points", [])
        if isinstance(item, dict) and isinstance(item.get("id"), str)
    }
    normalized = _make_point(point, config, rows, cols, existing_ids, timestamp, default_source)
    points = session.setdefault("points", [])
    if not isinstance(points, list):
        raise ValueError("points must be a list")
    if requested_index is None:
        points.append(normalized)
    else:
        wanted = _as_int(requested_index, normalized["scan_index"])
        position = len(points)
        for index, existing in enumerate(points):
            if (
                isinstance(existing, dict)
                and existing.get("block_row") == normalized["block_row"]
                and existing.get("block_col") == normalized["block_col"]
                and _as_int(existing.get("scan_index"), 0) >= wanted
            ):
                position = index
                break
        points.insert(position, normalized)
    return session


@_locked_session_write
def add_point(session_id: str, point: dict) -> dict:
    session = load_session(session_id)
    requested_index = point.get("scan_index") if isinstance(point, dict) else None
    _add_point_to_session(session, point, "manual", requested_index)
    return _write_session(session)


def _find_point(session: dict, point_id: str) -> tuple[list[dict], int]:
    _validate_id(point_id, "point id")
    points = session.get("points")
    if not isinstance(points, list):
        raise ValueError("points must be a list")
    for index, point in enumerate(points):
        if isinstance(point, dict) and point.get("id") == point_id:
            return points, index
    raise KeyError(f"point not found: {point_id}")


@_locked_session_write
def update_point(session_id: str, point_id: str, patch: dict) -> dict:
    if not isinstance(patch, dict):
        raise ValueError("patch must be an object")
    session = load_session(session_id)
    points, index = _find_point(session, point_id)
    existing = points[index]
    merged = {**existing, **patch}
    merged["id"] = point_id
    if ("x_m" in patch or "y_m" in patch) and "block_row" not in patch and "block_col" not in patch:
        merged.pop("block_row", None)
        merged.pop("block_col", None)
    config, _, rows, cols = _normalize_config(session.get("config"))
    points[index] = _make_point(
        merged,
        config,
        rows,
        cols,
        {item["id"] for item in points if isinstance(item, dict) and isinstance(item.get("id"), str) and item.get("id") != point_id},
        _now(),
        str(existing.get("source", "manual")),
        _as_int(existing.get("scan_index"), 1),
    )
    return _write_session(session)


@_locked_session_write
def delete_point(session_id: str, point_id: str) -> dict:
    session = load_session(session_id)
    points, index = _find_point(session, point_id)
    points.pop(index)
    return _write_session(session)


def _clear_blocks_in_session(session: dict, keys: set[tuple[int, int]], mark_rescan: bool) -> dict:
    session["points"] = [
        point
        for point in session.get("points", [])
        if not (isinstance(point, dict) and (_as_int(point.get("block_row"), -1), _as_int(point.get("block_col"), -1)) in keys)
    ]
    for block in session.get("blocks", []):
        if isinstance(block, dict) and (_as_int(block.get("row"), -1), _as_int(block.get("col"), -1)) in keys:
            block["demo_simulated_count"] = 0
            block["demo_next_index"] = 0
            block["status"] = "needs_rescan" if mark_rescan else "draft"
    return session


@_locked_session_write
def clear_blocks(session_id: str, blocks: list[dict], mark_rescan: bool = True) -> dict:
    if not isinstance(blocks, list):
        raise ValueError("blocks must be a list")
    session = load_session(session_id)
    config, _, rows, cols = _normalize_config(session.get("config"))
    keys: set[tuple[int, int]] = set()
    for block in blocks:
        if not isinstance(block, dict):
            raise ValueError("each block must be an object")
        row = _as_int(block.get("row"), -1)
        col = _as_int(block.get("col"), -1)
        if not 0 <= row < rows or not 0 <= col < cols:
            raise ValueError("block row/col out of range")
        keys.add((row, col))
    _clear_blocks_in_session(session, keys, mark_rescan)
    return _write_session(session)


@_locked_session_write
def consume_rescan_requests(session_id: str) -> dict:
    session = load_session(session_id)
    keys = {
        (_as_int(block.get("row"), -1), _as_int(block.get("col"), -1))
        for block in session.get("blocks", [])
        if isinstance(block, dict) and block.get("status") == "needs_rescan"
    }
    if not keys:
        return session
    _clear_blocks_in_session(session, keys, False)
    return _write_session(session)


@_locked_session_write
def reconfigure_session(session_id: str, config_patch: dict) -> dict:
    if not isinstance(config_patch, dict):
        raise ValueError("config_patch must be an object")
    session = load_session(session_id)
    old_config = session.get("config") if isinstance(session.get("config"), dict) else {}
    old_blocks = session.get("blocks", [])
    config, counts, rows, cols = _normalize_config({**old_config, **config_patch})
    new_state: dict[tuple[int, int], dict[str, int]] = {}
    for point in session.get("points", []):
        if not isinstance(point, dict):
            continue
        row, col = _block_from_xy(_as_float(point.get("x_m")), _as_float(point.get("y_m")), config, rows, cols)
        point["block_row"] = row
        point["block_col"] = col
        entry = new_state.setdefault((row, col), {"simulated": 0, "next_index": 0, "stored": 0})
        entry["stored"] += 1
    old_size = _as_float(old_config.get("block_size_m"), config["block_size_m"])
    if old_size <= 0:
        old_size = config["block_size_m"]
    for block in old_blocks:
        if not isinstance(block, dict):
            continue
        old_row = _as_int(block.get("row"), -1)
        old_col = _as_int(block.get("col"), -1)
        if old_row < 0 or old_col < 0:
            continue
        center_x = (old_col + 0.5) * old_size
        center_y = (old_row + 0.5) * old_size
        row, col = _block_from_xy(center_x, center_y, config, rows, cols)
        entry = new_state.setdefault((row, col), {"simulated": 0, "next_index": 0, "stored": 0})
        entry["simulated"] += max(0, _as_int(block.get("demo_simulated_count"), 0))
        entry["next_index"] = max(entry["next_index"], max(0, _as_int(block.get("demo_next_index"), 0)))
    session["config"] = config
    session["blocks"] = _build_blocks(config, counts, rows, cols, [], _now())
    for block in session["blocks"]:
        entry = new_state.get((block["row"], block["col"]))
        if not entry:
            continue
        block["demo_simulated_count"] = entry["simulated"]
        block["demo_next_index"] = max(entry["next_index"], entry["stored"])
    return _write_session(session)


def _first_number(measurement: dict[str, Any], keys: tuple[str, ...], default: float = 0.0) -> float:
    for key in keys:
        value = measurement.get(key)
        if value is None:
            continue
        if isinstance(value, str) and not value.strip():
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            return number
    return float(default)


@_locked_session_write
def record_measurement(session_id: str, measurement: dict) -> dict:
    if not isinstance(measurement, dict):
        raise ValueError("measurement must be an object")
    try:
        session = load_session(session_id)
    except FileNotFoundError:
        size = max(1e-9, _first_number(measurement, ("block_size_m",), 5.0))
        x_length = _first_number(measurement, ("field_x_m",), 10.0)
        y_length = _first_number(measurement, ("field_y_m",), 10.0)
        scan_index = _as_int(measurement.get("block_scan_index"), 1)
        points_per_block = _as_int(measurement.get("points_per_block", max(1, scan_index)), max(1, scan_index))
        config = {
            "x_length_m": x_length,
            "y_length_m": y_length,
            "block_size_m": size,
            "points_per_block": max(1, points_per_block),
            "reflector_height_m": _first_number(measurement, ("reflector_height_m",), 1.5),
            "wheel_circumference_m": _first_number(measurement, ("wheel_circumference_m",), 1.0),
            "start_x_m": _first_number(measurement, ("start_x_m",), 0.0),
            "start_y_m": _first_number(measurement, ("start_y_m",), 0.0),
            "start_yaw_deg": _first_number(measurement, ("start_yaw_deg",), 0.0),
            "tilt_mount_offset_deg": _first_number(measurement, ("tilt_mount_offset_deg",), 0.0),
        }
        session = _new_session(session_id, config)

    raw_x = measurement.get("point_x")
    raw_y = measurement.get("point_y")
    if raw_x is None or (isinstance(raw_x, str) and not raw_x.strip()):
        raw_x = measurement.get("platform_x")
    if raw_y is None or (isinstance(raw_y, str) and not raw_y.strip()):
        raw_y = measurement.get("platform_y")
    depth_source = measurement.get("point_z")
    if depth_source is None or (isinstance(depth_source, str) and not depth_source.strip()):
        depth_source = measurement.get("z_robot_m")

    point: dict[str, Any] = {
        "x_m": _as_float(raw_x),
        "y_m": _as_float(raw_y),
        "depth_mm": _as_float(depth_source) * 1000.0,
        "lidar_distance_m": _first_number(measurement, ("lidar_distance",)),
        "theta_v_deg": _first_number(measurement, ("theta_v_deg",)),
        "theta_h_deg": _first_number(measurement, ("theta_h_deg",)),
        "imu_pitch_deg": _first_number(measurement, ("pitch", "imu_pitch_deg")),
        "yaw_deg": _first_number(measurement, ("yaw",)),
        "platform_x_m": _first_number(measurement, ("platform_x",)),
        "platform_y_m": _first_number(measurement, ("platform_y",)),
        "reflector_height_m": _first_number(measurement, ("reflector_height_m",), session["config"]["reflector_height_m"]),
        "accepted": _as_bool(measurement.get("accepted_for_scan", True), True),
        "source": "hardware",
        "raw": copy.deepcopy(measurement),
    }
    if measurement.get("block_row") is not None:
        point["block_row"] = _as_int(measurement.get("block_row"), 0)
    if measurement.get("block_col") is not None:
        point["block_col"] = _as_int(measurement.get("block_col"), 0)
    if measurement.get("block_scan_index") is not None:
        point["scan_index"] = _as_int(measurement.get("block_scan_index"), 1)
    _add_point_to_session(session, point, "hardware", point.get("scan_index"))
    return _write_session(session)
