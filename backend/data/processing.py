"""Data aggregation helpers.

PSEUDOCODE:
    FUNCTION combine_readings(timestamp, actuators, encoder, imu, lidar, camera, position):
        CREATE empty record with timestamp
        FOR each source reading:
            CONVERT dataclass/object/mapping -> dict
            MERGE into record
        RETURN flat record for CSV/logging/coordinate calculation
"""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
from typing import Any, Mapping


def combine_readings(*, timestamp: str, actuators=None, encoder=None, imu=None, lidar=None, camera=None, position=None) -> dict[str, Any]:
    """Flatten component readings into one CSV-friendly measurement record."""
    # PSEUDOCODE: start with required metadata.
    record: dict[str, Any] = {"timestamp": timestamp}
    # PSEUDOCODE: merge each component in deterministic order.
    for source in (actuators, encoder, imu, lidar, camera, position):
        record.update(_as_mapping(source))
    return record


def _as_mapping(value: Any) -> dict[str, Any]:
    # PSEUDOCODE: accept dataclass, mapping, or simple object; normalize to dict.
    if value is None:
        return {}
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, Mapping):
        return dict(value)
    if hasattr(value, "__dict__"):
        return dict(vars(value))
    raise TypeError(f"Unsupported reading type: {type(value)!r}")
