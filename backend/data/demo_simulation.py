import math
import zlib

MAX_DEMO_POINTS = 1_000_000
DEMO_SAMPLE_LIMIT = 2_000
MAX_DEMO_STORED_POINTS = 20_000
_SENSOR_HEIGHT_M = 1.2
_TERRAIN_LAYERS = (
    (35.0, 0.28, 0.6, 0.18),
    (19.0, 2.1, 1.7, 0.11),
    (53.0, 1.25, 3.2, 0.14),
    (11.0, 0.7, 5.1, 0.06),
)


def sample_indices(count: int, limit: int = DEMO_SAMPLE_LIMIT) -> list[int]:
    if count <= 0 or limit <= 0:
        return []
    if count <= limit:
        return list(range(count))
    if limit <= 1:
        return [count - 1]
    indices: list[int] = []
    for step in range(limit):
        index = int(round(step * (count - 1) / (limit - 1)))
        if not indices or index > indices[-1]:
            indices.append(index)
    return indices


def generate_demo_point(
    session_id: str,
    config: dict,
    row: int,
    col: int,
    index: int,
    planned_count: int,
    timestamp: str,
) -> dict:
    size = float(config["block_size_m"])
    reflector_height = float(config["reflector_height_m"])
    mount_offset = float(config.get("tilt_mount_offset_deg", 0.0))
    reflector_x = float(config["x_length_m"]) / 2.0
    reflector_y = 0.0

    x, y = _path_position(session_id, row, col, index, planned_count, size)
    next_x, next_y = _path_position(session_id, row, col, index + 1, planned_count, size)
    heading_x = next_x - x
    heading_y = next_y - y
    if math.hypot(heading_x, heading_y) < 1e-9:
        previous_x, previous_y = _path_position(session_id, row, col, index - 1, planned_count, size)
        heading_x = x - previous_x
        heading_y = y - previous_y
    heading = math.atan2(heading_y, heading_x) if math.hypot(heading_x, heading_y) >= 1e-9 else 0.0

    terrain_mm = _terrain_height_mm(x, y)
    slope_x = (_terrain_height_mm(x + 0.25, y) - _terrain_height_mm(x - 0.25, y)) / 500.0
    slope_y = (_terrain_height_mm(x, y + 0.25) - _terrain_height_mm(x, y - 0.25)) / 500.0
    slope_along = slope_x * math.cos(heading) + slope_y * math.sin(heading)
    pitch = _clamp(math.degrees(math.atan(slope_along)) + _noise(session_id, row, col, index, 3) * 0.35, -8.0, 8.0)

    horizontal = math.hypot(x - reflector_x, y - reflector_y)
    horizontal_safe = max(horizontal, 1e-6)
    pivot_m = terrain_mm / 1000.0 + _SENSOR_HEIGHT_M
    vertical = reflector_height - pivot_m
    slant_true = math.hypot(horizontal, vertical)
    vertical_angle_true = math.degrees(math.atan2(vertical, horizontal_safe))
    bearing_world = math.degrees(math.atan2(reflector_y - y, reflector_x - x))

    slant_noise = _noise(session_id, row, col, index, 4) * 0.004
    angle_noise = _noise(session_id, row, col, index, 5) * 0.012
    height_noise = _noise(session_id, row, col, index, 6) * 0.0002
    pan_noise = _noise(session_id, row, col, index, 7) * 0.08
    yaw_noise = _noise(session_id, row, col, index, 8) * 0.4

    slant = max(0.02, slant_true + slant_noise)
    theta_v = _clamp(vertical_angle_true + angle_noise, -89.0, 89.0)
    sensor_height = _SENSOR_HEIGHT_M + height_noise
    pivot_estimated = reflector_height - slant * math.sin(math.radians(theta_v))
    depth_mm = (pivot_estimated - sensor_height) * 1000.0
    error_mm = depth_mm - terrain_mm

    tilt_angle = theta_v - pitch - mount_offset
    theta_h = _wrap_degrees(bearing_world - math.degrees(heading) + pan_noise)
    yaw = _wrap_degrees(math.degrees(heading) + yaw_noise)

    drift_phase = _noise(session_id, row, col, 0, 9) * math.pi
    platform_x = x + math.sin(index * 0.07 + drift_phase) * 0.012 + _noise(session_id, row, col, index, 10) * 0.004
    platform_y = y + math.cos(index * 0.05 + drift_phase) * 0.012 + _noise(session_id, row, col, index, 11) * 0.004

    return {
        "id": f"demo-{row:03d}-{col:03d}-{index:08d}",
        "block_row": row,
        "block_col": col,
        "scan_index": index + 1,
        "timestamp": timestamp,
        "x_m": x,
        "y_m": y,
        "depth_mm": depth_mm,
        "lidar_distance_m": slant,
        "theta_v_deg": theta_v,
        "theta_h_deg": theta_h,
        "imu_pitch_deg": pitch,
        "yaw_deg": yaw,
        "platform_x_m": platform_x,
        "platform_y_m": platform_y,
        "reflector_height_m": reflector_height,
        "accepted": True,
        "source": "demo",
        "raw": {
            "demo": True,
            "simulation": "moving-robot-v1",
            "terrain_height_mm": terrain_mm,
            "measurement_error_mm": error_mm,
            "tilt_angle_deg": tilt_angle,
            "pan_angle_world_deg": _wrap_degrees(bearing_world),
            "reflector_xy_m": [reflector_x, reflector_y],
            "sensor_height_m": sensor_height,
            "path_index": index,
            "planned_count": planned_count,
        },
    }


def _path_position(session_id: str, row: int, col: int, index: int, planned_count: int, size: float) -> tuple[float, float]:
    if planned_count <= 1:
        fraction_x = fraction_y = 0.5
    else:
        side = int(math.ceil(math.sqrt(planned_count)))
        lane = index // side
        slot = index % side
        if lane % 2:
            slot = side - 1 - slot
        cell = size / side
        jitter_x = (_noise(session_id, row, col, index, 1) - 0.5) * cell * 0.3
        jitter_y = (_noise(session_id, row, col, index, 2) - 0.5) * cell * 0.3
        fraction_x = (slot + 0.5) * cell + jitter_x
        fraction_y = (lane + 0.5) * cell + jitter_y
    fraction_x = _clamp(fraction_x, 0.0, size)
    fraction_y = _clamp(fraction_y, 0.0, size)
    return col * size + fraction_x, row * size + fraction_y


def _terrain_height_mm(x: float, y: float) -> float:
    height_m = 0.0
    for wavelength, angle, phase, amplitude in _TERRAIN_LAYERS:
        projection = x * math.cos(angle) + y * math.sin(angle)
        height_m += amplitude * math.sin(2.0 * math.pi * projection / wavelength + phase)
    return height_m * 1000.0


def _noise(session_id: str, row: int, col: int, index: int, channel: int) -> float:
    seed = f"{session_id}:{row}:{col}:{index}:{channel}".encode("utf-8")
    return (zlib.crc32(seed) / 0xFFFFFFFF) * 2.0 - 1.0


def _wrap_degrees(value: float) -> float:
    wrapped = (value + 180.0) % 360.0 - 180.0
    return wrapped


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
