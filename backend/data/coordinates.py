"""Coordinate/elevation derivation for reflector-based field surveying.

PSEUDOCODE (matches described system):
    INPUTS:
        H_ref                = known reflector height
        d                    = LiDAR distance to reflector
        theta_v (tilt)       = upward tilt positive
        theta_h (pan)        = horizontal turret angle
        robot_x, robot_y     = robot position in field/map frame (optional)
        robot_yaw            = robot heading from IMU (optional)

    DERIVE:
        z_robot = H_ref - d * sin(theta_v)
        horizontal_distance_to_reflector = d * cos(theta_v)

        # Optional reflector-relative vector (if needed by downstream mapping):
        dx_reflector_frame = horizontal_distance_to_reflector * cos(theta_h)
        dy_reflector_frame = horizontal_distance_to_reflector * sin(theta_h)

    OUTPUT:
        terrain-ready elevation/geometry variables; no mesh generation.
"""

from __future__ import annotations

import math
from typing import Mapping


def calculate_robot_elevation(measurement: Mapping[str, float], reflector_height_m: float) -> dict:
    """Calculate robot elevation from a fixed reflector measurement.

    Formula from system description:
        z_robot = H_ref - d * sin(theta_v)
    where upward tilt is positive.
    """
    # PSEUDOCODE: read required measurements.
    d = _num(measurement.get("lidar_distance", measurement.get("d_m", 0.0)))
    theta_v_deg = _num(measurement.get("tilt_angle", measurement.get("theta_v", 0.0)))
    theta_h_deg = _num(measurement.get("pan_angle", measurement.get("theta_h", 0.0)))
    theta_v = math.radians(theta_v_deg)
    theta_h = math.radians(theta_h_deg)

    # PSEUDOCODE: apply reflector height geometry.
    z_robot = float(reflector_height_m) - d * math.sin(theta_v)
    horizontal_distance = d * math.cos(theta_v)

    # PSEUDOCODE: derive reflector-relative horizontal components in turret frame.
    # X/Y convention here is local: x forward, y left; adjust with calibration as needed.
    dx_local = horizontal_distance * math.cos(theta_h)
    dy_local = horizontal_distance * math.sin(theta_h)

    return {
        "reflector_height_m": float(reflector_height_m),
        "theta_v_deg": theta_v_deg,
        "theta_h_deg": theta_h_deg,
        "distance_to_reflector_m": d,
        "horizontal_distance_to_reflector_m": horizontal_distance,
        "z_robot_m": z_robot,
        "reflector_dx_local_m": dx_local,
        "reflector_dy_local_m": dy_local,
    }


def calculate_coordinates(measurement: Mapping[str, float]) -> dict:
    """Legacy/generic beam projection helper.

    Kept for compatibility with earlier scanning prototype. The primary system
    described by the project computes robot elevation from a fixed reflector.
    """
    # PSEUDOCODE: compute sensor-local beam, rotate by IMU RPY, translate by platform pose.
    platform_x = _num(measurement.get("platform_x", 0.0))
    platform_y = _num(measurement.get("platform_y", 0.0))
    platform_z = _num(measurement.get("platform_z", 0.0))
    roll = math.radians(_num(measurement.get("roll", 0.0)))
    pitch = math.radians(_num(measurement.get("pitch", 0.0)))
    yaw = math.radians(_num(measurement.get("yaw", 0.0)))
    pan = math.radians(_num(measurement.get("pan_angle", 0.0)))
    tilt = math.radians(_num(measurement.get("tilt_angle", 0.0)))
    distance = _num(measurement.get("lidar_distance", 0.0))

    beam_sensor = (
        distance * math.cos(tilt) * math.cos(pan),
        distance * math.cos(tilt) * math.sin(pan),
        distance * math.sin(tilt),
    )
    beam_map = _rotate_rpy(beam_sensor, roll, pitch, yaw)
    return {
        "x": platform_x + beam_map[0],
        "y": platform_y + beam_map[1],
        "z": platform_z + beam_map[2],
        "beam_x": beam_map[0],
        "beam_y": beam_map[1],
        "beam_z": beam_map[2],
    }


def _rotate_rpy(vector, roll: float, pitch: float, yaw: float):
    # PSEUDOCODE: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    x, y, z = vector
    x1, y1, z1 = x, y * math.cos(roll) - z * math.sin(roll), y * math.sin(roll) + z * math.cos(roll)
    x2, y2, z2 = x1 * math.cos(pitch) + z1 * math.sin(pitch), y1, -x1 * math.sin(pitch) + z1 * math.cos(pitch)
    x3, y3, z3 = x2 * math.cos(yaw) - y2 * math.sin(yaw), x2 * math.sin(yaw) + y2 * math.cos(yaw), z2
    return x3, y3, z3


def _num(value) -> float:
    # PSEUDOCODE: defensive numeric coercion.
    if value is None:
        return 0.0
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0
