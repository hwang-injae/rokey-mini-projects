"""Geometry / depth helpers shared by search_and_track_server.

Kept dependency-free (numpy only) so it is easy to unit-test.
"""
import math

import numpy as np


def median_depth_m(depth_mm, x1, y1, x2, y2, dmin_m, dmax_m, frac=0.4, min_valid=10):
    """Median of valid depth (in metres) inside the central `frac` of a bbox.

    depth_mm : HxW uint16/np.ndarray, millimetres (0 = invalid).
    Returns None when there are not enough valid samples.
    """
    h, w = depth_mm.shape[:2]
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    bw, bh = max(1, x2 - x1), max(1, y2 - y1)
    mx, my = int(bw * (1.0 - frac) / 2.0), int(bh * (1.0 - frac) / 2.0)
    ax1, ay1 = max(0, x1 + mx), max(0, y1 + my)
    ax2, ay2 = min(w, x2 - mx), min(h, y2 - my)
    if ax2 <= ax1 or ay2 <= ay1:
        return None
    patch = depth_mm[ay1:ay2, ax1:ax2].astype(np.float32) / 1000.0
    valid = patch[(patch > dmin_m) & (patch < dmax_m)]
    if valid.size < min_valid:
        return None
    return float(np.median(valid))


def pixel_to_camera_point(u, v, z, K):
    """Back-project pixel (u, v) with depth z [m] to a 3D point in the camera frame."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return ((u - cx) * z / fx, (v - cy) * z / fy, float(z))


def yaw_to_quaternion(yaw):
    """(x, y, z, w) for a rotation about +Z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(q):
    """yaw [rad] from a geometry_msgs/Quaternion-like object."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_between(x0, y0, x1, y1):
    """Heading from (x0, y0) towards (x1, y1)."""
    return math.atan2(y1 - y0, x1 - x0)
