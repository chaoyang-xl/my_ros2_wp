"""Pure 2D geometry helpers for loop-closure correction."""

from __future__ import annotations

from math import cos, sin


def compose_2d(
    x: float,
    y: float,
    yaw: float,
    px: float,
    py: float,
) -> tuple[float, float]:
    """Apply 2D rigid transform (x, y, yaw) to point (px, py)."""
    return (
        cos(yaw) * px - sin(yaw) * py + x,
        sin(yaw) * px + cos(yaw) * py + y,
    )


def inverse_2d(x: float, y: float, yaw: float) -> tuple[float, float, float]:
    """Return inverse of a 2D rigid transform."""
    c = cos(yaw)
    s = -sin(yaw)
    return (-c * x + s * y, -s * x - c * y, -yaw)


def transform_old_map_to_new_map(
    px: float,
    py: float,
    tf_old: tuple[float, float, float],
    tf_new: tuple[float, float, float],
) -> tuple[float, float]:
    """Correct old-map point into new-map coordinates.

    ``tf_old`` / ``tf_new`` are odom->map transforms before and after loop
    closure. This matches ``lookup_transform("map", "odom", ...)``.

    Formula:
        P_odom = T_old_map_odom^{-1}(P_old_map)
        P_new_map = T_new_map_odom(P_odom)
    """
    inv_old_x, inv_old_y, inv_old_yaw = inverse_2d(*tf_old)
    ox, oy = compose_2d(inv_old_x, inv_old_y, inv_old_yaw, px, py)
    return compose_2d(*tf_new, ox, oy)
