"""Unit tests for pure-pursuit controller helper logic."""

import math

from hybrid_algorithm_pkg.pure_pursuit_controller import (
    infer_motion_direction,
    normalize_angle,
)


def test_infer_motion_direction_detects_forward_segment():
    path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]

    assert infer_motion_direction(path, 0) == 1


def test_infer_motion_direction_detects_reverse_segment():
    path = [(0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)]

    assert infer_motion_direction(path, 0) == -1


def test_controller_angle_normalization_stays_in_principal_range():
    normalized = normalize_angle(3.0 * math.pi)

    assert -math.pi <= normalized < math.pi
