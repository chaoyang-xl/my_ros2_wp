"""Unit tests for the ROS-independent Hybrid A* core."""

import math

from hybrid_algorithm_pkg.hybrid_astar import (
    _motion_is_collision_free,
    calc_2d_heuristic,
    extract_path,
    hybrid_astar,
    Node,
    normalize_angle,
    path_is_collision_free,
    SearchConfig,
)
import numpy as np
import pytest


def test_normalize_angle_stays_in_principal_range():
    for angle in (-9.0 * math.pi, -math.pi, 0.0, math.pi, 9.0 * math.pi):
        normalized = normalize_angle(angle)
        assert -math.pi <= normalized < math.pi


def test_dijkstra_heuristic_respects_obstacles():
    grid = np.zeros((7, 7), dtype=np.uint8)
    grid[1:6, 3] = 1
    distances = calc_2d_heuristic(grid, Node(5, 3, 0.0))

    direct_distance = math.hypot(5 - 1, 3 - 3)
    assert distances[3, 1] > direct_distance
    assert distances[3, 3] == pytest.approx(math.inf)


def test_motion_primitive_cannot_jump_through_wall():
    grid = np.zeros((8, 8), dtype=np.uint8)
    grid[:, 3] = 1
    config = SearchConfig(step_size=4.0, collision_check_interval=0.25)

    assert not _motion_is_collision_free(
        Node(1, 4, 0.0), 0.0, 1, grid, config
    )


def test_hybrid_astar_finds_collision_free_detour():
    grid = np.zeros((25, 25), dtype=np.uint8)
    grid[5:20, 12] = 1
    config = SearchConfig(
        step_size=1.5,
        wheelbase=3.0,
        goal_tolerance=1.5,
        max_iterations=80_000,
        collision_check_interval=0.25,
    )

    result = hybrid_astar(Node(3, 12, 0.0), Node(21, 12, 0.0), grid, config)
    path = extract_path(result)

    assert result is not None
    assert len(path) > 2
    assert path_is_collision_free(path, grid)
    assert math.hypot(path[-1][0] - 21, path[-1][1] - 12) <= 1.5


def test_invalid_or_blocked_endpoints_fail_fast():
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[8, 8] = 1

    assert hybrid_astar(Node(-1, 1, 0.0), Node(8, 8, 0.0), grid) is None
    with pytest.raises(ValueError):
        hybrid_astar(Node(1, 1, 0.0), Node(2, 2, 0.0), np.array([]))
