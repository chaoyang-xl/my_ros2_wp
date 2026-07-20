"""Kinematically feasible Hybrid A* search on a 2D occupancy grid."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Iterable, Optional

import numpy as np


@dataclass
class SearchConfig:
    """Parameters expressed in occupancy-grid cells and radians."""

    step_size: float = 4.0
    wheelbase: float = 8.0
    max_steering_angle: float = math.radians(30.0)
    steering_samples: int = 5
    heading_bins: int = 72
    goal_tolerance: float = 2.0
    reverse_penalty: float = 1.5
    direction_change_penalty: float = 2.0
    steering_penalty: float = 0.1
    max_iterations: int = 250_000
    collision_check_interval: float = 0.5

    def __post_init__(self) -> None:
        if self.step_size <= 0 or self.wheelbase <= 0:
            raise ValueError("step_size and wheelbase must be positive")
        if self.steering_samples < 2 or self.heading_bins < 1:
            raise ValueError("steering_samples >= 2 and heading_bins >= 1 are required")
        if self.max_iterations < 1 or self.collision_check_interval <= 0:
            raise ValueError("max_iterations and collision_check_interval must be positive")


class Node:
    """A continuous vehicle state stored by the Hybrid A* search."""

    def __init__(self, x: float, y: float, theta: float, direction: int = 1):
        self.x = float(x)
        self.y = float(y)
        self.theta = normalize_angle(theta)
        self.direction = 1 if direction >= 0 else -1
        self.g = 0.0
        self.h = 0.0
        self.parent: Optional[Node] = None

    def f(self) -> float:
        return self.g + self.h

    def __lt__(self, other: "Node") -> bool:
        return self.f() < other.f()


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def calc_2d_heuristic(grid: np.ndarray, goal: Node) -> np.ndarray:
    """Compute obstacle-aware distance-to-go with an 8-connected Dijkstra search."""
    occupancy = _validate_grid(grid)
    distances = np.full(occupancy.shape, np.inf, dtype=float)
    goal_x, goal_y = int(goal.x), int(goal.y)
    if not _is_free_cell(goal_x, goal_y, occupancy):
        return distances

    motions = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )
    distances[goal_y, goal_x] = 0.0
    queue = [(0.0, goal_x, goal_y)]

    while queue:
        cost, x, y = heapq.heappop(queue)
        if cost > distances[y, x]:
            continue
        for dx, dy, motion_cost in motions:
            nx, ny = x + dx, y + dy
            if not _is_free_cell(nx, ny, occupancy):
                continue
            if dx and dy and (
                not _is_free_cell(x + dx, y, occupancy)
                or not _is_free_cell(x, y + dy, occupancy)
            ):
                continue
            new_cost = cost + motion_cost
            if new_cost < distances[ny, nx]:
                distances[ny, nx] = new_cost
                heapq.heappush(queue, (new_cost, nx, ny))
    return distances


def heuristic(node: Node, goal: Node) -> float:
    """Return an obstacle-free lower-bound estimate used by tests and callers."""
    distance = math.hypot(node.x - goal.x, node.y - goal.y)
    heading_error = abs(normalize_angle(node.theta - goal.theta))
    return distance + 0.5 * heading_error


def is_collision(node: Node, grid: np.ndarray) -> bool:
    """Return whether a state lies outside the map or in an occupied cell."""
    occupancy = _validate_grid(grid)
    return not _is_free_cell(int(node.x), int(node.y), occupancy)


def simulate_motion(
    node: Node,
    steering_angle: float,
    direction: int = 1,
    step_size: float = 4.0,
    wheelbase: float = 8.0,
) -> Node:
    """Propagate one bicycle-model motion primitive."""
    direction = 1 if direction >= 0 else -1
    x = node.x + direction * step_size * math.cos(node.theta)
    y = node.y + direction * step_size * math.sin(node.theta)
    theta = node.theta + direction * step_size / wheelbase * math.tan(steering_angle)
    return Node(x, y, theta, direction)


def hybrid_astar(
    start: Node,
    goal: Node,
    grid: np.ndarray,
    config: Optional[SearchConfig] = None,
) -> Optional[Node]:
    """
    Search for a collision-free path and return its terminal node.

    The map follows ROS OccupancyGrid indexing: grid[y, x]. Any non-zero
    value is considered occupied.
    """
    occupancy = _validate_grid(grid)
    config = config or SearchConfig()
    if is_collision(start, occupancy) or is_collision(goal, occupancy):
        return None

    h_map = calc_2d_heuristic(occupancy, goal)
    start_x, start_y = int(start.x), int(start.y)
    if not math.isfinite(h_map[start_y, start_x]):
        return None

    start.g = 0.0
    start.h = h_map[start_y, start_x]
    counter = itertools.count()
    open_list = [(start.f(), next(counter), start)]
    best_cost = {_state_key(start, config.heading_bins): 0.0}
    steering_angles = np.linspace(
        -config.max_steering_angle,
        config.max_steering_angle,
        config.steering_samples,
    )

    iterations = 0
    while open_list and iterations < config.max_iterations:
        _, _, current = heapq.heappop(open_list)
        iterations += 1
        current_key = _state_key(current, config.heading_bins)
        if current.g > best_cost.get(current_key, math.inf):
            continue
        if math.hypot(current.x - goal.x, current.y - goal.y) <= config.goal_tolerance:
            return current

        for steering_angle in steering_angles:
            for direction in (1, -1):
                new_node = simulate_motion(
                    current, float(steering_angle), direction,
                    config.step_size, config.wheelbase,
                )
                if not _motion_is_collision_free(
                    current, float(steering_angle), direction, occupancy, config,
                ):
                    continue

                x, y = int(new_node.x), int(new_node.y)
                obstacle_distance = h_map[y, x]
                if not math.isfinite(obstacle_distance):
                    continue

                motion_cost = config.step_size
                if direction < 0:
                    motion_cost *= config.reverse_penalty
                if direction != current.direction:
                    motion_cost += config.direction_change_penalty
                motion_cost += config.steering_penalty * abs(float(steering_angle))
                new_node.g = current.g + motion_cost
                new_node.h = obstacle_distance
                new_node.parent = current

                key = _state_key(new_node, config.heading_bins)
                if new_node.g >= best_cost.get(key, math.inf):
                    continue
                best_cost[key] = new_node.g
                heapq.heappush(open_list, (new_node.f(), next(counter), new_node))
    return None


def extract_path(node: Optional[Node]) -> list[tuple[float, float]]:
    """Backtrack a terminal node into start-to-goal (x, y) coordinates."""
    path = []
    while node is not None:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]


def extract_trajectory(node: Optional[Node]) -> list[tuple[float, float, float]]:
    """Backtrack a terminal node into start-to-goal poses."""
    trajectory = []
    while node is not None:
        trajectory.append((node.x, node.y, node.theta))
        node = node.parent
    return trajectory[::-1]


def _state_key(node: Node, heading_bins: int) -> tuple[int, int, int, int]:
    bin_width = 2.0 * math.pi / heading_bins
    heading_bin = int(round((normalize_angle(node.theta) + math.pi) / bin_width)) % heading_bins
    return int(node.x), int(node.y), heading_bin, node.direction


def _motion_is_collision_free(
    node: Node,
    steering_angle: float,
    direction: int,
    grid: np.ndarray,
    config: SearchConfig,
) -> bool:
    sample_count = max(1, math.ceil(config.step_size / config.collision_check_interval))
    for distance in np.linspace(config.step_size / sample_count, config.step_size, sample_count):
        sample = simulate_motion(
            node, steering_angle, direction, float(distance), config.wheelbase,
        )
        if is_collision(sample, grid):
            return False
    return True


def _is_free_cell(x: int, y: int, grid: np.ndarray) -> bool:
    return 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0] and grid[y, x] == 0


def _validate_grid(grid: np.ndarray) -> np.ndarray:
    occupancy = np.asarray(grid)
    if occupancy.ndim != 2 or occupancy.size == 0:
        raise ValueError("grid must be a non-empty 2D array")
    return occupancy


def path_is_collision_free(path: Iterable[tuple[float, float]], grid: np.ndarray) -> bool:
    """Validate that all path coordinates occupy free cells."""
    occupancy = _validate_grid(grid)
    return all(_is_free_cell(int(x), int(y), occupancy) for x, y in path)
