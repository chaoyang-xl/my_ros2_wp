# Copyright 2026 weiyu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Tests for occupancy-grid single-point semantic projection."""

from my_work_pkg.semantic_projection import (
    OccupancyGridMap,
    SemanticSeed,
    SinglePointSemanticProjector,
)


def _make_grid(width, height, occupied_cells):
    """Build a tiny occupancy grid with selected occupied cells."""
    data = [0] * (width * height)
    for gx, gy in occupied_cells:
        data[gy * width + gx] = 100
    return OccupancyGridMap(
        width=width,
        height=height,
        resolution=0.5,
        origin_x=0.0,
        origin_y=0.0,
        data=data,
    )


def test_projection_snaps_visual_seed_to_island_centroid():
    """A rough seed near a bench-like island should snap to its centroid."""
    grid = _make_grid(
        width=10,
        height=10,
        occupied_cells={
            (4, 4),
            (5, 4),
            (6, 4),
            (4, 5),
            (5, 5),
            (6, 5),
        },
    )
    projector = SinglePointSemanticProjector(
        search_radius_m=1.0,
        min_total_pixels=3,
        max_aspect_ratio=3.0,
    )
    seed = SemanticSeed(label="bench", confidence=0.9, gx=2.05, gy=2.1)

    projected = projector.project(seed, grid)

    assert projected is not None
    assert projected.label == "bench"
    assert projected.center_x == 2.75
    assert projected.center_y == 2.5
    assert projected.total_pixels == 6
    assert projected.aspect_ratio == 1.5
    assert projected.size_x == 1.5
    assert projected.size_y == 1.0


def test_projection_rejects_seed_without_nearby_occupied_cell():
    """A seed outside the search radius should not create a semantic object."""
    grid = _make_grid(width=10, height=10, occupied_cells={(8, 8)})
    projector = SinglePointSemanticProjector(search_radius_m=0.5)
    seed = SemanticSeed(label="bench", confidence=0.9, gx=1.0, gy=1.0)

    assert projector.project(seed, grid) is None


def test_projection_rejects_wall_like_aspect_ratio():
    """A long thin occupied island can be filtered as wall-like geometry."""
    grid = _make_grid(
        width=12,
        height=12,
        occupied_cells={(2, 2), (3, 2), (4, 2), (5, 2), (6, 2), (7, 2)},
    )
    projector = SinglePointSemanticProjector(
        search_radius_m=1.0,
        min_total_pixels=1,
        max_aspect_ratio=3.0,
    )
    seed = SemanticSeed(label="bench", confidence=0.9, gx=1.2, gy=1.2)

    assert projector.project(seed, grid) is None


def test_projection_rejects_island_near_map_border_when_margin_enabled():
    """Optional wall-margin filtering rejects objects too close to map edges."""
    grid = _make_grid(width=10, height=10, occupied_cells={(1, 1), (1, 2)})
    projector = SinglePointSemanticProjector(
        search_radius_m=1.0,
        min_total_pixels=1,
        wall_margin_m=1.0,
    )
    seed = SemanticSeed(label="bench", confidence=0.9, gx=0.7, gy=0.7)

    assert projector.project(seed, grid) is None
