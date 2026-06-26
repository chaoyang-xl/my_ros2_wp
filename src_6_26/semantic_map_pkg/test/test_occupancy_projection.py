from semantic_map_pkg.projection import (
    OccupancyGrid2D,
    SemanticMapSeed,
    SinglePointOccupancyProjector,
)


def _grid_with_cells(width, height, occupied_cells):
    data = [0] * (width * height)
    for col, row in occupied_cells:
        data[row * width + col] = 100
    return OccupancyGrid2D(
        width=width,
        height=height,
        resolution=0.5,
        origin_x=0.0,
        origin_y=0.0,
        data=data,
    )


def test_single_point_projection_snaps_to_island_centroid():
    grid = _grid_with_cells(
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
    projector = SinglePointOccupancyProjector(
        search_radius_m=1.0,
        min_total_pixels=3,
        max_aspect_ratio=3.0,
    )
    seed = SemanticMapSeed(label="bench", confidence=0.9, gx=2.05, gy=2.1)

    projected = projector.project_seed(seed, grid)

    assert projected is not None
    assert projected.label == "bench"
    assert projected.center_x == 2.75
    assert projected.center_y == 2.5
    assert projected.size_x == 1.5
    assert projected.size_y == 1.0
    assert projected.total_pixels == 6
    assert projected.aspect_ratio == 1.5
    assert len(projected.points_xy) == 6


def test_single_point_projection_requires_occupied_cell_inside_radius():
    grid = _grid_with_cells(width=10, height=10, occupied_cells={(8, 8)})
    projector = SinglePointOccupancyProjector(search_radius_m=0.5)
    seed = SemanticMapSeed(label="bench", confidence=0.9, gx=1.0, gy=1.0)

    assert projector.project_seed(seed, grid) is None


def test_single_point_projection_rejects_bad_island_geometry():
    grid = _grid_with_cells(
        width=10,
        height=10,
        occupied_cells={(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)},
    )
    projector = SinglePointOccupancyProjector(
        search_radius_m=1.0,
        min_total_pixels=1,
        max_aspect_ratio=2.0,
    )
    seed = SemanticMapSeed(label="bench", confidence=0.9, gx=1.0, gy=0.75)

    assert projector.project_seed(seed, grid) is None
