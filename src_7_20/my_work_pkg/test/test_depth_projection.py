"""Tests for bounding-box projection using aligned depth images."""

from math import isclose

import numpy as np

from my_work_pkg.yolo_seed_projection import (
    YoloDetection2D,
    YoloLidarSeedProjector,
)


def detection(u=2.0, v=2.0, track_id=None):
    return YoloDetection2D(
        label='chair', confidence=0.9, center_u=u, center_v=v,
        image_width=5, image_height=5, track_id=track_id,
    )


def test_center_pixel_projects_along_optical_axis():
    projector = YoloLidarSeedProjector(
        fx=100.0, fy=100.0, cx=2.0, cy=2.0, window_size=1,
    )
    seed = projector.project(detection(), np.full((5, 5), 2.0))
    assert seed is not None
    assert (seed.x, seed.y, seed.z) == (0.0, 0.0, 2.0)


def test_projection_uses_median_valid_depth():
    projector = YoloLidarSeedProjector(
        fx=100.0, fy=100.0, cx=2.0, cy=2.0, window_size=3,
    )
    depth = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 1.2, np.nan, 0.0],
        [0.0, 1.4, np.inf, 9.0, 0.0],
        [0.0, 0.1, 1.6, 1.8, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
    ])
    seed = projector.project(detection(), depth)
    assert seed is not None
    assert isclose(seed.z, 1.4)


def test_projection_applies_intrinsics():
    projector = YoloLidarSeedProjector(
        fx=100.0, fy=200.0, cx=2.0, cy=2.0, window_size=1,
    )
    seed = projector.project(
        detection(u=3.0, v=4.0), np.full((5, 5), 2.0),
    )
    assert seed is not None
    assert isclose(seed.x, 0.02)
    assert isclose(seed.y, 0.02)


def test_invalid_depth_is_rejected_and_track_id_is_preserved():
    projector = YoloLidarSeedProjector(
        fx=100.0, fy=100.0, cx=2.0, cy=2.0, window_size=1,
    )
    tracked = detection(track_id='17')
    assert projector.project(tracked, np.zeros((5, 5))) is None
    seed = projector.project(tracked, np.full((5, 5), 1.0))
    assert seed is not None
    assert seed.track_id == '17'
