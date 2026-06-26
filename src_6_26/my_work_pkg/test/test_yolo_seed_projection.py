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
"""Tests for YOLO bbox + LiDAR seed projection."""

from math import isclose, pi

from my_work_pkg.yolo_seed_projection import (
    YoloDetection2D,
    YoloLidarSeedProjector,
)


def test_pixel_to_bearing_uses_ros_left_positive_convention():
    """Left image pixels should become positive ROS +Y bearings."""
    projector = YoloLidarSeedProjector(camera_hfov_rad=1.0)

    assert projector.pixel_to_bearing(320.0, 640) == 0.0
    assert projector.pixel_to_bearing(0.0, 640) > 0.0
    assert projector.pixel_to_bearing(640.0, 640) < 0.0


def test_robust_range_returns_median_valid_scan_value():
    """Invalid scan values are ignored before taking the median."""
    projector = YoloLidarSeedProjector(scan_window_half=2)
    ranges = [99.0, 1.0, float("nan"), 1.4, float("inf"), 1.2, 99.0]

    assert projector.robust_range(ranges, 3) == 1.2


def test_project_center_detection_to_base_link_forward_seed():
    """A centered bbox should project straight ahead in base_link."""
    projector = YoloLidarSeedProjector(
        camera_hfov_rad=1.0,
        scan_window_half=0,
        min_valid_range_m=0.1,
        max_valid_range_m=5.0,
    )
    ranges = [2.0] + [float("inf")] * 359
    detection = YoloDetection2D(
        label="bench",
        confidence=0.9,
        center_u=320.0,
        image_width=640,
    )

    seed = projector.project(
        detection,
        scan_ranges=ranges,
        scan_angle_min=0.0,
        scan_angle_increment=2.0 * pi / 360.0,
    )

    assert seed is not None
    assert seed.label == "bench"
    assert seed.scan_index == 0
    assert isclose(seed.x, 2.0)
    assert isclose(seed.y, 0.0, abs_tol=1e-12)


def test_project_rejects_when_lidar_window_has_no_valid_range():
    """A visual detection without physical LiDAR support is not projected."""
    projector = YoloLidarSeedProjector(scan_window_half=1)
    detection = YoloDetection2D(
        label="bench",
        confidence=0.9,
        center_u=320.0,
        image_width=640,
    )

    seed = projector.project(
        detection,
        scan_ranges=[float("inf")] * 360,
        scan_angle_min=0.0,
        scan_angle_increment=2.0 * pi / 360.0,
    )

    assert seed is None
