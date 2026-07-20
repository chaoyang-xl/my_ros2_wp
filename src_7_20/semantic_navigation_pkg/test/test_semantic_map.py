"""Tests for semantic map loading and approach-pose selection."""

import json
from math import isclose

import pytest

from semantic_navigation_pkg.semantic_map import (
    GridMap,
    SemanticObject,
    choose_approach_pose,
    load_semantic_map,
    semantic_map_snapshot,
)


def _free_grid(width=40, height=40, resolution=0.1):
    return GridMap(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=-2.0,
        origin_y=-2.0,
        data=[0] * (width * height),
    )


def test_loader_filters_person_and_candidate_objects(tmp_path):
    path = tmp_path / "semantic_objects.json"
    path.write_text(
        json.dumps(
            {
                "frame_id": "map",
                "stamp_sec": 12.5,
                "objects": [
                    {
                        "id": "table_0",
                        "label": "table",
                        "state": "confirmed",
                        "source": "snapped",
                        "x": 1.0,
                        "y": 2.0,
                        "size_x": 0.8,
                        "size_y": 0.6,
                        "confidence": 0.9,
                    },
                    {
                        "id": "person_0",
                        "label": "person",
                        "state": "confirmed",
                        "x": 1.5,
                        "y": 2.0,
                    },
                    {
                        "id": "chair_0",
                        "label": "chair",
                        "state": "candidate",
                        "x": 0.0,
                        "y": 0.0,
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    semantic_map = load_semantic_map(path)

    assert [obj.object_id for obj in semantic_map.objects] == ["table_0"]
    snapshot = semantic_map_snapshot(semantic_map)
    assert snapshot["count"] == 1
    assert snapshot["source_stamp_sec"] == 12.5


def test_loader_rejects_duplicate_ids(tmp_path):
    path = tmp_path / "duplicate.json"
    path.write_text(
        json.dumps(
            {
                "objects": [
                    {"id": "table_0", "label": "table", "x": 0, "y": 0},
                    {"id": "table_0", "label": "table", "x": 1, "y": 1},
                ]
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="duplicate"):
        load_semantic_map(path)


def test_approach_pose_uses_nearest_free_side_and_faces_object():
    obj = SemanticObject(
        object_id="table_0",
        label="table",
        x=0.0,
        y=0.0,
        size_x=0.8,
        size_y=0.6,
        confidence=0.9,
        source="snapped",
    )

    pose = choose_approach_pose(
        _free_grid(),
        obj,
        robot_x=-1.5,
        robot_y=0.0,
        standoff_m=0.5,
        clearance_m=0.2,
        sample_count=16,
    )

    assert pose is not None
    assert pose.x < 0.0
    assert isclose(pose.y, 0.0, abs_tol=1e-12)
    assert isclose(pose.yaw, 0.0, abs_tol=1e-12)


def test_approach_pose_rejects_occupied_candidates():
    grid = GridMap(
        width=20,
        height=20,
        resolution=0.1,
        origin_x=-1.0,
        origin_y=-1.0,
        data=[100] * (20 * 20),
    )
    obj = SemanticObject("table_0", "table", 0.0, 0.0, 0.5, 0.5, 0.9, "snapped")

    assert choose_approach_pose(grid, obj, -0.5, 0.0) is None

