"""Tests for semantic object JSON snapshots."""

from my_work_pkg.semantic_projection import TrackedObject, semantic_objects_snapshot


def test_semantic_objects_snapshot_contains_current_objects():
    obj = TrackedObject(
        object_id="chair_0",
        label="chair",
        x=1.2,
        y=3.4,
        size_x=0.5,
        size_y=0.6,
        confidence=0.8,
        times_seen=3,
        last_seen=10.0,
        created_at=8.0,
        state="confirmed",
        source="snapped",
        pose={"posture": "sitting", "action_tags": ["reading"]},
        pose_last_seen=11.0,
    )

    snapshot = semantic_objects_snapshot([obj], stamp_sec=12.5)

    assert snapshot["frame_id"] == "map"
    assert snapshot["stamp_sec"] == 12.5
    assert snapshot["count"] == 1
    assert snapshot["objects"][0]["id"] == "chair_0"
    assert snapshot["objects"][0]["state"] == "confirmed"
    assert snapshot["objects"][0]["source"] == "snapped"
    assert snapshot["objects"][0]["pose"]["posture"] == "sitting"
    assert snapshot["objects"][0]["pose_last_seen"] == 11.0
