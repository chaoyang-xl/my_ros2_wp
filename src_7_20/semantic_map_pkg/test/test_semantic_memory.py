"""Tests for semantic object memory state transitions."""

from semantic_map_pkg.semantic_projection import SemanticMemory, SnappedObject


def _obj(label="chair", x=1.0, y=2.0, source="snapped", pose=None):
    return SnappedObject(
        label=label,
        confidence=0.8,
        center_x=x,
        center_y=y,
        size_x=0.4,
        size_y=0.4,
        source=source,
        pose=pose,
    )


def test_memory_promotes_candidate_after_min_observations():
    memory = SemanticMemory(match_distance=1.0, min_confirmed_seen=2)

    first = memory.update(_obj(), current_time=1.0)

    assert first.state == "candidate"
    assert memory.get_active_objects() == []
    assert memory.get_active_objects(include_candidates=True) == [first]

    second = memory.update(_obj(x=1.1), current_time=2.0)

    assert second.object_id == first.object_id
    assert second.state == "confirmed"
    assert memory.get_active_objects() == [second]


def test_memory_uses_label_specific_confirmation_threshold():
    memory = SemanticMemory(
        match_distance=1.0,
        min_confirmed_seen=2,
        label_min_confirmed_seen={"person": 3},
    )

    obj = memory.update(_obj(label="person"), current_time=1.0)
    obj = memory.update(_obj(label="person", x=1.1), current_time=2.0)

    assert obj.state == "candidate"

    obj = memory.update(_obj(label="person", x=1.2), current_time=3.0)

    assert obj.state == "confirmed"


def test_memory_uses_global_threshold_without_label_override():
    memory = SemanticMemory(
        match_distance=1.0,
        min_confirmed_seen=50,
    )

    obj = None
    for i in range(11):
        obj = memory.update(_obj(label="person", x=1.0 + 0.001 * i), current_time=float(i))

    assert obj is not None
    assert obj.times_seen == 11
    assert obj.state == "candidate"
    assert memory.get_active_objects() == []


def test_memory_tracks_observation_source():
    memory = SemanticMemory(match_distance=1.0, min_confirmed_seen=1)

    obj = memory.update(_obj(source="visual_only"), current_time=1.0)

    assert obj.state == "confirmed"
    assert obj.source == "visual_only"


def test_age_removes_stale_candidate_objects():
    memory = SemanticMemory(
        match_distance=1.0,
        timeout=2.0,
        min_confirmed_seen=3,
    )

    obj = memory.update(_obj(), current_time=1.0)
    assert obj.state == "candidate"

    memory.age(current_time=3.5)

    assert memory.get_active_objects(include_candidates=True) == []


def test_age_keeps_stale_confirmed_objects():
    memory = SemanticMemory(
        match_distance=1.0,
        timeout=2.0,
        min_confirmed_seen=1,
    )

    obj = memory.update(_obj(), current_time=1.0)
    assert obj.state == "confirmed"

    memory.age(current_time=100.0)

    assert memory.get_active_objects() == [obj]


def test_memory_keeps_latest_valid_pose_and_timestamp():
    memory = SemanticMemory(match_distance=1.0, min_confirmed_seen=1)
    first_pose = {"posture": "standing", "action_tags": ["waving"]}
    obj = memory.update(
        _obj(label="person", pose=first_pose),
        current_time=1.0,
    )

    assert obj.pose == first_pose
    assert obj.pose_last_seen == 1.0

    # A detection without a matched pose must not erase the last valid pose.
    obj = memory.update(_obj(label="person", x=1.1), current_time=2.0)
    assert obj.pose == first_pose
    assert obj.pose_last_seen == 1.0

    latest_pose = {"posture": "sitting", "posture_confidence": 0.9}
    obj = memory.update(
        _obj(label="person", x=1.1, pose=latest_pose),
        current_time=3.0,
    )
    assert obj.pose == latest_pose
    assert obj.pose_last_seen == 3.0
