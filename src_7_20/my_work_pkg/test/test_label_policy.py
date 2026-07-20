"""Tests for semantic projection label policy defaults."""

from my_work_pkg.semantic_projection_node import DEFAULT_LABEL_POLICIES


def test_default_policy_does_not_keep_visual_fallback():
    """Generic detections should not create wall-near visual-only markers."""
    assert DEFAULT_LABEL_POLICIES["default"]["snap_to_occupancy"] is True
    assert DEFAULT_LABEL_POLICIES["default"]["fallback_visual_on_reject"] is False


def test_person_policy_keeps_visual_fallback_with_clearance():
    """People remain visual-only capable, but require free-space clearance."""
    assert DEFAULT_LABEL_POLICIES["person"]["snap_to_occupancy"] is False
    assert DEFAULT_LABEL_POLICIES["person"]["fallback_visual_on_reject"] is True
    assert DEFAULT_LABEL_POLICIES["person"]["visual_clearance_m"] > 0.0
