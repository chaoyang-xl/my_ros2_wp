"""Tests for loop-closure coordinate correction geometry."""

from math import isclose, pi

from my_work_pkg.loop_closure_geometry import transform_old_map_to_new_map
from my_work_pkg.loop_closure_guard_node import (
    deferred_resnap_ready,
    resnap_radius_for_attempt,
)


def test_loop_closure_translation_with_odom_to_map_tf():
    """If odom->map shifts by +1m in x, old map points shift by +1m."""
    old_tf = (0.0, 0.0, 0.0)
    new_tf = (1.0, 0.0, 0.0)

    nx, ny = transform_old_map_to_new_map(2.0, 3.0, old_tf, new_tf)

    assert isclose(nx, 3.0)
    assert isclose(ny, 3.0)


def test_loop_closure_rotation_with_odom_to_map_tf():
    """A 90deg odom->map rotation maps old map x-axis onto new map y-axis."""
    old_tf = (0.0, 0.0, 0.0)
    new_tf = (0.0, 0.0, pi / 2.0)

    nx, ny = transform_old_map_to_new_map(1.0, 0.0, old_tf, new_tf)

    assert isclose(nx, 0.0, abs_tol=1e-12)
    assert isclose(ny, 1.0, abs_tol=1e-12)


def test_loop_closure_uses_inverse_old_transform_first():
    """Old map coordinates are first converted back through old odom->map."""
    old_tf = (10.0, 0.0, 0.0)
    new_tf = (20.0, 0.0, 0.0)

    nx, ny = transform_old_map_to_new_map(12.0, 3.0, old_tf, new_tf)

    assert isclose(nx, 22.0)
    assert isclose(ny, 3.0)


def test_deferred_resnap_requires_a_new_map_generation():
    """The map cached at jump detection must never be used for re-snap."""
    assert not deferred_resnap_ready(4, 4, 2, 2)
    assert deferred_resnap_ready(5, 4, 2, 2)


def test_deferred_resnap_requires_stable_tf_checks():
    """A fresh map alone is insufficient while map-to-odom is settling."""
    assert not deferred_resnap_ready(5, 4, 1, 2)
    assert deferred_resnap_ready(5, 4, 2, 2)


def test_resnap_retry_radius_grows_and_is_bounded():
    """Retries expand locally without exceeding the configured safe radius."""
    radii = [
        resnap_radius_for_attempt(0.3, 0.15, 0.6, attempt)
        for attempt in range(4)
    ]
    assert all(
        isclose(actual, expected)
        for actual, expected in zip(radii, [0.3, 0.45, 0.6, 0.6])
    )
