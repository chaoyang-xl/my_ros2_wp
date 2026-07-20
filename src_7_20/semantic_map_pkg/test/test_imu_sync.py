from builtin_interfaces.msg import Time

from semantic_map_pkg.imu_sync_node import (
    set_stamp_from_nanoseconds,
    stamp_nanoseconds,
)


def test_stamp_nanoseconds_round_trip():
    stamp = Time()
    expected = 1_783_992_380_543_729_812

    set_stamp_from_nanoseconds(stamp, expected)

    assert stamp.sec == 1_783_992_380
    assert stamp.nanosec == 543_729_812
    assert stamp_nanoseconds(stamp) == expected


def test_stamp_nanoseconds_normalizes_second_boundary():
    stamp = Time()

    set_stamp_from_nanoseconds(stamp, 2_000_000_001)

    assert stamp.sec == 2
    assert stamp.nanosec == 1
