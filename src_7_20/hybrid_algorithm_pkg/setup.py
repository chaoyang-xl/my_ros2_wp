"""Setuptools configuration for the ROS 2 package."""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "hybrid_algorithm_pkg"


def package_files(directory):
    """Return package data entries preserving nested directories."""
    entries = []
    for path, _, filenames in os.walk(directory):
        files = [os.path.join(path, filename) for filename in filenames]
        if files:
            entries.append((os.path.join("share", package_name, path), files))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "LICENSE"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*"),
        ),
        (
            os.path.join("share", package_name, "maps"),
            glob("maps/*"),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*"),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*"),
        ),
    ] + package_files("models"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="weiyu",
    maintainer_email="weiyu@users.noreply.github.com",
    description=(
        "ROS 2 Hybrid A* global planner with obstacle-aware heuristics "
        "and pure-pursuit tracking"
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            (
                "hybrid_astar_planner = "
                "hybrid_algorithm_pkg.hybrid_algorithm_planner:main"
            ),
            (
                "pure_pursuit_controller = "
                "hybrid_algorithm_pkg.pure_pursuit_controller:main"
            ),
        ],
    },
)
