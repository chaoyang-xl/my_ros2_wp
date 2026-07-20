import os
from glob import glob

from setuptools import find_packages, setup


package_name = "semantic_map_offline"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name,
            [
                "package.xml",
                "README.md",
                "TRACKING_AND_FUSION.md",
                "LICENSE",
                "requirements-eval.txt",
                "requirements-sam.txt",
                "requirements-yolo-world.txt",
            ],
        ),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
        (
            os.path.join("share", package_name, "config"),
            [path for path in glob("config/*") if os.path.isfile(path)],
        ),
        (
            os.path.join("share", package_name, "config", "class_list"),
            glob("config/class_list/*"),
        ),
        (
            os.path.join("share", package_name, "config", "cartographer"),
            glob("config/cartographer/*.lua"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="weiyu",
    maintainer_email="1074793744@qq.com",
    description="Offline RGB-D projection and semantic object tracking.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "offline_projector_node = semantic_map_offline.offline_projector_node:main",
            "sam_offline_projector_node = semantic_map_offline.sam_offline_projector_node:main",
            "object_fusion_node = semantic_map_offline.object_fusion_node:main",
            "compressed_rgbd_decoder_node = semantic_map_offline.compressed_rgbd_decoder_node:main",
            "odom_camera_tf_node = semantic_map_offline.odom_camera_tf_node:main",
            "yolo_world_recorder_node = semantic_map_offline.yolo_world_recorder_node:main",
            "cartographer_dataset_exporter = semantic_map_offline.cartographer_dataset_exporter_node:main",
        ],
    },
)
