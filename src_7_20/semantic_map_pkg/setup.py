import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'semantic_map_pkg'


def install_tree(directory):
    """Return data-file entries while preserving a resource directory tree."""
    entries = []
    for root, _, filenames in os.walk(directory):
        if not filenames:
            continue
        destination = os.path.join('share', package_name, root)
        sources = [os.path.join(root, filename) for filename in filenames]
        entries.append((destination, sources))
    return entries


data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        'share/' + package_name,
        [
            'package.xml',
                'README.md',
                'LICENSE',
                'THIRD_PARTY_NOTICES.md',
                'semantic_objects_interface.md',
                'ROSBAG_OFFLINE_TEST.md',
        ],
    ),
    (
        os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*launch.[pxy][yma]*')),
    ),
]
for resource_directory in ('config', 'urdf'):
    data_files.extend(install_tree(resource_directory))


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weiyu',
    maintainer_email='1074793744@qq.com',
    description='Semantic mapping from RGB-D detections and occupancy maps.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'frontend_bridge_node = '
            'semantic_map_pkg.frontend_bridge_node:main',
            'semantic_projection_node = '
            'semantic_map_pkg.semantic_projection_node:main',
            'loop_closure_guard_node = '
            'semantic_map_pkg.loop_closure_guard_node:main',
            'raw_seed_visualizer_node = '
            'semantic_map_pkg.raw_seed_visualizer_node:main',
            'map_to_image_validator_node = '
            'semantic_map_pkg.map_to_image_validator_node:main',
            'imu_sync_node = semantic_map_pkg.imu_sync_node:main',
        ],
    },
)
