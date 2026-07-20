"""Package configuration for semantic_navigation_pkg."""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'semantic_navigation_pkg'


def install_tree(directory):
    """Return data_files entries preserving one resource directory tree."""
    entries = []
    for root, _, filenames in os.walk(directory):
        if not filenames:
            continue
        sources = [os.path.join(root, filename) for filename in filenames]
        destination = os.path.join('share', package_name, root)
        entries.append((destination, sources))
    return entries


data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE', 'THIRD_PARTY_NOTICES.md']),
    (
        os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*launch.[pxy][yma]*')),
    ),
]
for resource_directory in ('config', 'urdf', 'worlds', 'models'):
    data_files.extend(install_tree(resource_directory))


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weiyu',
    maintainer_email='1074793744@qq.com',
    description='Standalone Gazebo, Nav2, and semantic object navigation.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'semantic_map_loader_node = '
            'semantic_navigation_pkg.semantic_map_loader_node:main',
            'semantic_goal_resolver_node = '
            'semantic_navigation_pkg.semantic_goal_resolver_node:main',
        ],
    },
)

