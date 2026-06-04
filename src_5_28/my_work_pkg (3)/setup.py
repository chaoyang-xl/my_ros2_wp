from setuptools import find_packages, setup

package_name = 'my_work_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='weiyu',
    maintainer_email='1074793744@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'semantic_projection_node = my_work_pkg.semantic_projection_node:main',
            'yolo_seed_projection_node = my_work_pkg.yolo_seed_projection_node:main',
        ],
    },
)
