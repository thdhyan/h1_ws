from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'h1_debug'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thakk100',
    maintainer_email='thakk100@umn.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_capture_node = h1_debug.camera_capture_node:main',
            'visualization_grid_node = h1_debug.visualization_grid_node:main',
            'pose_detection_node = h1_debug.pose_detection_node:main',
            'pose_skeleton_node = h1_debug.pose_skeleton_node:main',
        ],
    },
)
