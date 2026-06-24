from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wego_ez2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_dir={'': '.'},
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wego',
    maintainer_email='changmin@wego-robotics.com',
    description='Simple wego EZ ROS2 package for distance safety control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'distance_calculator = wego_ez2.lidar:main',
            'safety_decision = wego_ez2.main:main',
            'drive_limo = wego_ez2.motor:main',
            'scan_roi_filter = wego_ez2.scan_roi_filter:main',
            'twist_to_ackermann = wego_ez2.twist_to_ackermann:main',
            'move_to_pose = wego_ez2.move_to_pose:main',
        ],
    },
)
