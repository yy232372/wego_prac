from setuptools import setup

package_name = 'wego_ez'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['lidar', 'main', 'motor'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['wego_ez_launch.py']),
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
            'distance_calculator = lidar:main',
            'safety_decision = main:main',
            'drive_limo = motor:main',
        ],
    },
)
