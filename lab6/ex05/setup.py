from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('*.launch.py')),
        ('share/' + package_name, glob('*.urdf')),
        ('share/' + package_name, glob('*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roman',
    maintainer_email='thfjfjfjdjfh@gmail.com',
    description='Robot package with Gazebo Harmonic support',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'circle_movement = robot.circle_movement:main',
            'unique_movement = robot.unique_movement:main',
            'obstacle_avoidance = robot.obstacle_avoidance:main',
            'obstacle_avoidance_depth = robot.obstacle_avoidance_depth:main',
        ],
    },
)