import os
import sys
from glob import glob

from setuptools import find_packages, setup

package_name = 'slam'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),  # Registers scripts as a Python package
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
        ("share/" + package_name + "/scripts/point_cloud_processors", glob("scripts/point_cloud_processors/*.py")),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
    ],
    zip_safe=True,
    maintainer='mattkazan',
    maintainer_email='Mattbkazan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'pub_sub = slam.pub_sub:main',
            'advertiser = slam.advertise_topic:main',
            'display_bag = slam.display_bag:main',
        ],
    },
)
