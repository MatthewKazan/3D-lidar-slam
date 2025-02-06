from setuptools import find_packages, setup

package_name = 'slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs'],
    zip_safe=True,
    maintainer='mattkazan',
    maintainer_email='Mattbkazan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = slam.point_cloud_subscriber:main',
            'advertiser = slam.advertise_topic:main',
        ],
    },
)
