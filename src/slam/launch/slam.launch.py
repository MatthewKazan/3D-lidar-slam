from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import logging
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    launch_nodes = [
        # Start the static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link'],
        ),

        # Start the receiver nodes
        Node(
            package='slam',
            executable='advertiser',
            name='slam_advertiser',
        ),
        Node(
            package='slam',
            executable='pub_sub',
            name='slam_pub_sub',
            parameters=[{"algorithm": LaunchConfiguration('algorithm'), "config": LaunchConfiguration('config')}],

        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,  # Ensure the correct port is set
                'use_compression': True,
                # Enable compression for better performance
            }],
        )
    ]

    # Check if RViz should be launched
    launch_rviz = context.launch_configurations.get('launch_rviz', 'false').lower() == 'true'
    if launch_rviz:
        launch_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
            )
        )

    return launch_nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Set to "true" to launch RViz'
        ),
        DeclareLaunchArgument(
            'algorithm',
            default_value='icp',
            description='SLAM algorithm to use (icp or ...)'
        ),
        DeclareLaunchArgument(
            'config',
            default_value='lidar_config.yaml',
            description='Configuration file name for the camera that collected the data'
        ),
        OpaqueFunction(function=launch_setup),
    ])
