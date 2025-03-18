import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

os.environ["OMP_NUM_THREADS"] = "1"


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
            output='screen',
            parameters=[{"algorithm": LaunchConfiguration('algorithm'), "config": LaunchConfiguration('config')}],

        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,  # Ensure correct WebSocket port
                # 'use_compression': True,  # Enable compression (better performance)
                'fragment_size': 1048576,  # Increase buffer size to prevent throttling
                'max_message_size': 104857600,  # Allow large message sizes (100MB)
                # 'bson_only_mode': True,  # Keep JSON support
                'unregister_timeout': 1.0,
                'retry_interval': 0.05,  # Reduce WebSocket retry time
                'tcp_nodelay': True
                # # Disable Nagle's Algorithm for faster transmission
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
            description='SLAM algorithm to use (icp or dgr)'
        ),
        DeclareLaunchArgument(
            'config',
            default_value='config.yaml',
            description='Configuration file name for the camera that collected the data'
        ),
        OpaqueFunction(function=launch_setup),
    ])
