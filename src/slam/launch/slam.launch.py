from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
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
            executable='listener',
            name='slam_listener',
        ),
    ]

    # Include rosbridge_server launch file
    rosbridge_launch_file = os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
    launch_nodes.append(
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch_file)
        )
    )

    # Check if RViz should be launched
    launch_rviz = context.launch_configurations.get('launch_rviz', 'false').lower() == 'true'
    if launch_rviz:
        launch_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen'
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
        OpaqueFunction(function=launch_setup),
    ])
