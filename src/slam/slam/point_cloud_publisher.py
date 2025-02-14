import asyncio
import os

import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, \
    HistoryPolicy
from rclpy.serialization import serialize_message
from rclpy.service import SrvTypeResponse
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
from scripts.paths import PATH_TO_ROSBAGS, generate_unique_bag_name


class PointCloudPublisher(Node):
    """
    A ROS 2 node that publishes a PointCloud2 message to the /global_map topic.
    Used to separate ros2 functionality from the slam functionality.
    """
    def __init__(self):
        super().__init__('global_map_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(PointCloud2, '/global_map',
                                                qos_profile)
        self.global_map_ref = None

        self.save_service = self.create_service(Trigger, 'save_global_map',
                                                self.save_map_callback)
        self.toggle_save_inputs = self.create_service(Trigger, 'toggle_save_inputs',
                                                self.save_inputs_callback)
        self.should_save_inputs = False

        self.bag_dir_path = str(os.path.join(PATH_TO_ROSBAGS, generate_unique_bag_name(bag_prefix="rosbags")))
        self.input_writer = None
        self.global_writer = None

        self.get_logger().info(
            "PointCloud publisher started.")

    def reset(self):
        """
        Resets the publisher node
        """
        self.global_map_ref = None
        self.should_save_inputs = False
        self.input_writer = None
        self.global_writer = None
        self.bag_dir_path = str(os.path.join(PATH_TO_ROSBAGS, generate_unique_bag_name(bag_prefix="rosbags")))

    def setup_input_rosbags(self):
        """
        Resets the publisher node
        """
        input_bag_path = os.path.join(self.bag_dir_path, "input")
        self.input_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=input_bag_path,
                                                    storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")

        self.input_writer.open(storage_options, converter_options)
        self.input_writer.create_topic(
            rosbag2_py.TopicMetadata(
                name='/input_pointcloud',
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr"
            )
        )

    def publish_point_cloud(self, points_np: np.ndarray) -> None:
        """
        Constructs a PointCloud2 message from the given Nx3 numpy array and
        publishes it to the /global_map topic.

        :param points_np: The Nx3 numpy array of points to publish

        """
        # Create ROS2 message
        self.global_map_ref = points_np
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        cloud_msg = pc2.create_cloud_xyz32(header, points_np)
        self.publisher_.publish(cloud_msg)
        self.get_logger().info(
            f"Published {len(points_np)} points to /global_map.")

    def save_map_callback(self, request, response) -> SrvTypeResponse:
        """
        Saves the current global map to a ROS bag file when requested.

        :param request: The service request
        :param response: The service response

        :return: The service response
        """
        if self.global_map_ref is None:
            response.success = False
            response.message = "No global map available to save!"
            self.get_logger().warn(response.message)
            return response

        global_bag_path = os.path.join(self.bag_dir_path, "global")
        self.global_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=global_bag_path,
                                                    storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")

        self.global_writer.open(storage_options, converter_options)
        self.global_writer.create_topic(
            rosbag2_py.TopicMetadata(
                name='/global_map',
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr"
            )
        )
        self.save_point_cloud(writer=self.global_writer, points_np=self.global_map_ref, topic_name='/global_map')
        self.get_logger().info(f"Global map saved to {self.bag_dir_path}.")

        response.success = True
        response.message = "Global map saved successfully."
        return response

    def save_inputs_callback(self, request, response) -> SrvTypeResponse:
        """
        Saves the current global map to a ROS bag file when requested.

        :param request: The service request
        :param response: The service response

        :return: The service response
        """

        self.should_save_inputs = not self.should_save_inputs
        if self.should_save_inputs:
            self.setup_input_rosbags()
        self.get_logger().info(f"Toggled saving inputs to {self.should_save_inputs}.")

        response.success = True
        response.message = f"Toggled saving inputs to {self.should_save_inputs}."
        return response

    def save_point_cloud(self, writer: rosbag2_py.SequentialWriter, points_np: np.ndarray, topic_name: str) -> None:
        """
        Saves the global map reference to a ROS bag file
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        cloud_msg = pc2.create_cloud_xyz32(header, points_np)

        writer.write(topic_name, serialize_message(cloud_msg),
                     self.get_clock().now().nanoseconds)

    async def save_input_pointcloud(self, points_np: np.ndarray):
        """
        Saves the input point cloud to a ROS bag file

        :param points_np: The Nx3 numpy array of points to save
        """
        if not self.should_save_inputs:
            return
        self.get_logger().info("Saving input point cloud.")
        await asyncio.to_thread(
            self.save_point_cloud,
            writer=self.input_writer,
            points_np=points_np,
            topic_name='/input_pointcloud'
        )



