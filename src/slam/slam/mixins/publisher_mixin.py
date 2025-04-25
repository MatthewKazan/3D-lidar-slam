import os
import queue
import threading
import time
from typing import Any

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, \
    HistoryPolicy
from rclpy.service import SrvTypeResponse
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
from scripts.paths import PATH_TO_ROSBAGS, generate_unique_bag_name

from scripts.data_transfer import DataTransfer
import open3d as o3d

from slam.mixins.generic_handler_mixin import GenericHandlerMixin


class PointCloudPublisherMixin(GenericHandlerMixin):
    """
    A ROS 2 node that publishes a PointCloud2 message to the /global_map topic.
    Used to separate ros2 functionality from the slam functionality.
    """
    def __init_pointclouds_publisher__(self):
        super().__init_generic_handler__()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(PointCloud2, '/global_map',
                                                qos_profile)


        # self.timer = self.create_timer(0.1, self.check_and_publish)

        self.bag_dir_path = str(os.path.join(PATH_TO_ROSBAGS, 'global_maps'))
        self.global_writer = None
        self.global_map_ref = None
        self.publisher_thread = threading.Thread(target=self.check_and_publish)
        self.publisher_thread.start()
        self.get_logger().info(
            "PointCloud publisher started.")

    def reset(self):
        """
        Resets the publisher node
        """
        self.global_writer = None
        self.global_map_ref = None
        self.publish_point_cloud(np.array([[0,0,0]]))

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
        self.get_logger().debug(
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

        global_bag_path = os.path.join(self.bag_dir_path, generate_unique_bag_name(bag_prefix="global_map"))
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
        self.save_point_cloud(writer=self.global_writer, points=self.global_map_ref, topic_name='/global_map')
        self.get_logger().info(f"Global map saved to {global_bag_path}.")
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(self.global_map_ref)
        o3d.io.write_point_cloud(os.path.join(global_bag_path,"global_map.ply"), point_cloud)
        del self.global_writer


        response.success = True
        response.message = "Global map saved successfully."
        return response


    def check_and_publish(self):
        """
        Check if there is new point cloud data in the queue and publish it.
        """
        while not self.data_transfer.stop_event.is_set():
            with self.data_transfer.global_map_lock:
                try:
                    self.global_map_ref = self.data_transfer.global_map_queue.get_nowait()
                except queue.Empty:
                    # No new data to publish
                    continue
                if len(self.global_map_ref) > 0:
                    self.publish_point_cloud(self.global_map_ref)
            time.sleep(.1)



def one_shot_publisher(msg: Any, topic_name: str):
    """
    A one-shot publisher that publishes a single point cloud to the /global_map topic.
    """
    node = Node('one_off_publisher')
    publisher = node.create_publisher(msg.__type__, topic_name, 10)
    publisher.publish(msg)
    node.get_logger().info(f"Published to {topic_name}.")
    # Give it a short time to process the publish
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()