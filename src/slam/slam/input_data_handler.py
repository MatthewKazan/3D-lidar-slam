import multiprocessing
import os
import queue
import time
from multiprocessing import Queue

import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.service import SrvTypeResponse
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from scripts.data_transfer import DataTransfer
from scripts.paths import PATH_TO_ROSBAGS, generate_unique_bag_name
from slam.generic_data_handler import GenericHandler


class PointClouds2Subscriber(GenericHandler):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters, and stores it in a global map.
    """

    def __init__(self, data_transfer: DataTransfer):
        """
        Initialize the point cloud processor. Should be run in a separate process
        since it's a long-running task and slow.
        """
        super().__init__('pointcloud_subscriber', data_transfer)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,

        )

        # ROS 2 Subscriptions
        self.pc_subscription = self.create_subscription(
            PointCloud2, '/input_pointcloud', self.listener_callback, qos_profile,
        )

        self.get_logger().info('PointCloud processor has been started.')


        # Input saving stuff
        self.bag_dir_path = str(os.path.join(PATH_TO_ROSBAGS, 'input_bags'))
        self.input_writer = None
        self.should_save_inputs = False

        self.num_pcs = 0

    def listener_callback(self, msg):
        """
        Callback function that adds new point cloud data to the queue.
        """
        try:
            receive_time = time.time()
            sent_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            processing_delay = receive_time - sent_time
            self.get_logger().info(
                f"transport delay: {processing_delay:.6f} sec")

            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            self.data_transfer.pixel_depth_map_queue.put_nowait(points)
            if self.should_save_inputs:
                self.save_point_cloud(self.input_writer, points, "/input_pointcloud")

            self.num_pcs += 1
            self.get_logger().debug(f"Added {len(points)} new points to queue.")
            self.get_logger().debug(f"Queue size: {self.num_pcs}")

        except queue.Full:
            self.get_logger().warn("PointCloud queue is full! Dropping frame.")

    def reset(self):
        """
        Resets the subscriber node
        """
        self.num_pcs = 0
        self.input_writer = None

    def setup_input_rosbags(self):
        """
        Resets the publisher node
        """
        input_bag_path = os.path.join(self.bag_dir_path, generate_unique_bag_name(bag_prefix="inputs"))
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

    def save_inputs_callback(self, request, response) -> SrvTypeResponse:
        """
        Toggles input saving.

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


