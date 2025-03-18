from abc import ABC, abstractmethod

import numpy as np
import rosbag2_py
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from scripts.data_transfer import DataTransfer


class GenericHandler(Node, ABC):
    """
    Base class for handling generic node operations.
    """

    def __init__(self, name: str, data_transfer: DataTransfer):
        """
        Initialize the GenericHandler.
        :param name: The name of the node.
        """
        super().__init__(name)
        self.data_transfer = data_transfer

    @abstractmethod
    def reset(self):
        """
        Reset the handler.
        """
        pass

    def save_point_cloud(self, writer: rosbag2_py.SequentialWriter, points, topic_name: str) -> None:
        """
        Saves the global map reference to a ROS bag file
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        cloud_msg = pc2.create_cloud_xyz32(header, points)

        writer.write(topic_name, serialize_message(cloud_msg),
                     self.get_clock().now().nanoseconds)
