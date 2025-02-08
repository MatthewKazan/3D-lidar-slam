import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, \
    HistoryPolicy
from rclpy.serialization import serialize_message
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np


class PointCloudPublisher(Node):
    def __init__(self):
        """
        :param global_map_ref: Reference to the global map (PointCloud2 object)
        """
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

        self.get_logger().info(
            "PointCloud publisher started with external global map reference.")

    def publish_point_cloud(self, points_np: np.ndarray):
        # Create ROS2 message
        self.global_map_ref = points_np
        print(points_np)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        cloud_msg = pc2.create_cloud_xyz32(header, points_np)
        self.publisher_.publish(cloud_msg)
        self.get_logger().info(
            f"Published {len(points_np)} points to /global_map.")

    def save_map_callback(self, request, response):
        """ Saves the current global map when requested """
        if not self.global_map_ref:
            response.success = False
            response.message = "No global map available to save!"
            self.get_logger().warn(response.message)
            return response

        self.save_point_cloud()
        response.success = True
        response.message = "Global map saved successfully."
        return response

    def save_point_cloud(self):
        """ Saves the global map reference to a ROS bag file """
        # TODO: Make bag file name dynamic
        self.get_logger().info("Saving global map to rosbag...")

        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri="global_map_bag",
                                                    storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")

        writer.open(storage_options, converter_options)
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name='/global_map',
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr"
            )
        )
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        cloud_msg = pc2.create_cloud_xyz32(header, self.global_map_ref)

        writer.write('/global_map', serialize_message(cloud_msg),
                     self.get_clock().now().nanoseconds)
        self.get_logger().info("Global map saved to global_map_bag.")
