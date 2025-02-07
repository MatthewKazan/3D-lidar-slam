import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, \
    HistoryPolicy
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('global_map_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # ✅ Only keep the latest message
        )
        self.publisher_ = self.create_publisher(PointCloud2, '/global_map', qos_profile)
        self.get_logger().info("PointCloud publisher started.")

    def publish_point_cloud(self, points_np: np.ndarray):
        # Create ROS2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        cloud_msg = pc2.create_cloud_xyz32(header, points_np)
        self.publisher_.publish(cloud_msg)
        self.get_logger().info(f"Published {len(points_np)} points to /global_map.")

