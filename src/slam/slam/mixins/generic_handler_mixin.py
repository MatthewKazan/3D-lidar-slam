import rosbag2_py
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, \
    DurabilityPolicy

from rclpy.serialization import serialize_message
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

class GenericHandlerMixin:
    """
    Map a reset to a specific instance of its callback functions.
    """
    def __init_generic_handler__(self):
        reset_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._reset_cbg = ReentrantCallbackGroup()


        self.subscription = self.create_subscription(
            Empty, '/reset',
            self.reset, reset_qos,
            callback_group=self._reset_cbg,
        )
        self.get_logger().info('Generic handler has been started.')

    def save_point_cloud(self, writer: rosbag2_py.SequentialWriter, points,
        topic_name: str) -> None:
        """
        Saves the global map reference to a ROS bag file
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        cloud_msg = pc2.create_cloud_xyz32(header, points)

        writer.write(topic_name, serialize_message(cloud_msg),
                     self.get_clock().now().nanoseconds)
        self.get_logger().debug(f"Saved point cloud to {topic_name}")

    def reset(self, msg):
        """
        Reset the objects associated with this service mapping and set the event
        to signal that a reset has occurred to other processes or threads.
        """
        raise NotImplementedError("Reset method not implemented in the mixin class.")