import os
import sys
import time
import argparse
from distutils.util import strtobool

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, \
    HistoryPolicy
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


class BagPlayer(Node):
    """
    A ROS 2 node that reads and publishes messages from a ROS 2 bag file.
    Use with RViz to visualize global maps that have been saved to a bag file.

    Example usage:
    ros2 run slam display_bag global_map_bag/global_map_bag_0.db3 /global_map True
    """

    def __init__(self, bag_path: str, topic_name: str = "/global_map",
        loop: bool = False):
        super().__init__('bag_player')

        # QoS profile to match recorded data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(PointCloud2, topic_name,
                                                qos_profile)
        self.bag_path = bag_path
        self.topic_name = topic_name
        self.loop = loop
        self.is_shutdown = False  # Flag to handle clean shutdown

        self.get_logger().info(
            f"Playing bag: {bag_path} on topic: {topic_name}")

    def play_bag(self):
        """
        Reads messages from a ROS 2 bag and publishes them to the specified topic.
        """

        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path,
                                                    storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topic_types}

        try:
            while not self.is_shutdown and rclpy.ok():
                while reader.has_next():
                    topic, msg_data, timestamp = reader.read_next()

                    if topic == self.topic_name:
                        msg = deserialize_message(msg_data,
                                                  self.get_message_type(
                                                      type_map[topic]))
                        self.publisher_.publish(msg)

                        if rclpy.ok():
                            self.get_logger().info(
                                f"Published PointCloud2 from bag at timestamp {timestamp}")

                    time.sleep(0.1)  # Simulate real-time playback

                if not self.loop:
                    break

                reader.seek(0)  # Restart if looping
                time.sleep(1)

        except KeyboardInterrupt:
           print("Keyboard interrupt received. Shutting down.")

        finally:
            self.is_shutdown = True

    @staticmethod
    def get_message_type(type_name):
        from rosidl_runtime_py.utilities import get_message
        return get_message(type_name)


def main(args=None):
    rclpy.init(args=args)

    # Argument parsing
    parser = argparse.ArgumentParser(
        description="Play a ROS 2 bag file and publish its messages.")
    parser.add_argument("bag_path", type=str,
                        help="Path to the ROS 2 bag file.")
    parser.add_argument("topic_name", type=str, nargs="?",
                        default="/global_map",
                        help="Topic to publish messages to.")
    parser.add_argument("loop", type=lambda x: bool(strtobool(x)), nargs="?",
                        default=False, help="Loop playback (True/False).")

    args = parser.parse_args()

    # Resolve the absolute path
    dir_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../"))
    bag_path = os.path.join(dir_path, args.bag_path)

    node = BagPlayer(bag_path, topic_name=args.topic_name, loop=args.loop)

    try:
        node.play_bag()
    finally:
        if rclpy.ok():  # Prevent duplicate shutdown calls
            rclpy.shutdown()


if __name__ == '__main__':
    main()
