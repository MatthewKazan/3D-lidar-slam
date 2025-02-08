import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import time

class BagPlayer(Node):
    def __init__(self, bag_path: str, topic_name: str = "/global_map", loop: bool = False):
        super().__init__('bag_player')

        # QoS profile to match recorded data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(PointCloud2, topic_name, qos_profile)
        self.bag_path = bag_path
        self.topic_name = topic_name
        self.loop = loop

        self.get_logger().info(f"Playing bag: {bag_path} on topic: {topic_name}")

        self.play_bag()

    def play_bag(self):
        """ Reads and publishes messages from a ROS 2 bag """
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topic_types}

        try:
            while True:
                while reader.has_next():
                    (topic, msg_data, timestamp) = reader.read_next()

                    if topic == self.topic_name:
                        msg = deserialize_message(msg_data, self.get_message_type(type_map[topic]))
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"Published PointCloud2 from bag at timestamp {timestamp}")

                    time.sleep(0.1)  # Simulate real-time playback

                if not self.loop:
                    break  # Exit if not looping

                reader.seek(0)  # Restart if looping

        except KeyboardInterrupt:
            self.get_logger().info("Stopped by user")

    @staticmethod
    def get_message_type(type_name):
        from rosidl_runtime_py.utilities import get_message
        return get_message(type_name)


def main(args=None):
    rclpy.init(args=args)

    # ✅ Accept bag path as an argument
    if len(sys.argv) < 2:
        print("Usage: ros2 run <your_package> play_ros2_bag.py <bag_path>")
        return

    bag_name = sys.argv[1]  # Get the bag path from command line

    dir_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../"))
    bag_path = os.path.join(dir_path,
                               f'global_map_bag/{bag_name}')
    node = BagPlayer(bag_path, topic_name="/global_map", loop=False)
    # rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
