import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, \
    HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String



class TopicAdvertiser(Node):
    """
    This needs to exist to advertise the topics that the ios app will use.
    The websocket cannot publish to a topic unless it is advertised first
    """
    def __init__(self):
        super().__init__('topic_advertiser')
        reset_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Advertise the topic 'input_pointcloud' with the PointCloud2 message type
        self.publisher_ = self.create_publisher(PointCloud2, '/input_pointcloud', 10)
        self.get_logger().info('input_pointcloud topic has been advertised')
        self.publisher1_ = self.create_publisher(String, '/reset', reset_qos)
        self.get_logger().info('reset topic has been advertised')


def main(args=None):
    rclpy.init(args=args)
    topic_advertiser = TopicAdvertiser()
    rclpy.shutdown()


if __name__ == '__main__':
    main()