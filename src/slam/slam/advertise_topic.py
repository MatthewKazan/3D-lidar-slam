import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String



class TopicAdvertiser(Node):
    """
    This needs to exist to advertise the topics that the ios app will use.
    The websocket cannot publish to a topic unless it is advertised first
    """
    def __init__(self):
        super().__init__('topic_advertiser')

        # Advertise the topic 'pointcloud_topic' with the String message type
        self.publisher_ = self.create_publisher(PointCloud2, '/input_pointcloud', 10)
        self.get_logger().info('PointCloud publisher node has been started')
        self.publisher_ = self.create_publisher(String, '/reset', 10)
        self.get_logger().info('reset node has been started')

    #     # self.publish_message("Sample point cloud data")
    #
    # def publish_message(self, message):
    #     msg = String()
    #     msg.data = message  # The message data being sent
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f"Publishing: '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    topic_advertiser = TopicAdvertiser()
    # Keep the publisher running
    # rclpy.spin_once()
    # rclpy.spin(topic_advertiser)
    topic_advertiser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()