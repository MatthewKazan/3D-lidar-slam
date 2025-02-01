import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2



class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')

        # Advertise the topic 'pointcloud_topic' with the String message type
        self.publisher_ = self.create_publisher(PointCloud2, '/input_pointcloud', 10)
        self.get_logger().info('PointCloud publisher node has been started')

    #     # self.publish_message("Sample point cloud data")
    #
    # def publish_message(self, message):
    #     msg = String()
    #     msg.data = message  # The message data being sent
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f"Publishing: '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    pointcloud_publisher = PointCloudPublisher()
    # Keep the publisher running
    rclpy.spin(pointcloud_publisher)
    pointcloud_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()