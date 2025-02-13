import multiprocessing
import queue
import time
from multiprocessing import Queue

import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


# OPTIONAL NEXT STEPS -- ADDING FUNCTIONALITY
#TODO - speed up ICP
#TODO - increase outlier detection for ICP -- too much noise
#TODO - add algorithm for sequential icp and global icp
#TODO - built in ros2 icp?
#TODO - run on linux, fix readme
#TODO - maybe gtsam?

### DEEP LEARNING TO DO LIST:
#TODO - figure out how to use deep learning for this
#TODO - loop closure detection, feature extraction, etc.
#TODO - deep learning + icp hybrid approach and pure deep learning slam algorithm

# FAR OFF FUTURE FEATURES - MAYBE
#TODO - have phone display global map


class PointClouds2Subscriber(Node):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters, and stores it in a global map.
    """

    def __init__(self, input_queue: Queue, reset_event: multiprocessing.Event):
        """
        Initialize the point cloud processor. Should be run in a separate process
        since it's a long-running task and slow.
        """
        super().__init__('pointcloud_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,

        )

        # ROS 2 Subscriptions
        self.pc_subscription = self.create_subscription(
            PointCloud2, '/input_pointcloud', self.listener_callback, qos_profile,
        )
        self.clear_db_subscription = self.create_subscription(
            String, '/reset', self.reset_callback, 10
        )

        self.get_logger().info('PointCloud processor has been started.')

        # Thread-Safe Queue for Processing
        self.input_queue = input_queue

        # Event Flags for Stopping & Resetting
        self.stop_event = multiprocessing.Event()
        self.reset_event = reset_event
        self.num_pcs = 0

    def listener_callback(self, msg):
        """
        Callback function that adds new point cloud data to the queue.
        """
        try:
            receive_time = time.time()
            sent_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            processing_delay = receive_time - sent_time
            self.get_logger().debug(
                f"transport delay: {processing_delay:.6f} sec")

            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            self.input_queue.put_nowait(points)

            self.num_pcs += 1
            self.get_logger().debug(f"Added new point cloud data to queue.")
            self.get_logger().debug(f"Queue size: {self.num_pcs}")

        except queue.Full:
            self.get_logger().warn("PointCloud queue is full! Dropping frame.")

    def reset_callback(self, msg):
        """
        Callback to reset the point cloud processing.
        """
        self.get_logger().info(
            "Received reset signal, clearing point cloud processing.")
        self.input_queue.empty()
        self.reset_event.set()