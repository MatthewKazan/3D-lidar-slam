import os
import queue
import signal
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String
import multiprocessing
import time
from collections import deque
from multiprocessing import Queue
from typing import Optional

import numpy as np
import open3d as o3d

#TODO - clean up code

#TODO - reset button doesn't work good
#TODO - make bag name dynamic when saving
#TODO - speed up everything TO SLOWW
#TODO - add algorithm for sequential icp and global icp


#TODO - decide whether to use rviz or open3d for visualization
#TODO - built in ros2 icp?
#TODO - run on linux, fix readme
#TODO - maybe gtsam?

### DEEP LEARNING TO DO LIST:
#TODO - figure out how to use deep learning for this
#TODO - loop closure detection, feature extraction, etc.
#TODO - deep learning + icp hybrid approach and pure deep learning slam algorithm




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
            reliability=ReliabilityPolicy.RELIABLE,
            # Ensure reliable transmission
            history=HistoryPolicy.KEEP_ALL  # Keep all messages in the queue
        )

        # ROS 2 Subscriptions
        self.pc_subscription = self.create_subscription(
            PointCloud2, '/input_pointcloud', self.listener_callback, qos_profile
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


        #
        # # Handle system signals (SIGTERM, SIGINT)
        # signal.signal(signal.SIGINT, self.signal_handler)
        # signal.signal(signal.SIGTERM, self.signal_handler)

        # Start Processing Thread

    def listener_callback(self, msg):
        """
        Callback function that adds new point cloud data to the queue.
        """
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            self.input_queue.put_nowait(points)
            self.num_pcs += 1
            # self.get_logger().info(f"Added new point cloud data to queue.")
            self.get_logger().info(f"Queue size: {self.num_pcs}")
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

    # def signal_handler(self, signum, frame):
    #     """Handles external termination signals (SIGINT, SIGTERM)."""
    #     self.get_logger().info(f"Received signal {signum}, shutting down...")
    #     self.destroy_node()

    # def destroy_node(self):
    #     """
    #     Cleanup function when the node is shut down.
    #     """
    #     self.get_logger().info("Shutting down point cloud subscriber...")
    #
    #     # Signal the processing loop to stop
    #     self.stop_event.set()
    #
    #     super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = PointClouds2Subscriber()
#
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Keyboard interrupt received, shutting down...")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()
