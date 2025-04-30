import os
import queue
import time

import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy.logging
from rclpy.service import SrvTypeResponse
from sensor_msgs.msg import PointCloud2

from scripts.data_transfer import DataTransfer
from scripts.paths import PATH_TO_ROSBAGS, generate_unique_bag_name

from slam.mixins.generic_handler_mixin import GenericHandlerMixin

from slam.mixins.config_handler import ConfigHandlerMixin


class PointClouds2Subscriber(GenericHandlerMixin, Node):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters, and stores it in a global map.
    """

    def __init__(self, data_transfer: DataTransfer, config):
        """
        Initialize the point cloud processor. Should be run in a separate process
        since it's a long-running task and slow.
        """
        super().__init__(node_name="pointclouds_subscriber")
        self.data_transfer = data_transfer
        self.config = config

        self.__init_generic_handler__()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            # durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,

        )

        # ROS 2 Subscriptions
        self.pc_subscription = self.create_subscription(
            PointCloud2, '/input_pointcloud', self.listener_callback, qos_profile,
        )


        self.get_logger().info('PointCloud processor has been started.')


        # Input saving stuff
        self.bag_dir_path = str(os.path.join(PATH_TO_ROSBAGS, 'input_bags'))
        self.input_writer = None
        self.should_save_inputs = False

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

            points = pc2.read_points(msg, field_names=("x", "y", "z"),
                                     skip_nans=True)
            self.data_transfer.pixel_depth_map_queue.put_nowait(points)
            if self.config.is_saving_inputs:
                if self.input_writer is None:
                    self.setup_input_rosbags()
                self.save_point_cloud(self.input_writer, points,
                                      "/input_pointcloud")
            elif self.input_writer is not None:
                del self.input_writer
                self.input_writer = None
                self.get_logger().info("Closed input bag writer.")


            self.num_pcs += 1
            self.get_logger().debug(f"Added {len(points)} new points to queue.")
            if self.num_pcs % 20 == 0:
                self.get_logger().info(f"Queue size: {self.num_pcs}")

        except queue.Full:
            self.get_logger().warn("PointCloud queue is full! Dropping frame.")

    def reset(self, _):
        """
        Resets the subscriber node
        """
        if self.input_writer is not None:
            self.input_writer.close()
        self.num_pcs = 0
        self.input_writer = None
        self.should_save_inputs = False
        self.get_logger().info("Resetting subscriber node...")
        # self.pc_subscription.destroy()

    def setup_input_rosbags(self):
        """
        Resets the publisher node
        """
        input_bag_path = os.path.join(self.bag_dir_path, generate_unique_bag_name(bag_prefix="inputs"))
        self.input_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=input_bag_path,
                                                    storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")

        self.input_writer.open(storage_options, converter_options)
        self.input_writer.create_topic(
            rosbag2_py.TopicMetadata(
                name='/input_pointcloud',
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr"
            )
        )

    def save_inputs_callback(self, request, response) -> SrvTypeResponse:
        """
        Toggles input saving.

        :param request: The service request
        :param response: The service response

        :return: The service response
        """

        self.should_save_inputs = not self.should_save_inputs
        if self.should_save_inputs:
            self.setup_input_rosbags()
        else:
            if self.input_writer is not None:
                del self.input_writer
                self.input_writer = None
            self.get_logger().info("Closed input bag writer.")

        self.get_logger().info(f"Toggled saving inputs to {self.should_save_inputs}.")

        response.success = True
        response.message = f"Toggled saving inputs to {self.should_save_inputs}."
        return response

    def destroy_node(self):
        """
        Override the destroy_node method to stop the processor handler before destroying the node.
        """
        self.get_logger().info("Destroying PointClouds2Subscriber node...")
        if self.input_writer is not None:
            del self.input_writer
        super().destroy_node()



def run_subscriber_process(data_transfer, config):
    """
    This function will run in a separate process.
    It initializes rclpy, creates the subscriber node, and spins an executor.
    """
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = ""
    os.environ["RMW_FASTRTPS_USE_UDP"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    import rclpy
    rclpy.init(args=[
        '--ros-args',
        '-r', '__node:=pointclouds_subscriber',
    ])  # Initialize ROS in this process
    # Create subscriber node using the shared data_transfer object.
    subscriber_node = PointClouds2Subscriber(data_transfer, config)


    node = subscriber_node
    executor = MultiThreadedExecutor(2)
    time.sleep(1)
    node.get_logger().info(f"Adding {node.get_name()} to executor")
    node.get_logger().info(f"is saving inputs: {config.is_saving_inputs}")

    executor.add_node(node)
    try:
        executor.spin()  # Spin until shutdown
    except KeyboardInterrupt:
        pass
    finally:
        data_transfer.queue_shutdown()
        data_transfer.stop_event.set()
        executor.shutdown()
        node.destroy_node()
        rclpy.logging.get_logger("subscriber_process").info(
            f"Ended node {type(node)}")
        if rclpy.ok():
            rclpy.logging.get_logger("subscriber_process").info(
                "Shut down subscriber process...")
            rclpy.shutdown()


