import ctypes
import multiprocessing
import os
import traceback

import numpy as np
import rclpy.logging

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from scripts.algorithm_enum import AlgorithmType, DescriptorType
from scripts.data_transfer import DataTransfer
from scripts.pointcloud_processors.pointcloud_registration import ICPProcessor, DGRProcessor

from scripts.pointcloud_processors.pose_graph import \
    PoseGraphGTSAMICP

from scripts.pointcloud_processors.descriptor_generators.ndt_transformer import \
    NDTTransformer

from scripts.pointcloud_processors.descriptor_generators.scan_context import \
    ScanContext

from scripts.algorithm_constructors import processor_constructor

from scripts.process_pc_manager import ProcessPointCloudsHandler

from scripts.config import SLAMConfig, dataclass_to_namespace

from slam.mixins.publisher_mixin import PointCloudPublisherMixin
from slam.mixins.config_handler import ConfigHandlerMixin

from slam.mixins.service_mixin import (
    SimpleServiceMixin, ServiceMapping,
    GetAlgorithmsList, get_algorithms_list_callback
)
from std_srvs.srv import Trigger

from slam.input_data_handler import run_subscriber_process


class PointCloudSLAMNode(
    PointCloudPublisherMixin,
    ConfigHandlerMixin,
    SimpleServiceMixin,
    Node
):
    """
    A ROS 2 node that manages the point cloud processing thread.
    """

    def __init__(self,
        config: SLAMConfig,
        data_transfer: DataTransfer,
    ):
        super().__init__('process_point_clouds_handler')
        self.config = config
        self.data_transfer = data_transfer
        self.__init_pointclouds_publisher__()
        self.__init_config_handler__()
        self.__init_simple_service_mixin__(
            services=[
                ServiceMapping("/save_global_map", Trigger,
                               self.save_map_callback),
                ServiceMapping("/get_algorithms_list", GetAlgorithmsList,
                               get_algorithms_list_callback),
            ]
        )


        self.processor_handler = ProcessPointCloudsHandler(
            config=self.config,
            data_transfer=self.data_transfer,
        )
        timer_cbg = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.send_processing_request, callback_group=timer_cbg)

    def send_processing_request(self):
        """
        Send a processing request to the processor handler.
        """
        if not self.data_transfer.stop_event.is_set():
            try:
                self.processor_handler.process_loop()
            except Exception as e:
                rclpy.logging.get_logger("processing_manager").error(
                    f"Exception in PointCloudHandler: {type(e)}: {e}")
                traceback.print_exc()
                self.timer.cancel()

    def handle_param_specifics(self, event):
        """
        Handle parameter changes. This function is called when parameters are changed.
        """
        for p in event.changed_parameters:
            name = p.name
            rclpy.logging.get_logger("processing_manager").info(
                f"Parameter changed: {name}")
            if name == "algorithm_type":
                try:
                    self.processor_handler.set_algorithm(self.config.algorithm_type)
                except ValueError:
                    continue
                self.reset()
            elif name == "descriptor_type":
                try:
                    self.processor_handler.set_descriptor(self.config.descriptor_type)
                except ValueError:
                    continue
                self.reset()

    def reset(self, _=None):
        """
        Reset the processor handler and pose graph.
        """
        self.processor_handler.reset()
        self.data_transfer.reset()
        self.publish_point_cloud(np.array([[0,0,0]]))



    def destroy_node(self):
        """
        Override the destroy_node method to stop the processor handler before destroying the node.
        """
        # self.processor_handler.stop()
        self.timer.destroy()
        super().destroy_node()



def main():
    # No idea if the following lines are necessary but
    # websockets has been finicky so im not touching it
    multiprocessing.set_start_method("spawn", force=True)  # Fix multiprocessing issues on MacOS
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = ""
    os.environ["RMW_FASTRTPS_USE_UDP"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    # End weirdness
    rclpy.init()
    config = SLAMConfig()
    shared_config = dataclass_to_namespace(config)
    data_transfer = DataTransfer()
    node = PointCloudSLAMNode(config=shared_config, data_transfer=data_transfer)
    executor = rclpy.executors.MultiThreadedExecutor(8)
    executor.add_node(node)

    subscriber_proc = multiprocessing.Process(
        target=run_subscriber_process, args=(data_transfer,shared_config,), daemon=True
    )
    subscriber_proc.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"Executor crashed: {e}")
        traceback.print_exc()
    finally:
        data_transfer.stop_event.set()
        data_transfer.queue_shutdown()
        subscriber_proc.join(5)

        executor.shutdown()
        node.destroy_node()
        rclpy.logging.get_logger("processing_manager").info(
            f"Ended node {type(node)}")

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()