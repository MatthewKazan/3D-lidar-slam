import dataclasses
import multiprocessing
import os
import traceback
from collections import deque
from enum import Enum

import rclpy.logging
from rcl_interfaces.msg import SetParametersResult, ParameterEvent, \
    ParameterType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from scripts.algorithm_enum import AlgorithmType, DescriptorType
from scripts.data_transfer import DataTransfer
from scripts.pointcloud_processors.pointcloud_registration import ICPProcessor, DGRProcessor
from custom_interfaces.srv import SetAlgorithm
from rclpy.service import SrvTypeResponse

from scripts.pointcloud_processors.pose_graph import \
    PoseGraphGTSAMICP
from scripts.state import state

from scripts.pointcloud_processors.descriptor_generators.ndt_transformer import \
    NDTTransformer

from scripts.pointcloud_processors.descriptor_generators.scan_context import \
    ScanContext

from scripts.algorithm_constructors import processor_constructor

from scripts.process_pc_manager import ProcessPointCloudsHandler

from scripts.config import SLAMConfig

from slam.mixins.subscriber_mixin import PointCloudSubscriberMixin
from slam.mixins.publisher_mixin import PointCloudPublisherMixin
from slam.mixins.config_handler import ConfigHandlerMixin

from slam.mixins.service_mixin import (
    SimpleServiceMixin, ServiceMapping,
    GetAlgorithmsList, get_algorithms_list_callback,
    SetAlgorithmServiceMapping
)
from std_srvs.srv import Trigger


class PointCloudSLAMNode(
    PointCloudSubscriberMixin,
    PointCloudPublisherMixin,
    ConfigHandlerMixin,
    SimpleServiceMixin,
    Node
):
    """
    A ROS 2 node that manages the point cloud processing thread.
    """

    def __init__(self,
        # algorithm: str,
        # data_transfer: DataTransfer,
    ):
        super().__init__('process_point_clouds_handler')
        # self.algorithm = algorithm
        # self.set_up_config_handler()
        self.data_transfer = DataTransfer()
        self.__init_pointclouds_subscriber__()
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
        # 1) Create a separate callback group
        self._param_cbg = ReentrantCallbackGroup()

        # 2) Subscribe to parameter events in that group
        self.create_subscription(
            ParameterEvent,
            '/parameter_events',  # global events topic
            self._on_params_changed,
            10,
            callback_group=self._param_cbg)


        self.processor_handler = ProcessPointCloudsHandler(
            config=self.config,
            data_transfer=self.data_transfer,
        )
        self.timer = self.create_timer(0.1, self.send_processing_request)

        # self.processor_handler.start()
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

    def reset(self):
        """
        Reset the processor handler and pose graph.
        """
        self.processor_handler.reset()

    def _on_params_changed(self, params ):
        # build a result, default to OK
        successful = True
        reason = ""

        for p in params.changed_parameters:
            name, pv = p.name, p.value
            # Unpack into a plain Python value
            if pv.type == ParameterType.PARAMETER_BOOL:
                value = pv.bool_value
            elif pv.type == ParameterType.PARAMETER_INTEGER:
                value = pv.integer_value
            elif pv.type == ParameterType.PARAMETER_DOUBLE:
                value = pv.double_value
            elif pv.type == ParameterType.PARAMETER_STRING:
                value = pv.string_value
            elif pv.type == ParameterType.PARAMETER_BYTE_ARRAY:
                value = list(pv.byte_array_value)
            elif pv.type == ParameterType.PARAMETER_BOOL_ARRAY:
                value = list(pv.bool_array_value)
            elif pv.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                value = list(pv.integer_array_value)
            elif pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                value = list(pv.double_array_value)
            elif pv.type == ParameterType.PARAMETER_STRING_ARRAY:
                value = list(pv.string_array_value)
            else:
                self.get_logger().warn(
                    f"Unknown parameter type {pv.type} for '{name}'")
                continue

            # 1) find the field in SLAMConfig
            field = next(
                f for f in dataclasses.fields(SLAMConfig) if f.name == name)

            # 2) convert enums if needed
            if isinstance(field.type, type) and issubclass(field.type, Enum):
                try:
                    converted = field.type(value.upper())
                except ValueError:
                    successful = False
                    reason = f"Invalid {name}={value}"
                    break
            else:
                converted = value

            # 3) setattr on your config instance
            setattr(self.config, name, converted)
            self.get_logger().info(f"Updated config.{name} = {converted}")

            if name == "algorithm_type": self.processor_handler.set_algorithm(converted)
            if name == "descriptor_type": self.processor_handler.set_descriptor(converted)

        return SetParametersResult(successful=successful, reason=reason)

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
    node = PointCloudSLAMNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"Executor crashed: {e}")
        traceback.print_exc()
    finally:
        node.data_transfer.stop_event.set()


        executor.shutdown()
        node.destroy_node()
        rclpy.logging.get_logger("processing_manager").info(
            f"Ended node {type(node)}")

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()