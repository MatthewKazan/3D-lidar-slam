import multiprocessing
import time
import traceback

import numpy as np
import rclpy.logging
from rclpy.node import Node

from scripts.algorithm_enum import AlgorithmType, DescriptorType
from scripts.data_transfer import DataTransfer
from scripts.pointcloud_processors.pointcloud_registration import ICPProcessor, DGRProcessor
from custom_interfaces.srv import SetAlgorithm
from rclpy.service import SrvTypeResponse

from scripts.pointcloud_processors.pose_graph import \
    PoseGraphGTSAMICP
# from scripts.state import state

from scripts.pointcloud_processors.descriptor_generators.ndt_transformer import \
    NDTTransformer

from scripts.pointcloud_processors.descriptor_generators.scan_context import \
    ScanContext

from scripts.algorithm_constructors import processor_constructor

from scripts.config import SLAMConfig


class ProcessPointCloudsHandler:
    """
    Class to abstract multiprocessing for point cloud processing.
    """

    def __init__(self,
        config: SLAMConfig,
        data_transfer: DataTransfer,
    ):
        self.data_transfer = data_transfer

        self.cur_algorithm_type = None
        self.processor = None
        self.config = config
        self.set_algorithm(config.algorithm_type)
        self.cur_descriptor_type = None
        self.descriptor = None
        self.set_descriptor(config.descriptor_type)

        self.pose_graph = PoseGraphGTSAMICP(self.on_optimized_global_map, self.config)
        self.first_scan = True

    def process_loop(self) -> None:
        """
        Start the point cloud processing thread. Should be run in a separate process.
        """
        try:
            if self.first_scan:
                rclpy.logging.get_logger("processing_manager").info("Ready for point cloud processing")
                self.first_scan = False


            start_time = time.time()
            ### Do actual data processing ###
            scan_pc = self.processor.process()

            if scan_pc is None:
                return
            rclpy.logging.get_logger("processing_manager").debug(f"registration took {time.time() - start_time:.3f} seconds")

            # self.pose_graph.update_pose_graph(
            #     trans=self.processor.previous_transformation[-1],
            #     scan_pc=scan_pc,
            #     gen_descriptor=self.descriptor.generate_descriptor,
            # )

            if self.data_transfer.stop_event.is_set():
                raise KeyboardInterrupt("Stopping processing")

            # Transform the point cloud to the global map queue for publishing
            with self.data_transfer.global_map_lock:
                global_map = np.asarray(self.processor.global_map.points)
                self.data_transfer.global_map_queue.put(global_map)
            rclpy.logging.get_logger("processing_manager").info(f"Processing took {time.time() - start_time:.3f} seconds")

        except KeyboardInterrupt:
            rclpy.logging.get_logger("processing_manager").info("Stopped by user 1")
        except Exception as e:
            """
            Catch-all for any exceptions in the processing loop to avoid crashing the thread.
            """
            rclpy.logging.get_logger("processing_manager").error(
                f"Exception in process loop: {e}, {type(e)}")
            traceback.print_exc()
            raise

    def on_optimized_global_map(self, keyframes):
        """
        Callback for the pose graph optimizer to update the global map in the processor.
        """
        rclpy.logging.get_logger("processing_manager").debug(
            f"Called Optimizer")
        self.processor.rebuild_global_map(keyframes)

    def reset(self):
        """
        Reset the point cloud processor.
        """
        rclpy.logging.get_logger("processing_manager").info("Resetting processor")
        self.processor.reset()
        self.pose_graph.reset()

    def set_descriptor(self, descriptor_type) -> None:
        """
        Set the descriptor generator safely using Enum.

        :param descriptor_type: The descriptor type to set.
        """
        # descriptor_type = DescriptorType[descriptor_type.upper()]
        if descriptor_type == DescriptorType.NDT_T:
            self.descriptor = NDTTransformer(config=self.config)
        elif descriptor_type == DescriptorType.SCAN_CONTEXT:
            self.descriptor = ScanContext()

    def set_algorithm(self, algorithm: AlgorithmType):
        """Set the processing algorithm safely using Enum."""

        # algorithm = AlgorithmType[algorithm.upper()]

        constructor = processor_constructor[algorithm]
        self.processor = constructor(
            data_transfer=self.data_transfer,
            config=self.config,
        )
        self.cur_algorithm_type = algorithm

        rclpy.logging.get_logger("processing_manager").info(
            f"Switched to algorithm: {algorithm}")

