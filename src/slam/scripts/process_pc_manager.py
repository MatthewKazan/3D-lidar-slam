import multiprocessing
import time
import traceback

import numpy as np
import rclpy.logging
from rclpy.node import Node

from scripts.algorithm_enum import AlgorithmType, processor_constructor
from scripts.data_transfer import DataTransfer
from scripts.point_cloud_processors import ICPProcessor
from scripts.point_cloud_processors import DGRProcessor
from  custom_interfaces.srv import SetAlgorithm
from rclpy.service import SrvTypeResponse

from scripts.point_cloud_processors.pose_graph import \
    PoseGraphGTSAMICP
from scripts.state import state


class ProcessPointCloudsHandlerNode(Node):
    """
    A ROS 2 node that manages the point cloud processing thread.
    """

    def __init__(self,
        algorithm: str,
        data_transfer: DataTransfer,
    ):
        super().__init__('process_point_clouds_handler')
        self.algorithm = algorithm
        self.processor_handler = ProcessPointCloudsHandler(
            algorithm=algorithm,
            data_transfer=data_transfer,
        )
        self.data_transfer = data_transfer
        self.timer = self.create_timer(0.1, self.processor_handler.process_loop)

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

        self.processor_handler.process_loop()

    def set_algorithm(self, request, response) -> SrvTypeResponse:
        """
        Service callback to change the processing algorithm.
        """
        self.get_logger().info(f"Received request to set algorithm to {request.algorithm}")
        if request.algorithm not in AlgorithmType.__members__:
            response.success = False
            response.message = f"Unsupported algorithm: {request.algorithm}"
            self.get_logger().error(response.message)
            return response
        # self.algorithm.set(request.algorithm)
        self.processor_handler.set_algorithm(request.algorithm)

        response.success = True
        response.message = f"Algorithm set to {request.algorithm}"
        self.get_logger().debug(response.message)
        return response

    def reset(self):
        """
        Reset the processor handler and pose graph.
        """
        self.processor_handler.reset()

    def destroy_node(self):
        """
        Override the destroy_node method to stop the processor handler before destroying the node.
        """
        # self.processor_handler.stop()
        self.timer.destroy()
        super().destroy_node()



class ProcessPointCloudsHandler:
    """
    Class to abstract multiprocessing for point cloud processing.
    """

    def __init__(self,
        algorithm: str,
        data_transfer: DataTransfer,
    ):
        self.data_transfer = data_transfer

        self.algorithm = algorithm
        self.processor = None
        self.set_algorithm(self.algorithm)

        self.pose_graph = PoseGraphGTSAMICP(self.on_optimized_global_map)

    def process_loop(self) -> None:
        """
        Start the point cloud processing thread. Should be run in a separate process.
        """
        try:

            start_time = time.time()
            ### Do actual data processing ###
            scan_pc = self.processor.process()

            rclpy.logging.get_logger("processing_manager").debug(f"Processing took {time.time() - start_time:.3f} seconds")

            if scan_pc is None:
                return
            rclpy.logging.get_logger("processing_manager").info(f"registration took {time.time() - start_time:.3f} seconds")

            self.pose_graph.update_pose_graph(
                trans=self.processor.previous_transformation[-1],
                scan_pc=scan_pc,
                gen_descriptor=state.descriptor_fn,
            )

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

    def set_algorithm(self, algorithm: str):
        """Set the processing algorithm safely using Enum."""

        algorithm = AlgorithmType[algorithm.upper()]
        constructor = processor_constructor[algorithm]
        self.processor = constructor(
            data_transfer=self.data_transfer,
        )
        # if algorithm == AlgorithmType.ICP:
        #     processor = ICPProcessor(
        #         # config_path=self.config_path,
        #         data_transfer=self.data_transfer,
        #         # reset_event=self.reset_event
        #     )
        # elif algorithm == AlgorithmType.DGR:
        #     processor = DGRProcessor(
        #         # config_path=self.config_path,
        #         data_transfer=self.data_transfer,
        #         # reset_event=self.reset_event
        #     )
        # else:
        #     raise KeyError(f"Unsupported algorithm: {algorithm}")
        # self.processor = processor

        rclpy.logging.get_logger("processing_manager").info(
            f"Switched to algorithm: {algorithm}")

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
