import multiprocessing
import threading
import time

import numpy as np
import rclpy.logging
from rclpy.node import Node

from scripts.algorithm_enum import AlgorithmType
from scripts.data_transfer import DataTransfer
from scripts.point_cloud_processors import ICPProcessor
from scripts.point_cloud_processors import DGRProcessor
from  custom_interfaces.srv import SetAlgorithm
from rclpy.service import SrvTypeResponse

from scripts.point_cloud_processors.pose_graph import \
    PoseGraphGTSAMICP, compute_scan_context_descriptor
import traceback


class ProcessPointCloudsHandlerNode(Node):
    """
    A ROS 2 node that manages the point cloud processing thread.
    """

    def __init__(self,
        algorithm: str,
        config_path: str,
        data_transfer: DataTransfer,
        # stop_event: multiprocessing.Event,
        # reset_event: multiprocessing.Event,
    ):
        super().__init__('process_point_clouds_handler')
        self.algorithm = multiprocessing.Manager().Value('str', algorithm)
        self.processor_handler = ProcessPointCloudsHandler(
            algorithm=algorithm,
            config_path=config_path,
            data_transfer=data_transfer,
            # stop_event=stop_event,
            # reset_event=reset_event,
        )
        self.timer = self.create_timer(0.1, self.processor_handler.process_loop)

        # self.processor_handler.start()

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
        self.get_logger().info(response.message)
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
        algorithm: str,#multiprocessing.Value,
        config_path: str,
        data_transfer: DataTransfer,
        # stop_event: multiprocessing.Event,
        # reset_event: multiprocessing.Event,
    ):

        self.data_transfer = data_transfer

        # self.stop_event = stop_event
        # self.reset_event = reset_event
        self.config_path = config_path

        self.algorithm = algorithm
        # Can't define self.processor here because it is unpicklable, breaks multiprocessing
        self.processor = None
        self.pose_graph = PoseGraphGTSAMICP(self.on_optimized_global_map)
        self.set_algorithm(self.algorithm)

        # self.processor_thread = threading.Thread(target=self.process_loop)


    def process_loop(self) -> None:
        """
        Start the point cloud processing thread. Should be run in a separate process.
        """
        try:
            ### Handle ROS2 Events ###
            # cur_algorithm = self.algorithm.get()

            # while not self.stop_event.is_set():
            # if self.algorithm.get() != cur_algorithm:
            #     try:
            #         del self.processor
            #         self.processor = self.set_algorithm(self.algorithm.get())
            #     except KeyError as e:
            #         rclpy.logging.get_logger("processing_manager").error(f"Error setting algorithm: {e}, reverting to previous algorithm {cur_algorithm}")
            #         self.algorithm.set(cur_algorithm)  # Revert to previous algorithm
            #         return
            #     cur_algorithm = self.algorithm.get()
            start_time = time.time()

            # if self.data_transfer.pixel_depth_map_queue.empty():
            #     time.sleep(0.1)
            #     rclpy.logging.get_logger("processing_manager").info(
            #         f"Processing took {time.time() - start_time:.3f} seconds")
            #
            #     return

            ### Do actual data processing ###
            scan_pc = self.processor.process()

            rclpy.logging.get_logger("processing_manager").debug(f"Processing took {time.time() - start_time:.3f} seconds")

            if scan_pc is None:
                return
            rclpy.logging.get_logger("processing_manager").debug(f"registration took {time.time() - start_time:.3f} seconds")

            self.pose_graph.update_pose_graph(
                trans=self.processor.previous_transformation[-1],
                scan_pc=scan_pc,
                gen_descriptor=compute_scan_context_descriptor
            )
            rclpy.logging.get_logger("processing_manager").info(f"Processing took {time.time() - start_time:.3f} seconds")



            ### Handle more ROS2 Events ###
            rclpy.logging.get_logger("processing_manager").info(f"Sending global map to publisher")

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
        # rclpy.logging.get_logger("processing_manager").info("shutting down processing loop")

    def start(self) -> None:
        """
        Start the point cloud processing thread.
        """
        # self.processor_thread.start()
        # self.pose_graph.start()
        pass

    # def stop(self) -> None:
    #     """
    #     Stop the point cloud processing thread.
    #     """
    #     self.stop_event.set()
    #     self.pose_graph.stop()
        # self.processor_thread.join(timeout=15)

        rclpy.logging.get_logger("processing_manager").debug("Processing thread stopped")

    def set_algorithm(self, algorithm: str):
        """Set the processing algorithm safely using Enum."""

        algorithm = AlgorithmType[algorithm.upper()]
        if algorithm == AlgorithmType.ICP:
            processor = ICPProcessor(
                config_path=self.config_path,
                data_transfer=self.data_transfer,
                # reset_event=self.reset_event
            )
        elif algorithm == AlgorithmType.DGR:
            processor = DGRProcessor(
                config_path=self.config_path,
                data_transfer=self.data_transfer,
                # reset_event=self.reset_event
            )
        else:
            raise KeyError(f"Unsupported algorithm: {algorithm}")
        self.processor = processor

        rclpy.logging.get_logger("processing_manager").info(
            f"Switched to algorithm: {algorithm}")

    def on_optimized_global_map(self, keyframes):
        """
        Callback for the pose graph optimizer to update the global map in the processor.
        """
        rclpy.logging.get_logger("processing_manager").info(
            f"Called Optimizer")
        self.processor.rebuild_global_map(keyframes)

    def reset(self):
        """
        Reset the point cloud processor.
        """
        rclpy.logging.get_logger("processing_manager").info("Resetting processor")
        self.processor.reset()
        self.pose_graph.reset()


if __name__ == "__main__":
    pass