import multiprocessing
import time

import rclpy.logging
from rclpy.node import Node

from scripts.algorithm_enum import AlgorithmType
from scripts.data_transfer import DataTransfer
from scripts.point_cloud_processors import ICPProcessor
from scripts.point_cloud_processors import DGRProcessor
from  custom_interfaces.srv import SetAlgorithm
from rclpy.service import SrvTypeResponse


class ProcessPointCloudsHandlerNode(Node):
    """
    A ROS 2 node that manages the point cloud processing thread.
    """

    def __init__(self,
        algorithm: str,
        config_path: str,
        data_transfer: DataTransfer,
        stop_event: multiprocessing.Event,
        reset_event: multiprocessing.Event,
    ):
        super().__init__('process_point_clouds_handler')
        self.algorithm = multiprocessing.Manager().Value('str', algorithm)
        self.processor_handler = ProcessPointCloudsHandler(
            algorithm=self.algorithm,
            config_path=config_path,
            data_transfer=data_transfer,
            stop_event=stop_event,
            reset_event=reset_event,
        )
        self.processor_handler.start()

    def set_algorithm(self, request, response) -> SrvTypeResponse:
        """
        Service callback to change the processing algorithm.
        """
        self.get_logger().info(f"Received request to set algorithm to {request.algorithm}")
        self.algorithm.set(request.algorithm)

        response.success = True
        response.message = f"Algorithm set to {request.algorithm}"
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        """
        Override the destroy_node method to stop the processor handler before destroying the node.
        """
        self.processor_handler.stop()
        super().destroy_node()



class ProcessPointCloudsHandler:
    """
    Class to abstract multiprocessing for point cloud processing.
    """

    def __init__(self,
        algorithm: multiprocessing.Value,
        config_path: str,
        data_transfer: DataTransfer,
        stop_event: multiprocessing.Event,
        reset_event: multiprocessing.Event,
    ):

        self.data_transfer = data_transfer

        self.stop_event = stop_event
        self.reset_event = reset_event
        self.config_path = config_path

        self.algorithm = algorithm
        # Can't define self.processor here because it is unpicklable, breaks multiprocessing
        self.processor = None

        self.processor_thread = multiprocessing.Process(target=self.process_loop)

    def process_loop(self) -> None:
        """
        Start the point cloud processing thread. Should be run in a separate process.
        """
        try:
            self.processor = self.set_algorithm(self.algorithm.get())
            cur_algorithm = self.algorithm.get()

            while not self.stop_event.is_set():
                if self.algorithm.get() != cur_algorithm:
                    try:
                        self.processor = self.set_algorithm(self.algorithm.get())
                    except KeyError as e:
                        rclpy.logging.get_logger("processing_manager").error(f"Error setting algorithm: {e}, reverting to previous algorithm {cur_algorithm}")
                        self.algorithm.set(cur_algorithm)  # Revert to previous algorithm
                        continue
                    cur_algorithm = self.algorithm.get()
                if self.data_transfer.pixel_depth_map_queue.empty():
                    time.sleep(0.1)
                    continue
                start_time = time.time()
                self.processor.process()

                # Check here in case the reset_event was set during processing
                if self.reset_event.is_set():
                    rclpy.logging.get_logger("processing_manager").info("Resetting processor")
                    self.processor.reset()
                    continue

                with self.data_transfer.global_map_lock:
                    self.data_transfer.global_map_queue.put(self.processor.global_map)
                rclpy.logging.get_logger("processing_manager").info(f"Processing took {time.time() - start_time:.3f} seconds")

        except KeyboardInterrupt:
            rclpy.logging.get_logger("processing_manager").info("Stopped by user 1")

        # rclpy.logging.get_logger("processing_manager").info("shutting down processing loop")

    def start(self) -> None:
        """
        Start the point cloud processing thread.
        """
        self.processor_thread.start()

    def stop(self) -> None:
        """
        Stop the point cloud processing thread.
        """
        self.stop_event.set()
        self.processor_thread.join(timeout=15)

        if self.processor_thread.is_alive():
            self.processor_thread.terminate()  # Force terminate if it doesn't stop
            self.processor_thread.join()
        rclpy.logging.get_logger("processing_manager").debug("Processing thread stopped")

    def set_algorithm(self, algorithm: str):
        """Set the processing algorithm safely using Enum."""
        algorithm = AlgorithmType[algorithm.upper()]
        if algorithm == AlgorithmType.ICP:
            processor = ICPProcessor(
                config_path=self.config_path,
                data_transfer=self.data_transfer,
                reset_event=self.reset_event
            )
        elif algorithm == AlgorithmType.DGR:
            processor = DGRProcessor(
                config_path=self.config_path,
                data_transfer=self.data_transfer,
                reset_event=self.reset_event
            )
        else:
            raise KeyError(f"Unsupported algorithm: {algorithm}")

        rclpy.logging.get_logger("processing_manager").info(
            f"Switched to algorithm: {algorithm}")
        return processor


if __name__ == "__main__":
    pass