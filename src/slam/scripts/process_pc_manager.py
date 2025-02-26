import multiprocessing
import queue
import time

import rclpy.logging

from scripts.algorithm_enum import AlgorithmType
from scripts.data_transfer import DataTransfer
from scripts.process_point_clouds import ICPProcessor


class ProcessPointCloudsHandler:
    """
    Class to abstract multiprocessing for point cloud processing.
    """

    def __init__(self,
        algorithm: str,
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
        self.processor = None#self.set_algorithm(algorithm)

        self.processor_thread = multiprocessing.Process(target=self.process_loop)

    def process_loop(self) -> None:
        """
        Start the point cloud processing thread. Should be run in a separate process.
        """
        logger = rclpy.logging.get_logger("processing_manager")

        try:
            self.processor = self.set_algorithm(self.algorithm)

            while not self.stop_event.is_set():
                if self.reset_event.is_set():
                    logger.info("Resetting processor")
                    self.processor.reset()
                    continue
                if self.data_transfer.pixel_depth_map_queue.empty():
                    time.sleep(0.1)
                    continue
                self.processor.process()
                self.data_transfer.global_map_queue.put(self.processor.global_map)

        except KeyboardInterrupt:
            logger.info("Stopped by user")

        # self.logger.info("shutting down processing loop")



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
        self.processor_thread.join(timeout=3)

        if self.processor_thread.is_alive():
            self.processor_thread.terminate()  # Force terminate if it doesn't stop
            self.processor_thread.join()

    def set_algorithm(self, algorithm: str):
        """Set the processing algorithm safely using Enum."""
        algorithm = AlgorithmType[algorithm.upper()]
        if not isinstance(algorithm, AlgorithmType):
            raise ValueError(f"Invalid algorithm type: {algorithm}")

        self.algorithm = algorithm
        if self.algorithm == AlgorithmType.ICP:
            processor = ICPProcessor(
                config_path=self.config_path,
                data_transfer=self.data_transfer,
                reset_event=self.reset_event
            )
        elif self.algorithm == AlgorithmType.DEEP_LEARNING:
            processor = None
        else:
            raise ValueError(f"Unsupported algorithm: {algorithm}")

        print(
            f"Switched to algorithm: {self.algorithm.value}")  # Logging for debug
        return processor
