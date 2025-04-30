import multiprocessing
import queue

import rclpy.logging


class DataTransfer:
    """
    This class allows for data transfer between multiple threads since,
    ROS2 Nodes are not thread-safe and can't be pickled.
    """
    def __init__(self):
        """
        Initialize the data transfer object.
        """
        self.pixel_depth_map_lock = multiprocessing.Lock()
        self.pixel_depth_map_queue = multiprocessing.Queue()
        self.global_map_queue = multiprocessing.Queue(1)
        self.global_map_lock = multiprocessing.Lock()

        self.stop_event = multiprocessing.Event()

    def reset(self):
        """
        Reset the data transfer object.
        """
        with self.pixel_depth_map_lock:
            while True:
                try:
                    rclpy.logging.get_logger("data_transfer").info(
                        "Resetting pixel_depth_map_queue")
                    self.pixel_depth_map_queue.get_nowait()  # Non-blocking get
                except queue.Empty:
                    break  # Stop when the queue is empty

        with self.global_map_lock:
            try:
                rclpy.logging.get_logger("data_transfer").info(
                    "Resetting global_map_queue")
                self.global_map_queue.get_nowait()  # Non-blocking get
            except queue.Empty:
                return

    def queue_shutdown(self):
        """
        Shutdown the queues.
        """
        if self.pixel_depth_map_queue is None or self.global_map_queue is None:
            rclpy.logging.get_logger("data_transfer").info(
                "Queues already closed or not initialized")
            return
        self.reset()
        self.stop_event.set()
        self.pixel_depth_map_queue.close()
        self.global_map_queue.close()
        self.pixel_depth_map_queue.join_thread()
        self.global_map_queue.join_thread()
        self.pixel_depth_map_queue = None
        self.global_map_queue = None
        rclpy.logging.get_logger("data_transfer").info(
            "Queues closed and joined")
