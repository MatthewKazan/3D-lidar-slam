import multiprocessing
import queue


class DataTransfer:
    """
    This class allows for data transfer between multiple threads since,
    ROS2 Nodes are not thread-safe and can't be pickled.
    """
    def __init__(self):
        """
        Initialize the data transfer object.
        """
        self.pixel_depth_map_queue = multiprocessing.Queue()
        self.global_map_queue = multiprocessing.Queue(1)

    def reset(self):
        """
        Reset the data transfer object.
        """
        while not self.pixel_depth_map_queue.empty():
            self.pixel_depth_map_queue.get()
        if not self.global_map_queue.empty():
            self.global_map_queue.get()

