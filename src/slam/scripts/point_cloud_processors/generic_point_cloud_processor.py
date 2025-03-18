import multiprocessing
import time
from abc import ABC, abstractmethod

import numpy as np
import yaml

from scripts.data_transfer import DataTransfer


class ProcessPointClouds(ABC):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self,
        config_path: str,
        reset_event: multiprocessing.Event,
        data_transfer: DataTransfer,
        logger=None,
    ):
        """
        Initialize the point cloud processor with settings from a YAML file.
        Should be run in a separate process since it's a long-running task.

        :param config_path: Path to the YAML configuration file.
        :param reset_event: Thread-safe event triggered on database reset.
        """
        self.logger = logger

        # Declare instance variables with default values
        self.WIDTH = None
        self.HEIGHT = None
        self.ref_width = None
        self.ref_height = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Load YAML Configuration
        self.load_config(config_path)

        self.global_map = None
        self.point_clouds_in_map = 0

        self.data_transfer = data_transfer

        with self.data_transfer.global_map_lock:
            if not self.data_transfer.global_map_queue.empty():
                self.global_map = self.data_transfer.global_map

        self.previous_transformation = [np.identity(4)]

        self.start_time = None
        self.reset_event = reset_event


        self.logger.info(f"Using {self.__class__.__name__} for point cloud processing")

    def load_config(self, config_path: str) -> None:
        """
        Loads configuration parameters of the lidar scanner from a YAML file.

        :param config_path: Path to the YAML configuration file.
        """
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # Camera Parameters
        self.WIDTH = config['camera']['width']
        self.HEIGHT = config['camera']['height']
        self.ref_width = config['camera']['ref_width']
        self.ref_height = config['camera']['ref_height']
        self.fx = config['camera']['fx']
        self.fy = config['camera']['fy']
        self.cx = config['camera']['cx']
        self.cy = config['camera']['cy']

        # Scale intrinsic parameters to the new resolution
        scale_x = self.WIDTH / self.ref_width
        scale_y = self.HEIGHT / self.ref_height
        self.fx *= scale_x
        self.fy *= scale_y
        self.cx *= scale_x
        self.cy *= scale_y

        self.logger.info(
            f"Loaded camera parameters: WIDTH={self.WIDTH}, HEIGHT={self.HEIGHT}, fx={self.fx}, fy={self.fy}")


    def project_pixel_to_3d(self, points: list) -> np.ndarray:
        """
        Convert pixel coordinates to 3D coordinates in meters using the camera intrinsics.

        :param points: X,Y pixel coordinate, z meters

        :return: X, Y, Z coordinates in meters
        """
        points_3d = np.zeros((len(points), 3), dtype=np.float32)
        for i, point in enumerate(points):
            x, y, z = point
            x = (x - self.cx) * z / self.fx
            y = (y - self.cy) * z / self.fy
            points_3d[i]=[x, y, z]

        return points_3d


    def process(self) -> None:
        """
        Main fn to process point clouds each loop iteration.
        """
        point_cloud_pixel = self.data_transfer.pixel_depth_map_queue.get()

        self.logger.debug(f"{len(point_cloud_pixel)} points received from queue")
        # Process the point cloud
        if point_cloud_pixel is not None:
            self.logger.info(
                f"pcs processed so far: {self.point_clouds_in_map}")

            point_cloud_3d = self.project_pixel_to_3d(point_cloud_pixel)
            del point_cloud_pixel
            self.construct_global_map(point_cloud_3d)
            self.point_clouds_in_map += 1

    def reset(self) -> None:
        """
        Reset the point cloud processor.
        """
        self.logger.info("Resetting processor")
        self.global_map = None
        self.point_clouds_in_map = 0
        self.previous_transformation = [np.identity(4)]
        self.start_time = None
        self.reset_event.clear()
        time.sleep(1)

    @abstractmethod
    def construct_global_map(self, points: np.array) -> None:
        """
        Construct the global map from the point cloud.
        Store the global map in the global_map instance variable.

        :param points: A numpy array of x,y,z points in meters
        """
        pass

    @abstractmethod
    def downsample_global_map(self) -> np.ndarray:
        """
        Function which downsamples the global map to reduce the number of points
        in the global map. Used to reduce number of points published to avoid overwhelming rviz
        without sacrificing local accuracy.

        :return: numpy array of downsampled points
        """
        pass


if __name__ == "__main__":
    pass