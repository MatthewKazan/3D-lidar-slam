import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String
import multiprocessing
import time
from collections import deque
from multiprocessing import Queue
from typing import Optional
from abc import ABC, abstractmethod

import numpy as np
import open3d as o3d
from sqlalchemy import create_engine, text


class ProcessPointClouds(ABC):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self,
        config_path: str,
        result_queue: Optional[Queue] = None,
        stop_event: Optional[multiprocessing.Event] = None,
        reset_event: Optional[multiprocessing.Event] = None):
        """
        Initialize the point cloud processor with settings from a YAML file.
        Should be run in a separate process since it's a long-running task.

        :param config_path: Path to the YAML configuration file.
        :param result_queue: Thread-safe queue to store the latest point cloud map.
        :param stop_event: Thread-safe event to stop this object from running.
        :param reset_event: Thread-safe event triggered on database reset.
        """

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

        self.point_cloud_map = o3d.geometry.PointCloud()
        self.point_clouds_in_map = 0

        self.previous_transformation = [np.identity(4)]

        self.start_time = None


    def load_config(self, config_path: str):
        """
        Loads configuration parameters from a YAML file.
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

        print(
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


class ICPProcessor(ProcessPointClouds):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self):
        """
        Initialize the point cloud processor. Should be run in a separate process
        since its a long-running task and slow.

        :param result_queue: thread safe queue to store the latest point cloud map
        :param stop_event: thread safe event to stop this object from running
        :param reset_event: thread safe event triggered on database reset
        """

        super().__init__()

        self.cur_scan_id = 0

        self.point_cloud_map = o3d.geometry.PointCloud()
        self.point_clouds_in_map = 0

        self.previous_transformation = [np.identity(4)]

