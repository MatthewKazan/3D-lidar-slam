import copy
import multiprocessing
import queue
import time
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
import yaml
import open3d as o3d

from scripts.data_transfer import DataTransfer
from scripts.point_cloud_processors.pose_graph import PoseGraphGTSAMICP


class ProcessPointClouds(ABC):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self,
        config_path: str,
        # reset_event: multiprocessing.Event,
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
        self.config = None
        self.load_config(config_path)

        self.global_map = None
        self.point_clouds_in_map = 0

        self.data_transfer = data_transfer

        # with self.data_transfer.global_map_lock:
        #     if not self.data_transfer.global_map_queue.empty():
        #         self.global_map = self.data_transfer.global_map

        self.previous_transformation = [np.identity(4)]

        self.start_time = None
        self.pcs_to_align_with = []
        # self.reset_event = reset_event

        self.logger.info(f"Using {self.__class__.__name__} for point cloud processing")

    def load_config(self, config_path: str) -> None:
        """
        Loads configuration parameters of the lidar scanner from a YAML file.

        :param config_path: Path to the YAML configuration file.
        """
        with open(config_path, 'r') as file:
            self.config = yaml.safe_load(file)

        # Camera Parameters
        self.WIDTH = self.config['camera']['width']
        self.HEIGHT = self.config['camera']['height']
        self.ref_width = self.config['camera']['ref_width']
        self.ref_height = self.config['camera']['ref_height']
        self.fx = self.config['camera']['fx']
        self.fy = self.config['camera']['fy']
        self.cx = self.config['camera']['cx']
        self.cy = self.config['camera']['cy']

        # Scale intrinsic parameters to the new resolution
        scale_x = self.WIDTH / self.ref_width
        scale_y = self.HEIGHT / self.ref_height
        self.fx *= scale_x
        self.fy *= scale_y
        self.cx *= scale_x
        self.cy *= scale_y

        self.logger.info(
            f"Loaded camera parameters: WIDTH={self.WIDTH}, HEIGHT={self.HEIGHT}, fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")


    def project_pixel_to_3d(self, points: list) -> np.ndarray:
        """
        Convert pixel coordinates to 3D coordinates in meters using the camera intrinsics.

        :param points: X,Y pixel coordinate, z meters

        :return: X, Y, Z coordinates in meters
        """
        points_3d = np.zeros((len(points), 3), dtype=np.float32)
        for i, point in enumerate(points):
            x, y, z = point
            # x = (x - self.cx) * z / self.fx
            # y = (y - self.cy) * z / self.fy
            points_3d[i]=[x, y, z]

        return points_3d


    def process(self) -> Optional[o3d.geometry.PointCloud]:
        """
        Main fn to process point clouds each loop iteration.

        :return: The new point cloud transformed to align with the global map
        """
        with self.data_transfer.pixel_depth_map_lock:
            try:
                point_cloud_pixel = self.data_transfer.pixel_depth_map_queue.get_nowait()
                self.logger.debug(
                    f"{len(point_cloud_pixel)} points received from queue")

            except queue.Empty:
                # No new data to publish
                return None

        # Process the point cloud
        if point_cloud_pixel is None:
            return None

        self.logger.info(
            f"pcs processed so far: {self.point_clouds_in_map}")

        # point_cloud_3d = self.project_pixel_to_3d(point_cloud_pixel)
        points = np.zeros((len(point_cloud_pixel), 3), dtype=np.float32)
        for i, point in enumerate(point_cloud_pixel):
            x, y, z = point
            points[i] = [x, y, z]
        del point_cloud_pixel
        point_cloud_3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

        # If this is the first point cloud, set it as the global map
        if self.point_clouds_in_map == 0:
            self.point_clouds_in_map += 1
            self.global_map = point_cloud_3d
            self.pcs_to_align_with.append(point_cloud_3d)
            return point_cloud_3d
        # Transform the new point cloud into the global map frame and add to self.global_map
        global_oriented_pc = self.construct_global_map(point_cloud_3d)
        self.pcs_to_align_with.append(global_oriented_pc)
        if len(self.pcs_to_align_with) > 10:
            # Keep the last 10 point clouds for alignment to save memory
            self.pcs_to_align_with.pop(0)
        self.point_clouds_in_map += 1
        return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

    def reset(self) -> None:
        """
        Reset the point cloud processor.
        """
        self.logger.info("Resetting processor")
        self.global_map = None
        self.point_clouds_in_map = 0
        self.previous_transformation = [np.identity(4)]
        self.start_time = None
        # self.reset_event.clear()
        time.sleep(1)

    def rebuild_global_map(self, keyframes) -> None:
        """
        Rebuild the global map from the pose graph keyframes.
        """
        self.global_map = o3d.geometry.PointCloud()
        self.point_clouds_in_map = 0
        self.previous_transformation = []
        self.pcs_to_align_with = []

        for i, keyframe in enumerate(keyframes):
            T = np.array(keyframe['pose'].matrix())
            pc = keyframe['point_cloud']
            # pc = pc.transform(T)

            self.global_map += pc
            self.point_clouds_in_map += 1
            self.previous_transformation.append(T)

            if i > len(keyframes) - 10:
                self.pcs_to_align_with.append(pc)


        # self.global_map = transformed_clouds
        self.logger.info(f"Global map rebuilt from {len(keyframes)} keyframes")

    @abstractmethod
    def construct_global_map(self, points: np.array) -> o3d.geometry.PointCloud:
        """
        Construct the global map from the point cloud.
        Store the global map in the global_map instance variable.

        :param points: A numpy array of x,y,z points in meters
        """
        pass

    # @abstractmethod
    # def downsample_global_map(self) -> np.ndarray:
    #     """
    #     Function which downsamples the global map to reduce the number of points
    #     in the global map. Used to reduce number of points published to avoid overwhelming rviz
    #     without sacrificing local accuracy.
    #
    #     :return: numpy array of downsampled points
    #     """
    #     pass


# def transform_point_cloud(pc: np.ndarray, T: np.ndarray) -> np.ndarray:
#     """
#     Transform a point cloud using a 4x4 transformation matrix.
#
#     :param pc: The point cloud to transform
#     :param T: The 4x4 transformation matrix
#
#     :return: The transformed point cloud
#     """
#     # Add a row of [0, 0, 0, 1] to the point cloud to make it homogeneous.
#     pc_h = np.hstack((pc, np.ones((pc.shape[0], 1))))
#     # Transform the point cloud using the transformation matrix.
#     pc_transformed_h = np.dot(T, pc_h.T).T
#     # Remove the homogeneous coordinate and return the transformed point cloud.
#     return pc_transformed_h[:, :3]

if __name__ == "__main__":
    pass