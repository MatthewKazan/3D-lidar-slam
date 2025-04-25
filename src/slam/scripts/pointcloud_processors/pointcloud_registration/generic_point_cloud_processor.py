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
from scripts.pointcloud_processors.pose_graph import PoseGraphGTSAMICP

from scripts.paths import CONFIG_PATH


class ProcessPointClouds(ABC):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self,
        data_transfer: DataTransfer,
        logger,
    ):
        """
        Initialize the point cloud processor with settings from a YAML file.

        :param config_path: Path to the YAML configuration file.
        :param data_transfer: Thread-safe data object to send and receive pcs.
        """
        self.logger = logger

        self.global_map = o3d.geometry.PointCloud()
        self.point_clouds_in_map = 0

        self.data_transfer = data_transfer

        self.previous_transformation = [np.identity(4)]

        self.start_time = None
        self.pcs_to_align_with = []

        self.logger.info(f"Using {self.__class__.__name__} for point cloud processing")

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

        self.logger.debug(
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
            return copy.deepcopy(point_cloud_3d)
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
        self.global_map = o3d.geometry.PointCloud()
        self.point_clouds_in_map = 0
        self.previous_transformation = [np.identity(4)]
        self.start_time = None
        time.sleep(1)

    def rebuild_global_map(self, keyframes) -> None:
        """
        Rebuild the global map from the pose graph keyframes.
        """
        self.global_map = o3d.geometry.PointCloud()
        self.point_clouds_in_map = 0
        # self.previous_transformation = [np.eye(4)]
        self.pcs_to_align_with = []

        for i, keyframe in enumerate(keyframes):
            T = np.array(keyframe['pose'].matrix())
            pc = copy.deepcopy(keyframe['point_cloud'])
            # pc = pc.transform(T)

            self.global_map += pc.voxel_down_sample(0.02)
            self.point_clouds_in_map += 1
            # self.previous_transformation.append(T)
            self.global_map = self.global_map

            if i > len(keyframes) - 10:
                self.pcs_to_align_with.append(keyframe['point_cloud'])


        # self.global_map = transformed_clouds
        self.logger.debug(f"Global map rebuilt from {len(keyframes)} keyframes")

    @abstractmethod
    def construct_global_map(self, point_cloud: o3d.geometry.PointCloud, voxel_size=0.02) -> o3d.geometry.PointCloud:
        """
        Construct the global map from the point cloud.
        Store the global map in the global_map instance variable.

        :param point_cloud: A numpy array of x,y,z points in meters
        :param voxel_size: The voxel size for downsampling the point clouds
        """
        pass


if __name__ == "__main__":
    pass