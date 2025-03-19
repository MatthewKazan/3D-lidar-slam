import multiprocessing
import os
import sys

import numpy as np
import open3d as o3d
import rclpy.logging

from scripts.point_cloud_processors.generic_point_cloud_processor import ProcessPointClouds

from scripts.data_transfer import DataTransfer

from scripts.paths import PATH_TO_BUILD_DGR, PATH_TO_BUILD_MINK

# Set working directory to where DGR expects to be
# There must be a better way to do this
os.chdir(PATH_TO_BUILD_DGR)  # Change working directory
sys.path.append(PATH_TO_BUILD_DGR)  # Ensure the path is in Python's search
sys.path.append(PATH_TO_BUILD_MINK)  # Ensure the path is in Python's search

import libs.DeepGlobalRegistration.core.deep_global_registration as dgr
from libs.DeepGlobalRegistration.config import get_config

class DGRProcessor(ProcessPointClouds):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self,
        config_path: str,
        reset_event: multiprocessing.Event,
        data_transfer: DataTransfer,
    ):
        """
        Initialize the point cloud processor with settings from a YAML file.
        Should be run in a separate process since it's a long-running task.

        :param config_path: Path to the YAML configuration file.
        :param reset_event: Thread-safe event triggered on database reset.
        """
        super().__init__(
            config_path,
            reset_event,
            data_transfer,
            rclpy.logging.get_logger("DGR processor")
        )
        # Weirdness since DGR can't have other args in command line when it runs
        import sys

        if "--ros-args" in sys.argv:
            ros_args_index = sys.argv.index("--ros-args")
            sys.argv = sys.argv[:ros_args_index]  # Remove ROS arguments
        dgr_config = get_config()
        dgr_config.weights = self.config['dgr_config']['weights']

        self.dgr = dgr.DeepGlobalRegistration(dgr_config, device='cpu')

    def construct_global_map(self, points: np.array) -> None:
        """
        Align the new point cloud with the global map using ICP and add it to the map.

        :param points: The new point cloud to add to the global map

        :return: The new point cloud transformed to align with the global map
        """
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        # Can this be done quickly?
        # point_cloud, _ = point_cloud.remove_statistical_outlier(nb_neighbors=5, std_ratio=6)

        o3d_global_map = o3d.geometry.PointCloud()
        o3d_global_map.points = o3d.utility.Vector3dVector(self.global_map)

        dgr_result_transformation = self.dgr.register(point_cloud, o3d_global_map)

        point_cloud = point_cloud.transform(dgr_result_transformation)
        self.global_map = np.asarray((point_cloud + o3d_global_map).voxel_down_sample(0.025).points)

    def downsample_global_map(self) -> np.ndarray:
        """
        Downsample the global map to reduce the number of points. maybe unnecessary

        :return: The downsampled global map
        """
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(self.global_map)
        point_cloud = point_cloud.voxel_down_sample(0.05)
        self.global_map = np.asarray(point_cloud.points)
        return point_cloud.points