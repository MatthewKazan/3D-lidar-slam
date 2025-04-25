import os
import sys

import open3d as o3d
import rclpy.logging

from scripts.pointcloud_processors.pointcloud_registration.generic_point_cloud_processor import ProcessPointClouds
from scripts.data_transfer import DataTransfer
from scripts.paths import PATH_TO_BUILD_DGR, PATH_TO_BUILD_MINK
from scripts.state import state

from scripts.pointcloud_processors.utils.open3d_utils import \
    compute_icp_transformation, o3d_from_np_point_cloud, np_from_o3d_point_cloud

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
        data_transfer: DataTransfer,
    ):
        """
        Initialize the point cloud processor with settings from a YAML file.
        Should be run in a separate process since it's a long-running task.

        :param data_transfer: Thread-safe data object to send and receive pcs.
        """
        super().__init__(
            data_transfer,
            rclpy.logging.get_logger("DGR processor")
        )
        # Weirdness since DGR can't have other args in command line when it runs
        import sys

        if "--ros-args" in sys.argv:
            ros_args_index = sys.argv.index("--ros-args")
            sys.argv = sys.argv[:ros_args_index]  # Remove ROS arguments
        self.dgr_config = get_config()
        self.dgr_config.weights = state.config['dgr_config']['weights']
        self.pcs_to_align_with = []

        self.dgr = dgr.DeepGlobalRegistration(self.dgr_config, device='cpu')
        self.max_points = 120000

    def construct_global_map(self, point_cloud: o3d.geometry.PointCloud, voxel_size=.02) -> None:
        """
        Align the new point cloud with the global map using ICP and add it to the map.

        :param point_cloud: The new point cloud to add to the global map

        :return: The new point cloud transformed to align with the global map
        """

        o3d_global_map = o3d.geometry.PointCloud()
        points = 0
        # DGR is a little whiny about the number of points it wants to align
        for i in range(len(self.pcs_to_align_with), 0, -1):
            pc = self.pcs_to_align_with[i - 1]
            if points < self.max_points:
                o3d_global_map += pc
                points += len(pc.points)

        # point_cloud = point_cloud.voxel_down_sample(voxel_size)
        # DGR just hangs seemingly indefinitely sometimes
        dgr_result_transformation = self.dgr.register(point_cloud, o3d_global_map)

        self.previous_transformation.append(dgr_result_transformation)
        point_cloud = point_cloud.transform(dgr_result_transformation)
        self.global_map += point_cloud

        return point_cloud

    # def downsample_global_map(self) -> np.ndarray:
        # """
        # Downsample the global map to reduce the number of points. maybe unnecessary
        #
        # :return: The downsampled global map
        # """
        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(self.global_map)
        # point_cloud = point_cloud.voxel_down_sample(0.05)
        # self.global_map = np.asarray(point_cloud.points)
        # return point_cloud.points