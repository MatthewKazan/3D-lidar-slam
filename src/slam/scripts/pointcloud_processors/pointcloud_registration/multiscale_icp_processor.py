import open3d as o3d
import numpy as np
import rclpy.logging
from scripts.pointcloud_processors.pointcloud_registration import \
    ICPProcessor

from scripts.data_transfer import DataTransfer

from scripts.pointcloud_processors.utils.open3d_utils import \
    compute_multiscale_icp_transformation


class MultiscaleICPProcessor(ICPProcessor):
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

        """
        super().__init__(
            data_transfer,
            rclpy.logging.get_logger("multiscale icp processor")
        )

    def align_point_clouds_with_icp(self, source_cloud: o3d.geometry.PointCloud, target_cloud: o3d.geometry.PointCloud,
        voxel_size: float =0.02) -> np.ndarray:
        """
        Align two point clouds using coarse to fine, point-to-plane ICP

        :param source_cloud: The point cloud to align
        :param target_cloud: The global map to align to
        :param voxel_size: The voxel size for downsampling the point clouds

        :return: The 4x4 transformation matrix to align the new point cloud with the global map
        """

        icp_result = compute_multiscale_icp_transformation(
            source_cloud=source_cloud,
            target_cloud=target_cloud,
            t_init=self.previous_transformation[-1],
            voxel_size=voxel_size,
            logger=self.logger,
            max_iterations=100
        )
        self.previous_transformation.append(icp_result.transformation)
        return icp_result.transformation