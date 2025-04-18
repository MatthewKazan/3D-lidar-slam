"""
This module contains classes process and store point clouds. Gets raw point
cloud data from the ros2, converts it from pixels to meters then does some
algorithm to store it in a global map. All algorithms are subclasses of the
ProcessPointClouds class. The ICPProcessor class aligns the new point cloud with
"""
import multiprocessing
import queue
import time

import numpy as np
import open3d as o3d
import rclpy.logging
from scipy.spatial import cKDTree

from scripts.data_transfer import DataTransfer
from scripts.point_cloud_processors.generic_point_cloud_processor import ProcessPointClouds
from scripts.point_cloud_processors.utils.open3d_utils import \
    compute_icp_transformation


# from scripts.point_cloud_processors.generic_point_cloud_processor impor

class ICPProcessor(ProcessPointClouds):
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

        :param config_path: Path to the YAML configuration file.
        """
        super().__init__(
            data_transfer,
            rclpy.logging.get_logger("icp processor")
        )
        self.pcs_to_align_with = []
        self.prev_downsample_freq = 10
        self.downsample_freq = 10

    def construct_global_map(self, point_cloud: o3d.geometry.PointCloud, voxel_size=0.02) -> o3d.geometry.PointCloud:
        """
        Align the new point cloud with the global map using ICP and add it to the map.

        :param point_cloud: The new point cloud to add to the global map
        :param voxel_size: The voxel size for downsampling the point clouds

        :return: The new point cloud transformed to align with the global map
        """

        o3d_global_map = o3d.geometry.PointCloud()
        for pc in self.pcs_to_align_with:
            o3d_global_map += pc

        # point_cloud = point_cloud.voxel_down_sample(voxel_size)
        # Perform ICP alignment
        t = time.time()
        icp_result_transformation = self.align_point_clouds_with_icp(
            point_cloud, o3d_global_map, voxel_size)
        self.logger.debug(f"ICP took {time.time() - t:.3f} seconds")
        if icp_result_transformation is None:
            raise ValueError("ICP failed to find a transformation")

        point_cloud = point_cloud.voxel_down_sample(voxel_size).transform(icp_result_transformation)
        self.global_map += point_cloud
        # self.outlier_removal()

        return point_cloud

    def align_point_clouds_with_icp(self, source_cloud, target_cloud,
        voxel_size=0.02) -> np.ndarray:
        """
        Align two point clouds using RANSAC and then ICP.

        :param source_cloud: The new point cloud to align
        :param target_cloud: The global map to align the new point cloud with
        :param voxel_size: The voxel size for downsampling the point clouds

        :return: The 4x4 transformation matrix to align the new point cloud with the global map
        """

        # print("time to get to align_point_clouds_with_icp: ", time.time() - self.start_time)
        # Downsample the clouds
        icp_transformation = compute_icp_transformation(
            source_cloud=source_cloud,
            target_cloud=target_cloud,
            t_init=self.previous_transformation[-1],
            voxel_size=voxel_size,
        )
        if self.data_transfer.stop_event.is_set():
            raise KeyboardInterrupt("Stopping ICP processing")

        # print("ICP Refined Transformation:")
        # print(result_icp.transformation)
        # self.logger.debug(f"Fitness: {result_icp.fitness}, RMSE: {result_icp.inlier_rmse}")
        self.previous_transformation.append(icp_transformation)
        return icp_transformation

    def outlier_removal(self) -> None:
        """
        Do some basic point cloud processing on the global map every few point
        clouds, remove outliers, downsample, etc.
        """
        # Pulled all of these numbers out of nowhere
        if self.point_clouds_in_map % self.downsample_freq == 0 or self.data_transfer.pixel_depth_map_queue.empty():
            self.logger.info(
                "downsampling and outlier removal on global map")
            self.logger.debug(
                "started downsampling and outlier removal on global map")

            self.global_map = self.global_map.voxel_down_sample(0.001)

            # if self.point_clouds_in_map % 40 == 0:
            #     point_cloud_map, _ = point_cloud_map.remove_statistical_outlier(
            #         nb_neighbors=80, std_ratio=2)
            #     # This is slow but seems to make a difference
            #     point_cloud_map, _ = point_cloud_map.remove_radius_outlier(
            #         nb_points=8, radius=0.023)
            #
            #     self.global_map = np.asarray(point_cloud_map.points)
            #     self.logger.info(
            #         "stopped downsampling and outlier removal on global map")
            #
            #     return

            self.global_map, _ = self.global_map.remove_statistical_outlier(
                nb_neighbors=45, std_ratio=2.6)

            self.logger.debug(
                "stopped downsampling and outlier removal on global map")

    def downsample_global_map(self) -> np.ndarray:
        """
        Downsample the global map to reduce the number of points. maybe unnecessary

        :return: The downsampled global map
        """
        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(self.global_map)
        # point_cloud = point_cloud.voxel_down_sample(0.2)
        # self.global_map = np.asarray(point_cloud.points)
        return self.global_map

    def rebuild_global_map(self, keyframes) -> None:
        super().rebuild_global_map(keyframes)
        self.outlier_removal()
        # self.prev_downsample_freq = 10
        # self.downsample_freq = 10

    def reset(self) -> None:
        """
        Reset the point cloud processor.
        """
        super().reset()
        # self.prev_downsample_freq = 10
        # self.downsample_freq = 10

    def EstimateCorrespondences(self, X: np.ndarray, Y: np.ndarray,
        t: np.ndarray, R: np.ndarray, dmax: float = 0.05) -> np.ndarray:
        t = t.reshape(1, -1)
        transformed_X = (X @ R.T) + t

        # Build a KD-tree for Y
        tree = cKDTree(Y)
        # Query the KD-tree for each transformed point in X
        distances, indices = tree.query(transformed_X)

        # Filter based on dmax and create correspondence pairs
        C = np.array([[X[i], Y[idx]] for i, (dist, idx) in
                      enumerate(zip(distances, indices)) if dist < dmax])
        return C

    def ComputeOptimalRigidRegistration(self, C: np.array):
        """
        :param C: Set of estimated point correspondences between X and Y
        :return: T = (t, R) ￿ SE(d) that optimally aligns corresponding points of X␣
        ↪and Y in the least-squares sense
        """

        X = np.array([pair[0] for pair in C])
        Y = np.array([pair[1] for pair in C])
        # Compute the centroids of X and Y
        x_centroid = np.mean(X, axis=0)
        y_centroid = np.mean(Y, axis=0)
        deriv_x = X - x_centroid
        deriv_y = Y - y_centroid
        W = 1 / len(C) * sum([np.outer(deriv_y[i], deriv_x[i].T) for i in range(len(C))])
        U, S, V_T = np.linalg.svd(W)
        R = U @ np.diag([1, 1, np.linalg.det(U @ V_T)]) @ V_T
        # R = V @ U.T
        t = y_centroid - R @ x_centroid
        return t, R

    def ICP_algorithm(self, X : np.array, Y: np.array, t: np.array, R: np.array, dmax: float = .25, num_ICP_iters: int = 30) -> np.array:
        """
        :param X: Pointcloud X, set of points in Rd
        :param Y: Pointcloud Y, set of points in Rd
        :param t: estimated rigid translation to align X to Y
        :param R: Estimated rigid rotation to align X to Y
        :param dmax: Maximum admissible distance for associating two points
        :param num_ICP_iters: Number of ICP iterations to perform

        :return: An estimated set C of correspondences between points in X and Y,
        and the rigid registration T = (ˆt, ˆR) ￿ SE(d) that optimally aligns␣
        ↪corresponding points
        of X and Y in the least-squares sense
        """
        for _ in range(num_ICP_iters):
            C = self.EstimateCorrespondences(X, Y, t, R, dmax)
            t, R = self.ComputeOptimalRigidRegistration(C)
            self.logger.debug(f"t: {t}, R: {R}")
        return t, R, C


if __name__ == "__main__":
    pass