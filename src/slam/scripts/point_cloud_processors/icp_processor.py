"""
This module contains classes process and store point clouds. Gets raw point
cloud data from the ros2, converts it from pixels to meters then does some
algorithm to store it in a global map. All algorithms are subclasses of the
ProcessPointClouds class. The ICPProcessor class aligns the new point cloud with
"""
import multiprocessing

import numpy as np
import open3d as o3d
import rclpy.logging

from scripts.data_transfer import DataTransfer

from scripts.point_cloud_processors.generic_point_cloud_processor import ProcessPointClouds


class ICPProcessor(ProcessPointClouds):
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
            rclpy.logging.get_logger("icp processor")
        )

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

        # Perform ICP alignment
        icp_result_transformation = self.align_point_clouds_with_icp(
            point_cloud, o3d_global_map)
        if icp_result_transformation is None:
            return

        point_cloud = point_cloud.transform(icp_result_transformation)
        self.global_map = np.asarray((point_cloud + o3d_global_map).points)
        self.do_stuff()

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
        source_down = source_cloud.voxel_down_sample(voxel_size)
        target_down = target_cloud.voxel_down_sample(voxel_size)

        # # # Estimate normals
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2,
                                                 max_nn=20))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2,
                                                 max_nn=20))

        # print("time to get to registration_icp: ", time.time() - self.start_time)
        # Align with ICP
        result_icp = o3d.pipelines.registration.registration_icp(
            source_down, target_down,
            max_correspondence_distance=voxel_size * 2.5,
            init=self.previous_transformation[-1],
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=500,
                relative_fitness=1e-6,
                relative_rmse=1e-6
            )
        )

        # print("ICP Refined Transformation:")
        # print(result_icp.transformation)
        # print(f"Fitness: {result_icp.fitness}, RMSE: {result_icp.inlier_rmse}")
        self.previous_transformation.append(result_icp.transformation)
        return result_icp.transformation

    def do_stuff(self) -> None:
        """
        Do some basic point cloud processing on the global map every few point
        clouds, remove outliers, downsample, etc.
        """
        # Pulled all of these numbers out of nowhere
        if self.point_clouds_in_map < 10:
            return
        if self.point_clouds_in_map % 10 == 0 or self.data_transfer.pixel_depth_map_queue.empty():
            point_cloud_map = o3d.geometry.PointCloud()
            point_cloud_map.points = o3d.utility.Vector3dVector(self.global_map)
            point_cloud_map: o3d.geometry.PointCloud = point_cloud_map.voxel_down_sample(
                0.001)

            if self.point_clouds_in_map % 40 == 0:
                point_cloud_map, _ = point_cloud_map.remove_statistical_outlier(
                    nb_neighbors=80, std_ratio=2)
                # This is slow but seems to make a difference
                point_cloud_map, _ = point_cloud_map.remove_radius_outlier(
                    nb_points=8, radius=0.023)

                self.global_map = np.asarray(point_cloud_map.points)
                return

            point_cloud_map, _ = point_cloud_map.remove_statistical_outlier(
                nb_neighbors=45, std_ratio=3.5)

            self.global_map = np.asarray(point_cloud_map.points)

    def downsample_global_map(self) -> np.ndarray:
        """
        Downsample the global map to reduce the number of points. maybe unnecessary

        :return: The downsampled global map
        """
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(self.global_map)
        # point_cloud = point_cloud.voxel_down_sample(0.2)
        # self.global_map = np.asarray(point_cloud.points)
        return point_cloud.points


if __name__ == "__main__":
    pass