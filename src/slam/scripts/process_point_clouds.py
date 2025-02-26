"""
This module contains classes process and store point clouds. Gets raw point
cloud data from the ros2, converts it from pixels to meters then does some
algorithm to store it in a global map. All algorithms are subclasses of the
ProcessPointClouds class. The ICPProcessor class aligns the new point cloud with
"""
import asyncio
import multiprocessing
import os
import threading
import time
from abc import ABC, abstractmethod

import numpy as np
import open3d as o3d
import rclpy.logging
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

        # If this is the first point cloud, set it as the global map
        if self.point_clouds_in_map == 0:
            self.point_clouds_in_map += 1
            self.global_map = points
            return
        
        # Do some basic point cloud processing, dont know if this is necessary
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
