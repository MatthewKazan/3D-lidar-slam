import os
import signal

import yaml

import multiprocessing
import time
from collections import deque
from queue import Queue
from typing import Optional
from abc import ABC, abstractmethod

import numpy as np
import open3d as o3d

from slam.point_cloud_publisher import PointCloudPublisher


class ProcessPointClouds(ABC):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self,
        config_path: str,
        stop_event: multiprocessing.Event,
        input_queue: Queue,
        reset_event: multiprocessing.Event,
        publisher_node: PointCloudPublisher,
    ):
        """
        Initialize the point cloud processor with settings from a YAML file.
        Should be run in a separate process since it's a long-running task.

        :param config_path: Path to the YAML configuration file.
        :param input_queue: Thread-safe queue to get the latest point cloud map.
        :param publisher_node: Publisher node to publish the global map after each processed point cloud.
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

        self.global_map = None
        self.point_clouds_in_map = 0

        self.previous_transformation = [np.identity(4)]

        self.start_time = None
        self.input_queue = input_queue
        self.stop_event = stop_event
        self.reset_event = reset_event
        self.publisher_node = publisher_node
        self.publisher_node.get_logger().info(f"Using {self.__class__.__name__} for point cloud processing")

        # Start Processing Thread
        # self.process_thread = multiprocessing.Process(target=self.start_process_thread)

        # Handle system signals (SIGTERM, SIGINT)
        # signal.signal(signal.SIGINT, self.signal_handler)
        # signal.signal(signal.SIGTERM, self.signal_handler)
        # self.start_process_thread()

    def load_config(self, config_name: str):
        """
        Loads configuration parameters from a YAML file.
        """
        dir_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../../"))
        config_path = os.path.join(dir_path,
                                          f'configs/{config_name}')
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


    def run(self):
        """
        Main loop to process point clouds.
        """
        while not self.stop_event.is_set():
            # Get the latest point cloud from the database
            if self.input_queue.empty():
                time.sleep(0.1)
                continue
            point_cloud_pixel = self.input_queue.get()
            self.publisher_node.get_logger().info(f"point cloud received from queue")

            # Process the point cloud
            if point_cloud_pixel is not None:
                point_cloud_3d = self.project_pixel_to_3d(point_cloud_pixel)
                self.construct_global_map(point_cloud_3d)
                self.publisher_node.publish_point_cloud(self.global_map)
                self.point_clouds_in_map += 1

            # Sleep for a short duration to avoid busy-waiting
            time.sleep(0.01)


    @abstractmethod
    def construct_global_map(self, points: np.array) -> None:
        """
        Construct the global map from the point cloud.

        :param points: A numpy array of x,y,z points in meters
        """
        pass


class ICPProcessor(ProcessPointClouds):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self,
        config_path: str,
        publisher_node: PointCloudPublisher,
        input_queue: Queue,
        reset_event: multiprocessing.Event,
        stop_event: multiprocessing.Event,
    ):
        """
        Initialize the point cloud processor with settings from a YAML file.
        Should be run in a separate process since it's a long-running task.

        :param config_path: Path to the YAML configuration file.
        :param input_queue: Thread-safe queue to get the latest point cloud map.
        :param publisher_node: Publisher node to publish the global map after each processed point cloud.
        :param reset_event: Thread-safe event triggered on database reset.
        """
        super().__init__(
            config_path=config_path,
            publisher_node=publisher_node,
            stop_event=stop_event,
            input_queue=input_queue,
            reset_event=reset_event
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
        self.point_clouds_in_map += 1
        self.global_map = np.asarray((point_cloud + o3d_global_map).points)

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



def get_processor(
    algorithm: str,
    config_path: str,
    queue: multiprocessing.Queue,
    reset_event: multiprocessing.Event,
    stop_event: multiprocessing.Event,
    publisher_node: PointCloudPublisher,
) -> ProcessPointClouds:
    """
    Factory function to get the appropriate point cloud processor class based on the algorithm.
    """
    if algorithm == 'icp':
        return ICPProcessor(
            config_path=config_path,
            publisher_node=publisher_node,
            stop_event=stop_event,
            input_queue=queue,
            reset_event=reset_event
        )
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}")