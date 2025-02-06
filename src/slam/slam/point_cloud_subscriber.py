import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String
import multiprocessing
import time
from collections import deque
from multiprocessing import Queue
from typing import Optional

import numpy as np
import open3d as o3d
from sqlalchemy import create_engine, text


#TODO - decide whether to use rviz or open3d for visualization
#TODO - actually generate the global point cloud map
#TODO - implement a way to reset the visualizer
#TODO - built in ros2 icp?
#TODO - speed up everything TO SLOWW
#TODO - figure out how to use deep learning for this
#TODO - make it easier to run everything - to many steps, do we need advertiser?
#TODO - run on linux, is readme correct?

class PointClouds2Subscriber(Node):
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

        # Camera stuff for pixel to 3D conversion
        self.WIDTH = 256
        self.HEIGHT = 192
        self.ref_width = 1920
        self.ref_height = 1440
        self.fx = 1342.9849
        self.fy = 1342.9849
        self.cx = 965.6142
        self.cy = 717.95435
        scale_x = self.WIDTH / self.ref_width
        scale_y = self.HEIGHT / self.ref_height
        self.fx = self.fx * scale_x
        self.fy = self.fy * scale_y
        self.cx = self.cx * scale_x
        self.cy = self.cy * scale_y

        # Create SQLAlchemy engine
        self.cur_scan_id = 0

        self.point_cloud_map = o3d.geometry.PointCloud()
        self.point_clouds_in_map = 0

        self.previous_transformation = [np.identity(4)]

        self.start_time = None

        super().__init__('pointcloud_processor')
        self.pc_subscription = self.create_subscription(
            PointCloud2,
            '/input_pointcloud',  # Name of the topic you want to subscribe to
            self.listener_callback,
            10
        )
        self.clear_db_subscription = self.create_subscription(
            String,
            '/reset',  # Name of the topic you want to subscribe to
            self.reset_callback,
            10
        )
        self.get_logger().info('PointCloud processor has been started.')
        self.publisher_ = self.create_publisher(PointCloud2, '/output_pointcloud', 10)



    def listener_callback(self, msg: PointCloud2):
        self.get_logger().info("Received point cloud message")
        # Convert PointCloud2 message to a more usable format (e.g., list of points)
        points_list = list(
            pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        )

        if not points_list:
            self.get_logger().warn("No valid points found in the message.")
            return

        # Convert list of points to a NumPy array of shape (N, 3)
        points_np = self.project_pixel_to_3d(points_list)
        output = pc2.create_cloud_xyz32(msg.header, points_np)
        self.publisher_.publish(output)

        # Create an Open3D point cloud and set its points
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(points_np)
        self.locate_pc_in_world(o3d_pc)
        # You can add additional processing here

    def reset_callback(self, msg):
        self.get_logger().info("Clearing database")
        self.point_clouds_in_map = 0
        self.cur_scan_id = 0
        self.point_cloud_map.clear()
        self.previous_transformation = [np.identity(4)]


    def locate_pc_in_world(self, point_cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """
        Align the new point cloud with the global map using ICP and add it to the map.

        :param point_cloud: The new point cloud to add to the global map

        :return: The new point cloud transformed to align with the global map
        """
        # Do some basic point cloud processing, dont know if this is necessary
        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(points)
        # Can this be done quickly?
        # point_cloud, _ = point_cloud.remove_statistical_outlier(nb_neighbors=5, std_ratio=6)

        # If this is the first point cloud, set it as the global map
        if self.point_clouds_in_map == 0:
            self.point_clouds_in_map += 1
            self.point_cloud_map = point_cloud
            return point_cloud

        # Perform ICP alignment
        icp_result_transformation = self.align_point_clouds_with_icp(
            point_cloud, self.point_cloud_map)
        if icp_result_transformation is None:
            return self.point_cloud_map  # .voxel_down_sample(0.1)

        point_cloud = point_cloud.transform(icp_result_transformation)
        self.point_clouds_in_map += 1
        self.point_cloud_map += point_cloud

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







class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'pointcloud_topic',  # Name of the topic you want to subscribe to
            self.listener_callback,
            10
        )
        self.get_logger().info('PointCloud subscriber has been started.')

    def listener_callback(self, msg):
        self.get_logger().info("Received point cloud message")
        # Convert PointCloud2 message to a more usable format (e.g., list of points)
        point_cloud = pc2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        for point in point_cloud:
            self.get_logger().info(f"Point: {point}")
        # You can add additional processing here

def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointClouds2Subscriber()

    rclpy.spin(pointcloud_subscriber)

    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
