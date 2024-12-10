import random
import threading
import time
from typing import Optional

import open3d as o3d
import numpy as np
import pandas as pd
from sqlalchemy import create_engine, Table, MetaData

# Database connection settings
DB_SETTINGS = {
    'host': "localhost",
    'database': "postgres",
    'user': "newuser",
    'password': "password"
}


def compute_normals(point_cloud, radius=0.1, max_nn=30):
    """
    Compute normals for a point cloud. No longer used in the final implementation.

    :param point_cloud: Open3D PointCloud object
    :param radius: Radius for normal estimation
    :param max_nn: Maximum number of neighbors for normal estimation
    """

    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    )
    point_cloud.orient_normals_consistent_tangent_plane(k=30)  # Optional for consistency
    return point_cloud


class ProcessPointClouds:
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """
    def __init__(self):

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
        self.engine = create_engine(f"postgresql+psycopg2://{DB_SETTINGS['user']}:{DB_SETTINGS['password']}@{DB_SETTINGS['host']}/{DB_SETTINGS['database']}")
        self.cur_scan_id = 0

        self.point_cloud_map = o3d.geometry.PointCloud()
        self.point_clouds_in_map = []
        metadata = MetaData()
        self.processed_point_table = Table('point_cloud', metadata, autoload_with=self.engine)

        # Continuously fetch new point clouds and process them so when the user requests
        # the latest point cloud map, it's readily available
        self.processing_thread = threading.Thread(target=self.run)
        self.process = True
        self.flag_new_points = True

    def get_new_elements(self) -> Optional[o3d.geometry.PointCloud]:
        """
        Get the latest point cloud map. If new points are available, return the
        updated map. If no new points are available, return an empty map.
        If the database is empty, return None.

        :return: Open3D PointCloud object of global map
        """
        if self.flag_new_points:
            self.flag_new_points = False
            return self.point_cloud_map.__copy__()#.voxel_down_sample(0.1)
        elif len(self.point_clouds_in_map) == 0:
            return None
        return o3d.geometry.PointCloud()

    def run(self) -> None:
        """
        Continuously fetch new point clouds from the database and process them.
        """
        while self.process:
            new_pointcloud = self.get_next_pointcloud()
            if new_pointcloud is None or len(new_pointcloud.points) == 0:
                time.sleep(1)
                continue
            self.flag_new_points = True
            # if len(self.point_clouds_in_map) % 5 == 0:
                # result_fgr = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
                #     source, target, source_feature, target_feature,
                #     o3d.pipelines.registration.FastGlobalRegistrationOption(
                #         maximum_correspondence_distance=0.05
                #     )
                # )

            # pd.DataFrame(new_pointcloud.points, columns=['x','y','z']).to_sql('global_map', self.engine, if_exists='append', index=False)



    def get_next_pointcloud(self) -> Optional[o3d.geometry.PointCloud]:
        """
        Fetch the next point cloud from the database and process it.

        :return: The global map with the new point cloud added if possible
        """
        # Fetch the latest scan_id
        query_max_id = "SELECT MAX(scan_id) FROM point_cloud;"
        max_id_df = pd.read_sql_query(query_max_id, self.engine)
        max_scan_id = max_id_df.iloc[0, 0]

        # Check if there's any new data
        if max_scan_id is None:
            # database was empty
            self.point_clouds_in_map = []
            self.cur_scan_id = 0
            self.point_cloud_map.clear()
            return None
        elif max_scan_id == self.cur_scan_id:
            # No new data
            return o3d.geometry.PointCloud()

        print(f"Fetching point cloud data for scan_id: {self.cur_scan_id + 1}")
        self.cur_scan_id += 1

        # Fetch the point cloud data for the maximum scan_id
        query_points = f"SELECT x, y, z FROM point_cloud WHERE scan_id = {self.cur_scan_id};"
        points_df = pd.read_sql_query(query_points, self.engine)

        # Convert to NumPy array for Open3D
        points = points_df[['x', 'y', 'z']].values
        if len(points) == 0:
            return o3d.geometry.PointCloud()
        points_3d = np.zeros((len(points), 3))

        # Convert pixel coordinates to 3D coordinates
        for i in range(0, len(points), 3):
            points_3d[i] = self.project_pixel_to_3d(points[i][0], points[i][1], points[i][2])
        # align the point cloud with the global map
        return self.locate_pc_in_world(points_3d)


    def project_pixel_to_3d(self, x, y, depth) -> np.ndarray:
        """
        Convert pixel coordinates to 3D coordinates in meters using the camera intrinsics.

        :param x: X pixel coordinate
        :param y: Y pixel coordinate
        :param depth: Depth value in meters at the x,y pixel coordinate

        :return: X, Y, Z coordinates in meters
        """
        z = depth
        x = (x - self.cx) * z / self.fx
        y = (y - self.cy) * z / self.fy
        return np.array([x, y, z])


    def locate_pc_in_world(self, points) -> o3d.geometry.PointCloud:
        """
        Align the new point cloud with the global map using ICP and add it to the map.

        :param points: The new point cloud to add to the global map

        :return: The new point cloud transformed to align with the global map
        """
        # Do some basic point cloud processing, dont know if this is necessary
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        point_cloud.remove_duplicated_points()
        point_cloud.remove_statistical_outlier(nb_neighbors=10, std_ratio=1.0)
        point_cloud.remove_non_finite_points()
        point_cloud.remove_radius_outlier(nb_points=10, radius=0.02)
        random_color = (random.random(), random.random(), random.random())
        point_cloud.paint_uniform_color(random_color)

        # If this is the first point cloud, set it as the global map
        if len(self.point_clouds_in_map) == 0:
            self.point_clouds_in_map.append(point_cloud.__copy__())
            self.point_cloud_map = point_cloud
            return point_cloud

        # Perform ICP alignment
        icp_result_transformation = self.align_point_clouds_with_ransac_icp(point_cloud, self.point_cloud_map)
        if icp_result_transformation is None:
            return self.point_cloud_map.__copy__()#.voxel_down_sample(0.1)

        point_cloud = point_cloud.transform(icp_result_transformation)
        self.point_clouds_in_map.append(point_cloud)
        self.point_cloud_map += point_cloud
        # self.point_cloud_map = self.point_cloud_map.voxel_down_sample(0.01)
        return point_cloud


    def align_point_clouds_with_ransac_icp(self, source_cloud, target_cloud, voxel_size=0.02) -> np.ndarray:
        """
        Align two point clouds using RANSAC and then ICP.

        :param source_cloud: The new point cloud to align
        :param target_cloud: The global map to align the new point cloud with
        :param voxel_size: The voxel size for downsampling the point clouds

        :return: The 4x4 transformation matrix to align the new point cloud with the global map
        """
        # Downsample the clouds
        source_down = source_cloud.__copy__().voxel_down_sample(voxel_size)
        target_down = target_cloud.__copy__().voxel_down_sample(voxel_size)

        # Estimate normals
        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
        source_down.orient_normals_consistent_tangent_plane(k=30)  # Optional for consistency
        target_down.orient_normals_consistent_tangent_plane(k=30)  # Optional for consistency

        # Compute FPFH features
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 4, max_nn=100)
        )
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 4, max_nn=100)
        )

        # RANSAC for initial alignment
        distance_threshold = .3#voxel_size * 1.5
        result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, False,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3,  # RANSAC iterations
            [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
            ],
            o3d.pipelines.registration.RANSACConvergenceCriteria(max_iteration=400000, confidence=0.999)
        )

        # Refine with ICP
        result_icp = o3d.pipelines.registration.registration_icp(
            source_down, target_down,
            max_correspondence_distance=.1,
            init=result_ransac.transformation,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=500,  # Increase the number of iterations
                relative_fitness=1e-6,  # Convergence threshold for fitness
                relative_rmse=1e-6     # Convergence threshold for RMSE
            )
        )

        # print("ICP Refined Transformation:")
        # print(result_icp.transformation)
        # print(f"Fitness: {result_icp.fitness}, RMSE: {result_icp.inlier_rmse}")

        return result_icp.transformation
