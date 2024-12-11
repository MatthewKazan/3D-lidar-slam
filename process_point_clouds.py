import multiprocessing
import time
from collections import deque
from multiprocessing import Queue
from typing import Optional

import numpy as np
import open3d as o3d
from sqlalchemy import create_engine, text

# Database connection settings
DB_SETTINGS = {
    'host': "localhost",
    'database': "postgres",
    'user': "newuser",
    'password': "password"
}

import os

os.environ["OMP_NUM_THREADS"] = str(os.cpu_count())


class ProcessPointClouds:
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self, result_queue: Queue, stop_event: multiprocessing.Event,
        reset_event: multiprocessing.Event):
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
        self.engine = create_engine(
            f"postgresql+psycopg2://{DB_SETTINGS['user']}:{DB_SETTINGS['password']}@{DB_SETTINGS['host']}/{DB_SETTINGS['database']}")
        self.cur_scan_id = 0

        self.point_cloud_map = o3d.geometry.PointCloud()
        self.point_clouds_in_map = 0

        # Continuously fetch new point clouds and process them so when the user requests
        # the latest point cloud map, it's readily available
        # Multiprocessing setup
        self.stop_event = stop_event
        self.result_queue = result_queue

        self.previous_transformation = [np.identity(4)]

        self.start_time = None
        self.reset_event = reset_event

    def run(self) -> None:
        """
        Continuously fetch new point clouds from the database and process them.
        """
        past_20_ave_time = deque(maxlen=20)
        no_new_data = None

        while not self.stop_event.is_set():
            # get the time it takes to process a point cloud for debugging
            self.start_time = time.time()
            new_pointcloud = self.get_next_pointcloud()
            end_time = time.time()
            past_20_ave_time.append(end_time - self.start_time)

            if new_pointcloud is None:
                continue
            if len(new_pointcloud.points) == 0:
                self.result_queue.put(
                    np.asarray(o3d.geometry.PointCloud().points))
                time.sleep(1)
                if no_new_data is None:
                    file_name = "app.ply"
                    print("writing to point cloud to file: {}".format(file_name))
                    o3d.io.write_point_cloud(file_name, self.point_cloud_map)
                    no_new_data = True
            else:
                # print(f"average time: {sum(past_20_ave_time) / len(past_20_ave_time)}")

                # Serialize the current map and send it to the queue
                self.result_queue.put(np.asarray(self.point_cloud_map.points))
                no_new_data = None

            if self.point_clouds_in_map % 10 == 0 or no_new_data:
                self.point_cloud_map: o3d.geometry.PointCloud = self.point_cloud_map.voxel_down_sample(
                    0.001)

                if self.point_clouds_in_map % 40 == 0 or no_new_data:
                    self.point_cloud_map, _ = self.point_cloud_map.remove_statistical_outlier(
                        nb_neighbors=60, std_ratio=2)
                    no_new_data = False
                    continue

                self.point_cloud_map, _ = self.point_cloud_map.remove_statistical_outlier(
                    nb_neighbors=30, std_ratio=3.5)
        print("Stopping point cloud processing thread")

    def reset(self) -> None:
        """
        When the user clears the database, reset the global map and point cloud list.
        """
        self.point_clouds_in_map = 0
        self.cur_scan_id = 0
        self.point_cloud_map.clear()
        self.previous_transformation = [np.identity(4)]
        self.reset_event.set()
        # stupid but im tired, wait for the visualizer to receive the reset event
        # and clear the queue, then its safe to move on
        while not self.result_queue.empty():
            pass

    def get_next_pointcloud(self) -> Optional[o3d.geometry.PointCloud]:
        """
        Fetch the next point cloud from the database and process it.

        :return: The global map with the new point cloud added if possible
        """
        with self.engine.connect() as connection:
            # Check if the table is empty, meaning the database was cleared
            query = text("""
                         SELECT EXISTS (SELECT 1 FROM point_cloud);
                         """)
            result = connection.execute(query)
            count = result.scalar()  # Fetch the single scalar value
            if count == 0:
                self.reset()
                return None
            query = text("""
                SELECT x, y, z
                FROM point_cloud
                WHERE scan_id = :scan_id
            """)
            result = connection.execute(query, {"scan_id": self.cur_scan_id + 1})

        points = result.fetchall()
        if len(points) == 0:
            return o3d.geometry.PointCloud()
        points_3d = np.zeros((len(points), 3))

        # Convert pixel coordinates to 3D coordinates
        for i in range(0, len(points), 3):
            points_3d[i] = self.project_pixel_to_3d(points[i][0], points[i][1],
                                                    points[i][2])
        print(f"Fetching point cloud data for scan_id: {self.cur_scan_id + 1}")
        self.cur_scan_id += 1

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


class ProcessPointCloudsThread:
    """
    Class to abstract multiprocessing for point cloud processing.
    """

    def __init__(self):
        self.result_queue = multiprocessing.Queue()
        self.stop_event = multiprocessing.Event()
        self.reset_event = multiprocessing.Event()

        self.processor = multiprocessing.Process(target=self.start_processing)

    def start_processing(self) -> None:
        """
        Start the point cloud processing thread. Should be run in a separate process.
        """
        processor = ProcessPointClouds(self.result_queue, self.stop_event,
                                       self.reset_event)
        try:
            processor.run()
        except KeyboardInterrupt:
            print("Stopped by user")

    def start(self) -> None:
        """
        Start the point cloud processing thread.
        """
        self.processor.start()

    def stop(self) -> None:
        """
        Stop the point cloud processing thread.
        """
        self.stop_event.set()
        self.processor.join(timeout=5)  # Wait for the process to finish
        if self.processor.is_alive():
            self.processor.terminate()  # Force terminate if it doesn't stop
            self.processor.join()

    def get_latest_map(self) -> Optional[o3d.geometry.PointCloud]:
        """
        Get the latest point cloud map from the processing thread. If the queue
        is empty, return an empty point cloud indicating no new data. If the
        latest point cloud is None, the database was cleared and the visualizer
        should reset.

        :return: The latest point cloud map, an empty point cloud if no new
            data, or None if the database was cleared
        """
        # If the database was cleared, reset the visualizer, and queue
        if self.reset_event.is_set():
            self.reset_event.clear()
            while not self.result_queue.empty():
                self.result_queue.get(block=False)
            return None

        if self.result_queue.empty():
            return o3d.geometry.PointCloud()
        points = self.result_queue.get()

        if points is None:
            return None
        point_cloud = o3d.geometry.PointCloud()

        point_cloud.points = o3d.utility.Vector3dVector(points)
        return point_cloud


if __name__ == "__main__":
    raise NotImplementedError("This script is not meant to be run directly.")
