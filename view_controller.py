import os

import open3d as o3d
import numpy as np
import time
import pandas as pd
from keras.src.layers.preprocessing.image_preprocessing import HORIZONTAL
from sqlalchemy import create_engine

# Database connection settings
DB_SETTINGS = {
    'host': "localhost",
    'database': "postgres",
    'user': "newuser",
    'password': "password"
}

HORIZONTAL_FOV = 75  # Horizontal field of view in degrees
VERTICAL_FOV = 48    # Vertical field of view in degrees
WIDTH = 256
HEIGHT = 192
fx = 1342.9849 #WIDTH / (2.0 * np.tan(np.radians(HORIZONTAL_FOV) / 2.0))
fy = 1342.9849#HEIGHT / (2.0 * np.tan(np.radians(VERTICAL_FOV) / 2.0))
cx = 965.6142 #WIDTH / 2.0
cy = 717.95435#HEIGHT / 2.0


class ViewController:
    def __init__(self):
        # Create SQLAlchemy engine
        self.engine = create_engine(f"postgresql+psycopg2://{DB_SETTINGS['user']}:{DB_SETTINGS['password']}@{DB_SETTINGS['host']}/{DB_SETTINGS['database']}")
        self.cur_scan_id = -1

        # Create a visualizer window
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(width=1920, height=1080)

        # Initialize an empty point cloud
        self.point_cloud = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.point_cloud)

        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])  # Adjust size as needed
        self.vis.add_geometry(axis)
# Set initial view parameters (optional)
        self.vis.get_render_option().background_color = np.array([1, 1, 1])  # White background

        self.HORIZONTAL_FOV = 100  # Horizontal field of view in degrees
        self.VERTICAL_FOV = 1    # Vertical field of view in degrees
        self.WIDTH = 256
        self.HEIGHT = 192
        self.fx = 1342.9849
        self.fy = 1342.9849
        self.cx = 965.6142
        self.cy = 717.95435

    def project_pixel_to_3d(self, x, y, depth):
        # Calculate the 3D point in camera coordinates
        ref_width = 1920
        ref_height = 1440

        scale_x = self.WIDTH / ref_width
        scale_y = self.HEIGHT / ref_height
        self.fx = fx * scale_x
        self.fy = fy * scale_y
        self.cx = cx * scale_x
        self.cy = cy * scale_y
        z = depth
        x = (x - self.cx) * z / self.fx
        y = (y - self.cy) * z / self.fy
        return np.array([x, y, z])


    def fetch_new_point_cloud(self):
        # Fetch the latest scan_id
        query_max_id = "SELECT MAX(scan_id) FROM point_cloud;"
        max_id_df = pd.read_sql_query(query_max_id, self.engine)
        max_scan_id = max_id_df.iloc[0, 0]

        # Check if there's any new data
        if max_scan_id is None or max_scan_id == self.cur_scan_id:
            return None

        print(f"Fetching point cloud data for scan_id: {max_scan_id}")
        self.cur_scan_id = max_scan_id

        # Fetch the point cloud data for the maximum scan_id
        query_points = f"SELECT x, y, z FROM point_cloud WHERE scan_id = {max_scan_id};"
        points_df = pd.read_sql_query(query_points, self.engine)

        # Convert to NumPy array for Open3D
        points = points_df[['x', 'y', 'z']].values
        for i in range(len(points)):
            points[i] = self.project_pixel_to_3d(points[i][0], points[i][1], points[i][2])

    # print(max(points[:, 0]), min(points[:, 0]), max(points[:, 1]), min(points[:, 1]), max(points[:, 2]), min(points[:, 2]))
        return points

    def display_camera_info(self):
        ctr = self.vis.get_view_control()
        params = ctr.convert_to_pinhole_camera_parameters()
        # print("Camera position:", params.extrinsic[:3, 3])  # Extract position
        # print("Lookat:", params.extrinsic[:3, 2])            # Extract lookat vector
        # print("Up vector:", params.extrinsic[:3, 1])         # Extract up vector

    def set_initial_view(self, ctr, params):
        ctr = self.vis.get_view_control()
        param = o3d.io.read_pinhole_camera_parameters(os.path.join(os.path.curdir, 'viewpoint.json'))
        ctr.convert_from_pinhole_camera_parameters(param, allow_arbitrary=True)

    def run(self):

        try:
            while True:
                # Fetch the latest scan data
                new_points = self.fetch_new_point_cloud()

                # Update the point cloud if new data is available
                if new_points is not None:
                    ctr = self.vis.get_view_control()
                    params = ctr.convert_to_pinhole_camera_parameters()
                    self.point_cloud.points = o3d.utility.Vector3dVector(new_points)
                    self.vis.add_geometry(self.point_cloud)
                    self.vis.update_geometry(self.point_cloud)
                    self.vis.update_renderer()
                    self.set_initial_view(ctr=ctr, params=params)
                    self.vis.update_renderer()



            # Display camera information
                self.display_camera_info()

                # Keep the visualizer running and handle events
                self.vis.poll_events()
                self.vis.update_renderer()

                # Optional delay to control refresh rate
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopped by user")

        # Close the visualizer when done
        self.vis.destroy_window()

view = ViewController()
view.run()
