import os
import random
from datetime import datetime

import open3d as o3d
import numpy as np
import time

from process_point_clouds import ProcessPointClouds


class ViewController:
    def __init__(self):
        # Create a visualizer window
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(width=1920, height=1080)

        # Initialize an empty point cloud
        self.point_cloud = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.point_cloud)

        self.axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])  # Adjust size as needed
        self.vis.add_geometry(self.axis)
        # Set initial view parameters (optional)
        self.vis.get_render_option().background_color = np.array([1, 1, 1])  # White background

        self.processor = ProcessPointClouds()

    def set_initial_view(self, ctr, params):
        ctr = self.vis.get_view_control()
        param = o3d.io.read_pinhole_camera_parameters(os.path.join(os.path.curdir, 'viewpoint.json'))
        ctr.convert_from_pinhole_camera_parameters(param, allow_arbitrary=True)


    def run(self):
        self.processor.processing_thread.start()
        ctr = self.vis.get_view_control()
        params = ctr.convert_to_pinhole_camera_parameters()
        self.set_initial_view(ctr, params)
        self.vis.add_geometry(self.point_cloud)
        temp = True
        try:
            while True:
                # Fetch the latest scan data
                new_points = self.processor.get_new_elements()

                # print(new_points)
                # print(new_points)
                if new_points is None:
                    self.point_cloud.clear()
                    self.vis.clear_geometries()
                    self.vis.add_geometry(self.point_cloud)
                    self.vis.add_geometry(self.axis)
                    self.set_initial_view(ctr, params)

                elif len(new_points.points) > 0:
                    # print(f"Received {len(new_points.points)} {new_points.colors[0]} new points")

                    # new_points.paint_uniform_color([0.5, 0.5, 0.5])
                    self.point_cloud.points.extend(new_points.points)
                    # if random.random() < 0.1:
                    #     self.point_cloud = self.processor.point_cloud_map.voxel_down_sample(0.05)

                    # Update the point cloud if new data is available
                    if temp:
                        temp = False
                        self.vis.add_geometry(self.point_cloud)
                        self.set_initial_view(ctr, params)

                    self.vis.update_geometry(self.point_cloud)
                    # self.vis.update_renderer()
                    # self.set_initial_view(ctr=ctr, params=params)
                    # self.vis.update_renderer()


                # Keep the visualizer running and handle events
                self.vis.poll_events()
                self.vis.update_renderer()

                # Optional delay to control refresh rate
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopped by user")

        # Close the visualizer when done
        self.vis.destroy_window()
        self.processor.process = False
        self.processor.processing_thread.join(timeout=10)


view = ViewController()
view.run()
