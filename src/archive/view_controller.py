import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

from process_point_clouds import ProcessPointCloudsThread

import os
os.environ["OMP_NUM_THREADS"] = str(os.cpu_count())


class ViewController:
    """
    A class to visualize point clouds in real-time using Open3D.
    """
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
        render_options = self.vis.get_render_option()
        render_options.point_size = 7.0  # Increase point size
        render_options.background_color = np.array([1.0, 1.0, 1.0])  # White background
        render_options.light_on = True  # Enable lighting for better structure

        self.processor = ProcessPointCloudsThread()


    def set_initial_view(self) -> None:
        """
        Set the initial camera parameters of the visualizer

        """
        ctr = self.vis.get_view_control()
        param = o3d.io.read_pinhole_camera_parameters(os.path.join(os.path.curdir, 'configs/viewpoint.json'))
        ctr.set_constant_z_far(10000)  # Set far clipping plane (increase as needed)
        ctr.set_constant_z_near(0.1)
        ctr.convert_from_pinhole_camera_parameters(param, allow_arbitrary=True)


    def run(self) -> None:
        """
        Run the visualizer and update the point cloud in real-time as new data is available
        """
        # Start the processing thread in the background
        self.processor.start()
        self.set_initial_view()
        self.vis.add_geometry(self.point_cloud)
        temp = True
        try:
            while True:
                # Fetch the latest scan data
                new_points = self.processor.get_latest_map()

                #TODO:
                # This logic for updating the visualizer is based on previous implementation,
                # update it to to make more sense with the current implementation

                # None returned only when the database in empty i.e. after being cleared by the user
                # We need to clear the visualizer as well in preparation for new data
                if new_points is None:
                    if len(self.point_cloud.points) > 0:
                        self.point_cloud.clear()
                        self.vis.clear_geometries()
                        self.vis.add_geometry(self.point_cloud)
                        self.vis.add_geometry(self.axis)
                        self.set_initial_view()

                elif len(new_points.points) > 0:
                    # Update the point cloud if new data is available
                    new_points = new_points.voxel_down_sample(voxel_size=0.02)
                    self.point_cloud.points = new_points.points

                    points = np.asarray(self.point_cloud.points)
                    depths = points[:, 2]
                    depth_normalized = (depths - depths.min()) / (depths.max() - depths.min())  # Normalize to [0, 1]
                    colors = plt.cm.viridis(depth_normalized)[:, :3]  # Apply colormap and extract RGB
                    self.point_cloud.colors = o3d.utility.Vector3dVector(colors)

                    # hack because open3d add_geometry() is weird
                    if temp:
                        temp = False
                        self.vis.add_geometry(self.point_cloud)
                        self.set_initial_view()

                    self.vis.update_geometry(self.point_cloud)

                # Keep the visualizer running and handle events
                self.vis.poll_events()
                self.vis.update_renderer()

                # Optional delay to control refresh rate
                # time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopped by user")


        # Close the visualizer when done
        self.vis.destroy_window()
        self.processor.stop()

if __name__ == "__main__":
    view = ViewController()
    view.run()
