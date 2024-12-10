import os
import open3d as o3d
import numpy as np

from process_point_clouds import ProcessPointClouds


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
        self.vis.get_render_option().background_color = np.array([1, 1, 1])  # White background

        self.processor = ProcessPointClouds()

    def set_initial_view(self) -> None:
        """
        Set the initial camera view of the visualizer to be the initial camera position

        """
        ctr = self.vis.get_view_control()
        param = o3d.io.read_pinhole_camera_parameters(os.path.join(os.path.curdir, 'viewpoint.json'))
        ctr.convert_from_pinhole_camera_parameters(param, allow_arbitrary=True)


    def run(self) -> None:
        """
        Run the visualizer and update the point cloud in real-time as new data is available
        """
        # Start the processing thread in the background
        self.processor.processing_thread.start()
        self.set_initial_view()
        self.vis.add_geometry(self.point_cloud)
        temp = True
        try:
            while True:
                # Fetch the latest scan data
                new_points = self.processor.get_new_elements()

                # None returned only when the database in empty i.e. after being cleared by the user
                # We need to clear the visualizer as well in preparation for new data
                if new_points is None:
                    self.point_cloud.clear()
                    self.vis.clear_geometries()
                    self.vis.add_geometry(self.point_cloud)
                    self.vis.add_geometry(self.axis)
                    self.set_initial_view()

                elif len(new_points.points) > 0:
                    # Update the point cloud if new data is available
                    self.point_cloud.points = new_points.points
                    # self.point_cloud.colors = new_points.colors

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
        self.processor.process = False
        self.processor.processing_thread.join(timeout=10)


view = ViewController()
view.run()
