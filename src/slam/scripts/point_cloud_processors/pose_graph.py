import queue
import threading
import time
from typing import Callable
import open3d as o3d

import numpy as np
import gtsam
import rclpy.logging
from gtsam import Pose3, Rot3


def pose_from_matrix(matrix):
    """Convert a 4x4 numpy array to a GTSAM Pose3 object."""
    return Pose3(Rot3(matrix[:3, :3]), gtsam.Point3(matrix[:3, 3]))

def cosine_similarity(desc1: np.ndarray, desc2: np.ndarray) -> float:
    """
    Compute cosine similarity between two descriptors.
    """
    return np.dot(desc1, desc2) / (np.linalg.norm(desc1) * np.linalg.norm(desc2) + 1e-8)

def compute_loop_transformation(source_cloud, target_cloud, t_source, t_target,
    voxel_size=0.02) -> np.ndarray:
    """
    Align two point clouds using RANSAC and then ICP.

    :param source_cloud: The new point cloud to align
    :param target_cloud: The global map to align the new point cloud with
    :param voxel_size: The voxel size for downsampling the point clouds

    :return: The 4x4 transformation matrix to align the new point cloud with the global map
    """
    t_init = np.linalg.inv(t_source) @ t_target

    source_cloud_o3d = o3d.geometry.PointCloud()
    source_cloud_o3d.points = o3d.utility.Vector3dVector(source_cloud)

    target_cloud_o3d = o3d.geometry.PointCloud()
    target_cloud_o3d.points = o3d.utility.Vector3dVector(target_cloud)

    # Align with ICP
    result_icp = o3d.pipelines.registration.registration_icp(
        source_cloud_o3d, target_cloud_o3d,
        max_correspondence_distance=voxel_size * 2.5,
        init=t_init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=500,
            relative_fitness=1e-6,
            relative_rmse=1e-6
        )
    )

    return result_icp.transformation


class PoseGraphGTSAMICP:

    def __init__(self,
        on_optimization_complete,
        trans_thresh: float = 0.5,
        rot_thresh_deg: float = 10.0,
        loop_closure_similarity_threshold: float = .9,
        optimization_frequency: int = 5,
        min_keyframe_gap: int = 10,
    ):
        """

        :param trans_thresh: Translation threshold for adding keyframe, in meters
        :param rot_thresh_deg: Rotation threshold for adding keyframe, in degrees
        """
        self.last_keyframe_pose = Pose3()
        self.current_pose = Pose3()
        self.trans_thresh = trans_thresh
        self.rot_thresh_deg = rot_thresh_deg
        self.keyframes = []
        self.on_optimization_complete = on_optimization_complete

        self.stop_loop = False
        self.loop_closure_queue = queue.Queue()

        # Initialize GTSAM graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        odom_sigmas = np.array(
            [0.2, 0.2, 0.2, 0.1, 0.1, 0.1])  # in meters and radians
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(odom_sigmas)
        loop_sigmas = np.array([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])
        self.loop_noise = gtsam.noiseModel.Diagonal.Sigmas(loop_sigmas)

        self.loop_closure_similarity_threshold = loop_closure_similarity_threshold
        self.optimization_frequency = optimization_frequency
        self.MIN_KEYFRAME_GAP = min_keyframe_gap

        self.optimized_global_map = None
        # self.processor_thread = threading.Thread(target=self.optimization_loop)

    # def start(self):
    #     self.processor_thread.start()

    def stop(self):
        self.stop_loop = True
        # self.processor_thread.join(timeout=5)
        rclpy.logging.get_logger("pose_graph").debug("Optimizer thread stopped")

    def reset(self):
        self.last_keyframe_pose = np.eye(4)
        self.current_pose = np.eye(4)
        self.keyframes = []
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.stop_loop = False


    def should_add_keyframe(self) -> bool:
        """
        Determine whether to add a new keyframe based on the current pose.

        :return: Boolean indicating whether to add a new keyframe
        """
        # Extract translations (last column of the matrix)
        t_current = self.current_pose.matrix()[:3, 3]
        t_last = self.last_keyframe_pose.matrix()[:3, 3]
        translation_diff = np.linalg.norm(t_current - t_last)

        # Compute rotation difference:
        # Use the relative rotation matrix R_diff = R_last^T * R_current.
        R_current = self.current_pose.matrix()[:3, :3]
        R_last = self.last_keyframe_pose.matrix()[:3, :3]
        R_diff = R_last.T @ R_current
        # Compute the rotation angle from the trace of R_diff.
        angle_rad = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)

        # Debug: print differences (optional)
        # print(f"Translation diff: {translation_diff:.2f} m, Rotation diff: {angle_deg:.2f}°")

        # If either difference exceeds the threshold, we add a new keyframe.
        return translation_diff > self.trans_thresh or angle_deg > self.rot_thresh_deg

    def update_pose_graph(self, trans: np.ndarray, scan_pc: np.ndarray, gen_descriptor: Callable[[np.ndarray], np.ndarray]) -> None:
        """
        Do stfuf

        :param trans: The recent transformation computation to transform scan_pc to global map frame
        :param scan_pc: The most recent point cloud scan
        :param gen_descriptor: The function to generate the keyframe descriptor

        :return:
        """
        rclpy.logging.get_logger("pose_graph").info(
            f"cur pos: {type(trans)}")
        self.current_pose = pose_from_matrix(trans)



        scan_pc_global = transform_point_cloud(scan_pc.copy(), trans)

        if self.should_add_keyframe():
            current_index = len(self.keyframes)

            self.last_keyframe_pose = self.current_pose

            descriptor = gen_descriptor(scan_pc_global)
            keyframe = {
                'pose': self.current_pose,
                'descriptor': descriptor,
                'point_cloud': scan_pc,
            }

            self.keyframes.append(keyframe)
            rclpy.logging.get_logger("pose_graph").info("Adding keyframe, total keyframes: " + str(len(self.keyframes)))

            self.loop_closure_queue.put(keyframe)
            if current_index == 0:
                # For the first keyframe, initialize the graph with a prior factor.
                prior_mean = gtsam.Pose3()
                self.graph.add(
                    gtsam.PriorFactorPose3(0, prior_mean, self.odom_noise))
                self.initial_estimates.insert(0, prior_mean)
            elif current_index == 1:
                prev_pose = self.initial_estimates.atPose3(0)
                T_rel = prev_pose.between(self.current_pose)
                self.graph.add(
                    gtsam.BetweenFactorPose3(0, 1, T_rel, self.odom_noise))
                self.initial_estimates.insert(1, self.current_pose)
            elif current_index > 1:
                prev_pose = self.initial_estimates.atPose3(current_index - 1)
                # Compute relative transform from previous keyframe to current keyframe.
                T_rel = prev_pose.between(self.current_pose)
                odom_factor = gtsam.BetweenFactorPose3(current_index - 1,
                                                       current_index,
                                                           T_rel,
                                                       self.odom_noise)
                self.graph.add(odom_factor)
                self.initial_estimates.insert(current_index, self.current_pose)
            temp = None
            if len(self.keyframes) % self.optimization_frequency == 0:

                # if current_index > 0:
                #     assert len(self.keyframes) - 1 == current_index
                #     prev_pose = self.keyframes[-2]['pose']
                #     # Compute relative transform from previous keyframe to current keyframe.
                #     T_rel = prev_pose.between(self.current_pose)
                #     T_rel = compute_loop_transformation(
                #         self.keyframes[-2]['point_cloud'], self.keyframes[-1]['point_cloud'],
                #         self.keyframes[-2]['pose'].matrix(), self.keyframes[-1]['pose'].matrix())
                #     odom_factor = gtsam.BetweenFactorPose3(current_index - 1, current_index, pose_from_matrix(T_rel), self.odom_noise)
                #     self.graph.add(odom_factor)

                for i, kf in enumerate(self.keyframes):
                    # Skip recent keyframes.
                    if current_index - i < self.MIN_KEYFRAME_GAP:
                        continue
                    sim = cosine_similarity(keyframe['descriptor'],
                                            kf['descriptor'])
                    rclpy.logging.get_logger("pose_graph").info(
                        f"cosine sim: {sim}")
                    if sim > self.loop_closure_similarity_threshold:
                        rclpy.logging.get_logger("pose_graph").info(
                            f"Loop Closure Detected: keyframe {len(self.keyframes)} and keyframe {i} with similarity {sim:.4f}")
                        # Found a loop closure candidate.
                        # delta = compute_loop_transformation(
                        #     kf['point_cloud'], keyframe['point_cloud'],
                        #     kf['pose'].matrix(), keyframe['pose'].matrix())
                        delta = kf['pose'].between(keyframe['pose'])
                        # delta = pose_from_matrix(delta)
                        if delta is not None:
                            # loop_pose = pose_from_matrix(delta)
                            loop_factor = gtsam.BetweenFactorPose3(i,
                                                                   current_index,
                                                                   delta,
                                                                   self.loop_noise)
                            self.graph.add(loop_factor)
                            cur_pose = kf['pose'].compose(delta)
                            # self.initial_estimates.update(current_index, cur_pose)
                            temp = (i, current_index)
                            rclpy.logging.get_logger("pose_graph").info(
                                f"Added loop closure between keyframe {i} and {current_index}")



                # Step 3: Add one prior to anchor the graph
                # prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
                #     np.array([1e-6] * 6))
                # self.graph.add(gtsam.PriorFactorPose3(0,
                #     self.keyframes[0]['pose'], prior_noise))

                # print(f"Running optimization on {current_index} keyframes...")
                params = gtsam.LevenbergMarquardtParams()
                optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph,
                                                              self.initial_estimates,
                                                              params)
                # rclpy.logging.get_logger("pose_graph").info(f"{self.keyframes[0]['pose']}")

                try:
                    rclpy.logging.get_logger("pose_graph").info(
                        f"STARTING OPTIMIZERRRR!!!!!!!!!!!!!!!!!!!!!!!")
                    time.sleep(.5)
                    result = optimizer.optimize()

                    # Update keyframe poses with optimized values.
                    for idx in range(len(self.keyframes)):
                        new_pose = result.atPose3(idx)
                        delta = new_pose.between(
                            self.keyframes[idx]['pose'])
                        # rclpy.logging.get_logger("pose_graph").info(
                        #     f"Pose {idx} delta after optimization: {np.round(delta, 4)}")
                        self.keyframes[idx]['pose'] = new_pose
                    self.initial_estimates = result  # Update the initial estimates to the optimized values
                    rclpy.logging.get_logger("pose_graph").info(
                        f"Optimizer COMPLETE!!!!!!!!!!!!!!!!!!!!!!!")
                    # if temp is not None:
                    #     self.on_optimization_complete([self.keyframes[temp[0]], self.keyframes[temp[1]]])
                    #     return
                    self.on_optimization_complete(self.keyframes)
                except Exception as e:
                    rclpy.logging.get_logger("pose_graph").error(
                        f"Error optimizing: {e}")
                    return

    # def optimization_loop(self):
    #     current_index = 0
    #
    #     while not self.stop_loop:
    #         try:
    #             keyframe = self.loop_closure_queue.get_nowait().copy()
    #         except queue.Empty:
    #             time.sleep(0.1)
    #             continue
    #
    #         raise RuntimeError("FUCKKKKK")
    #
    #         # Convert keyframe pose to GTSAM Pose3.
    #         keyframe_pose = keyframe['pose']
    #         pose_gtsam = pose_from_matrix(keyframe_pose)
    #
    #         # self.initial_estimates.insert(current_index, pose_gtsam)
    #
    #         # if current_index > 0:
    #         #     assert len(self.keyframes) - 1 == current_index
    #         #     prev_pose = self.keyframes[current_index - 1]['pose']
    #         #     # Compute relative transform from previous keyframe to current keyframe.
    #         #     T_rel = np.linalg.inv(prev_pose).dot(keyframe_pose)
    #         #     rel_pose = pose_from_matrix(T_rel)
    #         #     odom_factor = gtsam.BetweenFactorPose3(current_index - 1, current_index, rel_pose, self.odom_noise)
    #         #     self.graph.add(odom_factor)
    #
    #         # Check for loop closures with older keyframes.
    #         for i, kf in enumerate(self.keyframes):
    #             # Skip recent keyframes.
    #             if current_index - i < self.MIN_KEYFRAME_GAP:
    #                 continue
    #             sim = cosine_similarity(keyframe['descriptor'],
    #                                     kf['descriptor'])
    #             if sim > self.loop_closure_similarity_threshold:
    #                 # Found a loop closure candidate.
    #                 T_loop = compute_loop_transformation(
    #                     keyframe['point_cloud'], kf['point_cloud'], keyframe['pose'], kf['pose'])
    #                 if T_loop is not None:
    #                     loop_pose = pose_from_matrix(T_loop)
    #                     loop_factor = gtsam.BetweenFactorPose3(i, current_index,
    #                                                            loop_pose,
    #                                                            self.loop_noise)
    #                     self.graph.add(loop_factor)
    #                     rclpy.logging.get_logger("pose_graph").info(
    #                         f"Added loop closure between keyframe {i} and {current_index}")
    #
    #         current_index += 1
    #
    #         # Periodically run the optimizer.
    #         if current_index % self.optimization_frequency == 0:
    #             self.graph = gtsam.NonlinearFactorGraph()
    #             self.initial_estimates = gtsam.Values()
    #             prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([.1]*6))
    #
    #             pose0 = pose_from_matrix(self.keyframes[0]['pose'])
    #             self.initial_estimates.insert(0, pose0)
    #             self.graph.add(gtsam.PriorFactorPose3(0, pose0, prior_noise))
    #
    #             # # Add all the other keyframes to the graph.
    #             for idx, kf in enumerate(self.keyframes[1:], start=1):
    #                 # Insert the pose into the initial estimates.
    #                 self.initial_estimates.insert(idx,
    #                                               pose_from_matrix(kf['pose']))
    #                 rclpy.logging.get_logger("pose_graph").info(
    #                     f"{pose_from_matrix(kf['pose'])}")
    #
    #             # print(f"Running optimization on {current_index} keyframes...")
    #             optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph,
    #                                                           self.initial_estimates)
    #
    #
    #             result = optimizer.optimize()
    #             # Update keyframe poses with optimized values.
    #             for idx in range(len(self.keyframes)):
    #                 opt_pose = result.atPose3(idx)
    #                 # Update the keyframe's pose in your database.
    #                 self.keyframes[idx]['pose'] = np.array(opt_pose.matrix())
    #
    #             self.on_optimization_complete(self.keyframes)
    #
    #             # print("Optimization complete.")
    #
    #         # Sleep briefly to prevent busy waiting.
    #         time.sleep(0.1)


def compute_scan_context_descriptor(scan, num_angle_bins=60, num_radius_bins=20,
    max_range=80.0):
    """
    Computes a Scan Context descriptor for a point cloud.

    Args:
        scan (np.ndarray): Input point cloud of shape (N, 3) with columns [x, y, z].
        num_angle_bins (int): Number of bins for the azimuth angle.
        num_radius_bins (int): Number of bins for the radial distance.
        max_range (float): Maximum range to consider for binning.

    Returns:
        descriptor (np.ndarray): Flattened descriptor vector.
    """
    # Initialize the descriptor matrix with a very small value.
    descriptor = np.full((num_radius_bins, num_angle_bins), -np.inf)

    # Compute polar coordinates (rho, theta) for each point.
    xs = scan[:, 0]
    ys = scan[:, 1]
    zs = scan[:, 2]
    rho = np.sqrt(xs ** 2 + ys ** 2)
    theta = np.arctan2(ys, xs)  # range [-pi, pi]

    # Only consider points within the max_range.
    valid = rho < max_range
    rho = rho[valid]
    theta = theta[valid]
    zs = zs[valid]

    # Map angles from [-pi, pi] to [0, 2*pi]
    theta = theta + np.pi

    # Determine bin indices.
    angle_bin_indices = np.floor(theta / (2 * np.pi) * num_angle_bins).astype(
        np.int32)
    radius_bin_indices = np.floor(rho / max_range * num_radius_bins).astype(
        np.int32)

    # Clamp indices to valid range.
    angle_bin_indices = np.clip(angle_bin_indices, 0, num_angle_bins - 1)
    radius_bin_indices = np.clip(radius_bin_indices, 0, num_radius_bins - 1)

    # Populate the descriptor matrix: use maximum z in each bin.
    for r_bin, a_bin, z in zip(radius_bin_indices, angle_bin_indices, zs):
        # Update the bin if this z is higher than the current stored value.
        if z > descriptor[r_bin, a_bin]:
            descriptor[r_bin, a_bin] = z

    # Replace -inf values with 0 (bins that received no points).
    descriptor[descriptor == -np.inf] = 0

    # Optionally, flatten and L2-normalize the descriptor.
    flat_descriptor = descriptor.flatten()
    norm = np.linalg.norm(flat_descriptor) + 1e-8
    flat_descriptor /= norm

    return flat_descriptor

def transform_point_cloud(pc: np.ndarray, T: np.ndarray) -> np.ndarray:
    """
    Transform a point cloud using a 4x4 transformation matrix.

    :param pc: The point cloud to transform
    :param T: The 4x4 transformation matrix

    :return: The transformed point cloud
    """
    # Add a row of [0, 0, 0, 1] to the point cloud to make it homogeneous.
    pc_h = np.hstack((pc, np.ones((pc.shape[0], 1))))
    # Transform the point cloud using the transformation matrix.
    pc_transformed_h = np.dot(T, pc_h.T).T
    # Remove the homogeneous coordinate and return the transformed point cloud.
    return pc_transformed_h[:, :3]