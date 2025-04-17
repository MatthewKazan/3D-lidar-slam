import multiprocessing
import queue
import threading
import time
import random
from typing import Callable, Optional
import open3d as o3d

import numpy as np
import gtsam
import rclpy.logging
from gtsam import Pose3, Rot3
import traceback
from sklearn.metrics.pairwise import cosine_similarity

from scripts.point_cloud_processors.ndt_transformer import \
    NDTTransformer

from scripts.point_cloud_processors.utils.gtsam_utils import Submap, \
    pose_from_matrix
from scripts.point_cloud_processors.utils.open3d_utils import \
    compute_icp_transformation

def _show(pcs):
    o3d_pcs = []
    for pc in pcs:
        pc_o3d = o3d.geometry.PointCloud()
        pc_o3d.points = o3d.utility.Vector3dVector(pc)
        pc_o3d.paint_uniform_color([random.random(), random.random(), random.random()])
        o3d_pcs.append(pc_o3d)
    rclpy.logging.get_logger("pose_graph").info(f"Showing {len(o3d_pcs)} point clouds")
    o3d.visualization.draw_geometries(o3d_pcs)


class PoseGraphGTSAMICP:

    def __init__(self,
        on_optimization_complete,
        trans_thresh: float = 0.5,
        rot_thresh_deg: float = 7,
        loop_closure_similarity_threshold: float = .95,
        optimization_frequency: int = 5,
        min_keyframe_gap: int = 20,
    ):
        """
        :param trans_thresh: Translation threshold for adding keyframe, in meters
        :param rot_thresh_deg: Rotation threshold for adding keyframe, in degrees
        """
        self.prev_keyframe_pose = np.eye(4)
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
            [0.3, 0.3, 0.3, 0.1, 0.1, 0.1])  # in meters and radians
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(odom_sigmas)
        loop_sigmas = np.array([.02, .02, .02, .01, .01, .01])  # in meters and radians
        # self.loop_noise = gtsam.noiseModel.Diagonal.Sigmas(loop_sigmas)
        base = gtsam.noiseModel.Diagonal.Sigmas(loop_sigmas)
        self.loop_noise = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber(1.345), base
        )
        self.loop_closure_similarity_threshold = loop_closure_similarity_threshold
        self.optimization_frequency = optimization_frequency
        self.MIN_KEYFRAME_GAP = min_keyframe_gap

        self.optimized_global_map = None
        self.submap = None
        self.loop_closures = set()

    def stop(self):
        self.stop_loop = True
        rclpy.logging.get_logger("pose_graph").debug("Optimizer thread stopped")

    def reset(self):
        self.prev_keyframe_pose = np.eye(4)
        self.keyframes = []
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.stop_loop = False
        self.submap = []

    def should_add_keyframe(self, current_pose: np.array, last_keyframe_pose: np.array) -> bool:
        """
        Determine whether to add a new keyframe based on the current pose.

        :return: Boolean indicating whether to add a new keyframe
        """
        # Compute the translation and rotation differences between the current and last keyframes.

        # Extract translations (last column of the matrix)
        t_current = current_pose[:3, 3]
        t_last = last_keyframe_pose[:3, 3]
        translation_diff = np.linalg.norm(t_current - t_last)
        # Compute rotation difference:
        # Use the relative rotation matrix R_diff = R_last^T * R_current.
        R_current = current_pose[:3, :3]
        R_last = last_keyframe_pose[:3, :3]
        R_diff = R_last.T @ R_current

        # Compute the rotation angle from the trace of R_diff.
        angle_rad = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)

        # If either difference exceeds the threshold, we add a new keyframe.
        return translation_diff > self.trans_thresh or angle_deg > self.rot_thresh_deg

    def update_pose_graph(self, trans: np.ndarray, scan_pc: o3d.geometry.PointCloud, gen_descriptor: Callable[[np.ndarray, Optional[int]], np.ndarray]) -> None:
        """
        Do stfuf

        :param trans: The recent transformation computation to transform scan_pc to global map frame
        :param scan_pc: The most recent point cloud scan
        :param gen_descriptor: The function to generate the keyframe descriptor

        :return:
        """

        if self.submap is None:
            self.submap = Submap(pose=trans, point_clouds=[scan_pc.transform(trans)])
        else:
            self.submap += scan_pc.transform(trans)

        if not self.should_add_keyframe(trans, self.submap.matrix):
            return
        current_index = len(self.keyframes)

        descriptor = gen_descriptor(self.submap.points, 20)

        keyframe = {
            'pose': self.submap.pose,
            'descriptor': descriptor,
            'point_cloud': self.submap.point_cloud,
        }
        self.submap = None

        self.keyframes.append(keyframe)
        rclpy.logging.get_logger("pose_graph").info("Adding keyframe, total keyframes: " + str(len(self.keyframes)))

        if current_index == 0:
            # For the first keyframe, initialize the graph with a prior factor.
            prior_mean = gtsam.Pose3()

            self.graph.add(
                gtsam.PriorFactorPose3(0, prior_mean,gtsam.noiseModel.Constrained.All(6)))
            self.initial_estimates.insert(0, prior_mean)

        elif current_index >= 1:

            prev_pose = self.initial_estimates.atPose3(current_index - 1)
            # Compute relative transform from previous keyframe to current keyframe.
            T_rel = prev_pose.between(keyframe['pose'])
            odom_factor = gtsam.BetweenFactorPose3(current_index - 1,
                                                   current_index,
                                                       T_rel,
                                                   self.odom_noise)
            self.graph.add(odom_factor)
            self.initial_estimates.insert(current_index, keyframe['pose'])

            loop_closures = []
            for i, kf in enumerate(self.keyframes):
                # Skip recent keyframes.
                if current_index - i < self.MIN_KEYFRAME_GAP:
                    continue
                sim = cosine_similarity(keyframe['descriptor'], kf['descriptor'])
                rclpy.logging.get_logger("pose_graph").info(
                    f"cosine sim {i},{current_index}: {sim}")
                if sim > self.loop_closure_similarity_threshold:
                    rclpy.logging.get_logger("pose_graph").info(
                        f"Loop Closure Detected: keyframe {current_index} and keyframe {i} with similarity {sim}")
                    # self.add_loop_closure_factor(i, current_index)
                    loop_closures.append((i, current_index))
            # Add loop closure factors to the graph.
            if len(loop_closures) > 4:
                loop_closures = random.sample(loop_closures, 4)
            for i, current_index in loop_closures:
                self.add_loop_closure_factor(i, current_index)

        if current_index % self.optimization_frequency == 0 and current_index > 0:
            params = gtsam.LevenbergMarquardtParams()
            dog_legg_params = gtsam.DoglegParams()
            optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph,
                                                          self.initial_estimates,
                                                          params)
            try:
                rclpy.logging.get_logger("pose_graph").info(
                    f"STARTING OPTIMIZERRRR!!!!!!!!!!!!!!!!!!!!!!!")
                result = optimizer.optimize()

                # Update keyframe poses with optimized values.
                for idx in range(1, len(self.keyframes)):
                    new_pose = result.atPose3(idx)
                    delta = self.keyframes[idx]['pose'].between(new_pose)
                    self.keyframes[idx]['point_cloud'] = self.keyframes[idx]['point_cloud'].transform(delta.matrix())
                    self.keyframes[idx]['pose'] = new_pose
                    self.initial_estimates.update(idx, new_pose)
                self.on_optimization_complete(self.keyframes)

            except Exception as e:
                rclpy.logging.get_logger("pose_graph").error(
                    f"Error optimizing: {e}")
                traceback.print_exc()
                return


    def add_loop_closure_factor(self, i, current_index):
        """
        Add a loop closure factor to the graph.

        :param i: The index of the previous keyframe
        :param current_index: The index of the current keyframe
        """
        prev_kf = self.keyframes[i]
        keyframe = self.keyframes[current_index]
        # Found a loop closure candidate.
        t_icp = compute_icp_transformation(
            prev_kf['point_cloud'], keyframe['point_cloud'],
            np.eye(4), logger=rclpy.logging.get_logger("loop_closure").info)#prev_kf['pose'].between(keyframe['pose']).matrix())
        t_new_prev = t_icp @ prev_kf['pose'].matrix()
        t_prev_cur = np.linalg.inv(t_new_prev) @ keyframe['pose'].matrix()

        delta = pose_from_matrix(t_prev_cur)

        # p = multiprocessing.Process(target=_show, args=(
        #     [np.asarray(prev_kf['point_cloud'].transform(t_icp).points),
        #      np.asarray(keyframe['point_cloud'].points)],))
        # p.daemon = True
        # p.start()

        if delta is not None:
            ## MATH
            # icp_T @ prev_T = new_prev_T
            # WANT: T_prev_current = prev_T ^ -1 @ cur_T
            loop_factor = gtsam.BetweenFactorPose3(i,
                                                   current_index,
                                                   delta,
                                                   self.loop_noise)
            self.graph.add(loop_factor)
            cur_pose = prev_kf['pose'].compose(delta)
            self.initial_estimates.update(current_index, cur_pose)
            self.loop_closures.add(current_index)
            self.loop_closures.add(i)
            rclpy.logging.get_logger("pose_graph").info(
                f"Added loop closure between keyframe {i} and {current_index}")