import copy
import multiprocessing
import queue
import random
from collections import defaultdict
from typing import Callable, Optional
import open3d as o3d

import numpy as np
import gtsam
import rclpy.logging
import traceback
from sklearn.metrics.pairwise import cosine_similarity

from scripts.config import SLAMConfig


def _show(pcs, ids):
    o3d_pcs = []
    for pc in pcs:
        pc_o3d = o3d.geometry.PointCloud()
        pc_o3d.points = o3d.utility.Vector3dVector(pc)
        pc_o3d.paint_uniform_color([random.random(), random.random(), random.random()])
        o3d_pcs.append(pc_o3d)
    rclpy.logging.get_logger("pose_graph").info(f"Showing {len(o3d_pcs)} point clouds")
    o3d.visualization.draw_geometries(o3d_pcs, window_name=f"Loop closure {ids}",)

from scripts.pointcloud_processors.descriptor_generators.ndt_transformer import \
    NDTTransformer

from scripts.pointcloud_processors.utils.gtsam_utils import Submap, \
    pose_from_matrix
from scripts.pointcloud_processors.utils.open3d_utils import \
    compute_icp_transformation, compute_multiscale_icp_transformation


class PoseGraphGTSAMICP:

    def __init__(self,
        on_optimization_complete,
        config: SLAMConfig,
        trans_thresh: float = .5,
        rot_thresh_deg: float = 7.5,
        loop_closure_similarity_threshold: float = .97,
        optimization_frequency: int = 2,
        min_keyframe_gap: int = 5,
    ):
        """
        :param trans_thresh: Translation threshold for adding keyframe, in meters
        :param rot_thresh_deg: Rotation threshold for adding keyframe, in degrees
        """
        self.config = config
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
        odom_sigmas = np.array([0.3, 0.3, 0.3, 0.15, 0.15, 0.15])
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(odom_sigmas)
        # self.odom_noise = gtsam.noiseModel.Diagonal.Variances(
        #     [0.02, 0.02, 0.02, 0.2, 0.2, 0.2])
        loop_noise = gtsam.noiseModel.Diagonal.Variances(
            [0.05, 0.05, 0.05, 0.5, 0.5, 0.5])
        loop_sigmas = np.array([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        base = gtsam.noiseModel.Diagonal.Variances(loop_sigmas)
        # self.loop_noise = base
        self.loop_noise = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber(1.345),
            base
        )

        self.loop_closure_similarity_threshold = loop_closure_similarity_threshold
        self.optimization_frequency = optimization_frequency
        self.MIN_KEYFRAME_GAP = min_keyframe_gap

        self.optimized_global_map = None
        self.submap = None
        self.loop_closures = defaultdict(list)

    def stop(self):
        self.stop_loop = True
        rclpy.logging.get_logger("pose_graph").debug("Optimizer thread stopped")

    def reset(self):
        self.prev_keyframe_pose = np.eye(4)
        self.keyframes = []
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.stop_loop = False
        self.submap = None

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

    def should_add_keyframe1(self, current_submap):
        voxels = len(copy.deepcopy(current_submap.points).voxel_down_sample(0.02))
        return voxels > 75000

    def update_pose_graph(self, trans: np.ndarray, scan_pc: o3d.geometry.PointCloud, gen_descriptor: Callable[[np.ndarray], np.ndarray]) -> None:
        """
        Do stfuf

        :param trans: The recent transformation computation to transform scan_pc to global map frame
        :param scan_pc: The most recent point cloud scan
        :param gen_descriptor: The function to generate the keyframe descriptor

        :return:
        """
        global_scan_pc = scan_pc.transform(trans)
        if self.submap is None:
            self.submap = Submap(pose=trans, point_cloud=global_scan_pc)
        else:
            self.submap += global_scan_pc

        # if not self.should_add_keyframe(trans, self.submap.matrix):
        #     return
        if not self.submap.is_submap_complete(self.config.pose_graph.voxel_size, self.config.pose_graph.point_thresh):
            return
        # rclpy.logging.get_logger("pose_graph").info(f"points in submap {len(self.keyframes)}: {len(self.submap.point_cloud.voxel_down_sample(0.02).points)}")
        current_index = len(self.keyframes)
        self.submap.outlier_rejection()
        descriptor = gen_descriptor(self.submap.points)

        keyframe = {
            'pose': self.submap.pose,
            'descriptor': descriptor,
            'point_cloud': copy.deepcopy(self.submap.point_cloud),
        }
        self.submap = None

        self.keyframes.append(keyframe)
        # return

        rclpy.logging.get_logger("pose_graph").info("Adding keyframe, total keyframes: " + str(len(self.keyframes)))

        if current_index == 0:
            # For the first keyframe, initialize the graph with a prior factor.
            prior_mean = keyframe['pose']
            # prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.03, 0.03, 0.03, 0.015, 0.015, 0.015]))#[0.005, 0.005, 0.005, .0025, .0025, .0025]))
            prior_noise = gtsam.noiseModel.Constrained.All(6)
            # self.graph.add(
            #     gtsam.PriorFactorPose3(0, prior_mean, prior_noise))
            self.initial_estimates.insert(0, prior_mean)
            self.graph.add(
                gtsam.PriorFactorPose3(0, self.initial_estimates.atPose3(0),
                                 prior_noise)
            )

        elif current_index >= 1:
            self.add_keyframe(keyframe, current_index)
            # if current_index % int(self.optimization_frequency / 2) == 0 and current_index > 0:
            #     self.detect_loop_closure(keyframe, current_index)
            if current_index % self.optimization_frequency == 0 and current_index > 0:
                self.detect_loop_closure(keyframe, current_index)
                self.run_optimizer()

    def add_keyframe(self, keyframe, current_index) -> None:
        """
        Add a new keyframe to the graph and add an odometry factor.

        :param keyframe: The current keyframe to add, made up of multiple point clouds,
            a pose, which is the point cloud registration result for the first scan,
            and a descriptor for the keyframe.
        :param current_index: The index of the current keyframe
        """
        prev_pose = self.initial_estimates.atPose3(current_index - 1)
        # Compute relative transform from previous keyframe to current keyframe.
        T_rel = prev_pose.between(keyframe['pose'])
        odom_factor = gtsam.BetweenFactorPose3(current_index - 1,
                                               current_index,
                                                   T_rel,
                                               self.odom_noise)
        self.graph.add(odom_factor)
        self.initial_estimates.insert(current_index, keyframe['pose'])


    def detect_loop_closure(self, keyframe, current_index) -> None:
        """
        Detect loop closures by comparing the current keyframe with previous keyframes.
        If the descriptor similarity is above a certain threshold, add a loop closure factor.

        :param keyframe: The current keyframe to compare with previous keyframes
        :param current_index: The index of the current keyframe
        """
        # collect (index, sim) pairs
        candidates = []
        for i, kf in enumerate(self.keyframes):
            # skip too‐recent keyframes
            if current_index - i < self.MIN_KEYFRAME_GAP:
                continue

            sim = cosine_similarity(
                keyframe['descriptor'].reshape(1, -1),
                kf['descriptor'].reshape(1, -1)
            )[0, 0]
            rclpy.logging.get_logger("pose_graph").info(
                f"cosine sim {i},{current_index}: {sim:.4f}"
            )

            # enforce your similarity threshold and max‐per‐node caps
            if (sim > self.loop_closure_similarity_threshold and
                len(self.loop_closures[i]) < 4 and
                len(self.loop_closures[current_index]) < 4):
                candidates.append((i, sim))

        # sort by descending similarity and pick top 4
        top4 = sorted(candidates, key=lambda x: -x[1])[:4]

        # add the loop factors
        for i, sim in top4:
            rclpy.logging.get_logger("pose_graph").info(
                f"Adding loop closure for keyframes {i} ↔ {current_index} (sim={sim:.4f})"
            )
            self.add_loop_closure_factor(i, current_index)

    def run_optimizer(self) -> None:
        """
        Run the GTSAM optimizer to optimize the pose graph.
        """
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosity("ERROR")
        # dog_legg_params = gtsam.DoglegParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph,
                                                      self.initial_estimates,
                                                      params)
        try:
            rclpy.logging.get_logger("pose_graph").info(
                f"STARTING OPTIMIZERRRR!!!!!!!!!!!!!!!!!!!!!!!")
            result = optimizer.optimizeSafely()

            # Update keyframe poses with optimized values.
            for idx in range(len(self.keyframes)):
                new_pose = result.atPose3(idx)
                delta = self.keyframes[idx]['pose'].between(new_pose)
                self.keyframes[idx]['point_cloud'] = self.keyframes[idx]['point_cloud'].transform(self.keyframes[idx]['pose'].inverse().matrix()).transform(new_pose.matrix())#delta.matrix())
                self.keyframes[idx]['pose'] = new_pose
                self.initial_estimates.update(idx, new_pose)
            self.on_optimization_complete(self.keyframes)

        except Exception as e:
            rclpy.logging.get_logger("pose_graph").error(
                f"Error optimizing: {e}")
            traceback.print_exc()
            return


    def add_loop_closure_factor(self, prev_index, current_index):
        """
        Add a loop closure factor to the graph.

        :param prev_index: The index of the previous keyframe
        :param current_index: The index of the current keyframe
        """

        prev_kf = self.keyframes[prev_index]
        keyframe = self.keyframes[current_index]

        # Both pointclouds are already in world frame
        prev_pc = prev_kf['point_cloud']
        cur_pc = keyframe['point_cloud']

        # When doing ICP between pointclouds already in the world frame,
        # you're calculating world→world transformation
        # This isn't what you want for the between factor

        # SOLUTION: Transform pointclouds to their respective local frames first
        # Then run ICP to find the relative transformation

        # Convert pointclouds to local frame
        prev_pc_local = copy.deepcopy(prev_pc)
        prev_pc_local.transform(np.linalg.inv(prev_kf['pose'].matrix()))

        cur_pc_local = copy.deepcopy(cur_pc)
        cur_pc_local.transform(np.linalg.inv(keyframe['pose'].matrix()))

        # Now run ICP between local frames
        init = np.eye(
            4)  # Identity is a good starting point for local→local
        # Calculate odometry chain between prev_index and current_index
        prev_pose_matrix = prev_kf['pose'].matrix()
        curr_pose_matrix = keyframe['pose'].matrix()

        # Initial transform from prev_local to curr_local
        init_transform = prev_pose_matrix @ np.linalg.inv(curr_pose_matrix)
        init = init_transform  # Use this as ICP initialization
        p = multiprocessing.Process(target=_show, args=(
            [np.asarray(copy.deepcopy(prev_pc_local).transform(init).points),
             np.asarray(cur_pc_local.points)],[prev_index, current_index],))
        p.daemon = True
        p.start()
        result = compute_multiscale_icp_transformation(
            prev_pc_local,
            cur_pc_local,
            init,
            logger=rclpy.logging.get_logger("loop_closure").info,
            max_iterations=1000,
        )
        init = np.eye(4)#prev_kf['pose'].between(keyframe['pose']).matrix()#np.linalg.inv(T_prev) @ T_cur#np.linalg.inv(T_prev) @ T_cur#np.eye(4)#
        p = multiprocessing.Process(target=_show, args=(
            [np.asarray(copy.deepcopy(prev_pc_local).transform(result.transformation).points),
             np.asarray(cur_pc_local.points)],[prev_index, current_index],))
        p.daemon = True
        p.start()


        # The ICP result is now the correct relative transformation
        # from prev_local → cur_local frame
        if result.fitness > 0.4:
            # Convert matrix to gtsam Pose3
            relative_pose = pose_from_matrix(result.transformation)
            odom_chain = self.initial_estimates.atPose3(prev_index).between(
                self.initial_estimates.atPose3(current_index))

            # Check if the difference is too large
            delta = odom_chain.between(relative_pose)
            # Handle translation difference
            trans_diff = np.linalg.norm(delta.translation())

            # For rotation, get rotation matrix and convert to axis-angle
            R = delta.rotation().matrix()
            # Calculate angle from rotation matrix using arccos((trace(R)-1)/2)
            angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
            rot_diff = np.degrees(angle)
            rclpy.logging.get_logger("pose_graph").info(
                f"rot diff: {rot_diff}, trans diff: {trans_diff}"
            )

            # Add this directly as the between factor
            loop_factor = gtsam.BetweenFactorPose3(
                prev_index,
                current_index,
                relative_pose.inverse(),
                self.loop_noise
            )
            self.graph.add(loop_factor)

            # For debugging
            rclpy.logging.get_logger("pose_graph").info(
                f"Added loop closure between {prev_index} and {current_index} with fitness {result.fitness}")
        # prev_kf = self.keyframes[prev_index]
        # keyframe = self.keyframes[current_index]
        # prev_kf_pc = copy.deepcopy(prev_kf['point_cloud'])#.transform(prev_kf['pose'].inverse().matrix())
        # keyframe_pc = copy.deepcopy(keyframe['point_cloud'])#.transform(keyframe['pose'].inverse().matrix())
        #
        # # 1) get the raw Pose3→numpy→4×4
        # T_prev = prev_kf['pose'].matrix()  # world ← prev
        # T_cur = keyframe['pose'].matrix()  # world ← cur
        # prev_pc = prev_kf['point_cloud']  # already in world
        # cur_pc = keyframe['point_cloud']  # already in world
        # # init = T_cur @ np.linalg.inv(T_prev)  # world→world guess
        # # 2) compute the relative‐pose guess
        # #    world⁻¹ × world  = prev⁻¹ × cur
        # init = np.eye(4)#prev_kf['pose'].between(keyframe['pose']).matrix()#np.linalg.inv(T_prev) @ T_cur#np.linalg.inv(T_prev) @ T_cur#np.eye(4)#
        # # p = multiprocessing.Process(target=_show, args=(
        # #     [np.asarray(copy.deepcopy(prev_kf_pc).transform(init).points),
        # #      np.asarray(keyframe_pc.points)],[prev_index, current_index],))
        # # p.daemon = True
        # # p.start()
        # # Found a loop closure candidate.
        # result = compute_multiscale_icp_transformation(
        #     prev_pc,
        #     cur_pc,
        #     init,
        #     logger=rclpy.logging.get_logger("loop_closure").info,
        #     max_iterations=1000,
        # )#prev_kf['pose'].between(keyframe['pose']).matrix())
        #
        # icp_t = pose_from_matrix(result.transformation)  # world→world
        #
        # # new_pose_est = prev_kf['pose'].compose(delta)
        # # delta = prev_kf['pose'].between(new_pose_est)
        # delta = prev_kf['pose'].inverse().compose(icp_t).compose(prev_kf['pose'])
        #
        # # rclpy.logging.get_logger("pose_graph").info(f"{new_pose_est}")
        # rclpy.logging.get_logger("delta").info(f"{delta}")
        # rclpy.logging.get_logger("prev_kf").info(f"{prev_kf['pose']}")
        # rclpy.logging.get_logger("between og").info(f"{prev_kf['pose'].between(keyframe['pose'])}")
        # rclpy.logging.get_logger("cur pose").info(f"{keyframe['pose']}")
        #
        #
        # # convert to BetweenFactor: conjugate into prev’s frame
        # # delta = prev_kf['pose'].inverse().compose(delta).compose(prev_kf['pose'])
        # # p = multiprocessing.Process(target=_show, args=(
        # #     [np.asarray(prev_kf_pc.transform(delta.matrix()).points),
        # #      np.asarray(keyframe_pc.points)],[i, current_index],))
        # # p.daemon = True
        # # p.start()
        #
        #
        # if delta is not None and result.fitness > 0.4:
        #     ## MATH
        #     # icp_T @ prev_T = new_prev_T
        #     # icp_t = global_prev_pc -> global_cur_pc
        #     # prev_T = world_frame -> global_prev_pose
        #     # WANT: T_prev_current = prev_T ^ -1 @ cur_T
        #     # loop_factor = gtsam.BetweenFactorPose3(
        #     #                                        current_index,
        #     #     prev_index,
        #     #                                        delta.inverse(),
        #     #                                        self.loop_noise)
        #     loop_factor_inv = gtsam.BetweenFactorPose3(prev_index,
        #                                            current_index,
        #                                            delta,
        #                                            self.loop_noise)
        #     # self.graph.add(loop_factor)
        #     self.graph.add(loop_factor_inv)
        #     cur_pose = prev_kf['pose'].compose(delta)
        #     # self.initial_estimates.update(current_index, cur_pose)
        #     self.loop_closures[prev_index].append(current_index)
        #     self.loop_closures[current_index].append(prev_index)
        #     rclpy.logging.get_logger("pose_graph").info(
        #         f"Added loop closure between keyframe {prev_index} and {current_index}")