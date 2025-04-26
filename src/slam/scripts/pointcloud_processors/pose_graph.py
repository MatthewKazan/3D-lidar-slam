import copy
import queue
import random
from collections import defaultdict
from typing import Callable
import open3d as o3d

import numpy as np
import gtsam
import rclpy.logging
import traceback
from sklearn.metrics.pairwise import cosine_similarity

from scripts.config import SLAMConfig

from scripts.data_transfer import DataTransfer


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
        data_transfer: DataTransfer,
    ):
        """
        Initialize the pose graph

        :param config: The configuration object containing the parameters for the pose graph
        :param data_transfer: The data transfer object for thread-safe communication
        :param on_optimization_complete: Callback function to be called when optimization is complete,
            exists to update the global map in pointcloud registration module
        """
        self.config = config
        self.data_transfer = data_transfer
        self.prev_keyframe_pose = np.eye(4)
        self.keyframes = []
        self.on_optimization_complete = on_optimization_complete

        self.stop_loop = False
        self.loop_closure_queue = queue.Queue()

        # Initialize GTSAM graph and initial estimates.
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        odom_sigmas = np.array([0.3, 0.3, 0.3, 0.15, 0.15, 0.15])
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(odom_sigmas)

        loop_sigmas = np.array([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        base = gtsam.noiseModel.Diagonal.Variances(loop_sigmas)
        self.loop_noise = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber(1.345),
            base
        )

        self.submap = None
        self.loop_closures = defaultdict(list)


    def reset(self):
        self.prev_keyframe_pose = np.eye(4)
        self.keyframes = []
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.stop_loop = False
        self.submap = None
        self.loop_closures = defaultdict(list)

    def update_pose_graph(self, trans: np.ndarray, scan_pc: o3d.geometry.PointCloud, gen_descriptor: Callable[[np.ndarray], np.ndarray]) -> None:
        """
        update the pose graph with a new keyframe if the keyframe is large enough.
        otherwise, just add the point cloud to the keyframe.

        :param trans: The recent transformation computation to transform scan_pc to global map frame
        :param scan_pc: The most recent point cloud scan
        :param gen_descriptor: The function to generate the keyframe descriptor
        """
        global_scan_pc = scan_pc.transform(trans)
        if self.submap is None:
            self.submap = Submap(pose=trans, point_cloud=global_scan_pc)
        else:
            self.submap += global_scan_pc

        if not self.submap.is_submap_complete(self.config.pose_graph.voxel_size, self.config.pose_graph.point_thresh):
            return

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

        rclpy.logging.get_logger("pose_graph").info("Adding keyframe, total keyframes: " + str(len(self.keyframes)))

        if current_index == 0:
            # For the first keyframe, initialize the graph with a prior factor.
            prior_mean = keyframe['pose']

            prior_noise = gtsam.noiseModel.Constrained.All(6)

            self.initial_estimates.insert(0, prior_mean)
            self.graph.add(
                gtsam.PriorFactorPose3(0, self.initial_estimates.atPose3(0),
                                 prior_noise)
            )

        elif current_index >= 1:
            self.add_keyframe(keyframe, current_index)
            if current_index % self.config.pose_graph.optimization_frequency == 0 and current_index > 0:
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
            if current_index - i < self.config.pose_graph.min_keyframe_gap:
                continue

            if self.data_transfer.stop_event.is_set():
                raise KeyboardInterrupt("Stopping loop closure processing")

            sim = cosine_similarity(
                keyframe['descriptor'].reshape(1, -1),
                kf['descriptor'].reshape(1, -1)
            )[0, 0]
            rclpy.logging.get_logger("pose_graph").info(
                f"cosine sim {i},{current_index}: {sim:.4f}"
            )

            # enforce your similarity threshold and max‐per‐node caps
            if (sim > self.config.pose_graph.loop_closure_similarity_threshold and
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
        params.setVerbosity(self.config.pose_graph.optimizer_verbosity)
        # dog_legg_params = gtsam.DoglegParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph,
                                                      self.initial_estimates,
                                                      params)
        try:
            rclpy.logging.get_logger("pose_graph").debug(
                f"STARTING OPTIMIZERRRR!!!!!!!!!!!!!!!!!!!!!!!")
            result = optimizer.optimizeSafely()

            # Update keyframe poses with optimized values.
            for idx in range(len(self.keyframes)):
                new_pose = result.atPose3(idx)
                # made this long and ugly because it makes the steps more clear
                self.keyframes[idx]['point_cloud'] = (
                    self.keyframes[idx]['point_cloud'] # world frame
                    .transform(self.keyframes[idx]['pose'].inverse().matrix()) # transform to local frame
                    .transform(new_pose.matrix())) # transform to new estimated pose in world frame

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
        This is a bad function but im too scared to change it.

        :param prev_index: The index of the previous keyframe
        :param current_index: The index of the current keyframe
        """

        prev_kf = self.keyframes[prev_index]
        keyframe = self.keyframes[current_index]

        # Both pointclouds are already in world frame
        prev_pc = prev_kf['point_cloud']
        cur_pc = keyframe['point_cloud']

        # Convert pointclouds to local frame
        prev_pc_local = copy.deepcopy(prev_pc)
        prev_pc_local.transform(np.linalg.inv(prev_kf['pose'].matrix()))

        cur_pc_local = copy.deepcopy(cur_pc)
        cur_pc_local.transform(np.linalg.inv(keyframe['pose'].matrix()))

        # Calculate odometry chain between prev_index and current_index
        prev_pose_matrix = prev_kf['pose'].matrix()
        curr_pose_matrix = keyframe['pose'].matrix()

        # Initial transform from prev_local to curr_local
        init_transform = prev_pose_matrix @ np.linalg.inv(curr_pose_matrix)

        result = compute_multiscale_icp_transformation(
            prev_pc_local,
            cur_pc_local,
            init_transform,
            logger=rclpy.logging.get_logger("loop_closure").info,
            max_iterations=1000,
        )

        if result.fitness > 0.4:
            # Convert matrix to gtsam Pose3
            relative_pose = pose_from_matrix(result.transformation)

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
