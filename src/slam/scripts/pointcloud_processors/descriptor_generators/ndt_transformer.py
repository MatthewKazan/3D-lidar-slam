import sys
from collections import defaultdict

import numpy as np
import rclpy.logging

from scripts.paths import PATH_TO_BUILD_DGR, PATH_TO_BUILD_MINK, PATH_TO_BUILD_NDT
import torch
import torch.nn as nn


from scripts.pointcloud_processors.descriptor_generators.generic_descriptor_generator import \
    GenericDescriptorGenerator

from scripts.config import SLAMConfig

from scripts.pointcloud_processors.utils.open3d_utils import \
    downsample_to_target

device = (
    torch.device("cuda") if torch.cuda.is_available()
    else torch.device("mps") if torch.backends.mps.is_available()
    else torch.device("cpu")
)

sys.path.append(PATH_TO_BUILD_NDT)  # Ensure the path is in Python's search

import libs.NDT_Transformer.models.NDTNetVlad as PNV
import libs.NDT_Transformer.config as cfg


class NDTTransformer(GenericDescriptorGenerator):
    """
    A class to process and store point clouds. Gets raw point cloud data from
    the database, converts it from pixels to meters and stores it in a global map.
    """

    def __init__(self, config: SLAMConfig):
        """

        """
        super().__init__(rclpy.logging.get_logger("ndt_transformer"))
        self.model = PNV.NDTNetVlad(num_points=cfg.NUM_POINTS,
                               output_dim=cfg.FEATURE_OUTPUT_DIM,
                               emb_dims=cfg.EMB_DIMS,
                               layer_number=cfg.LAYER_NUMBER)
        model_path = config.ndt_weights_path
        self.model = self.model.to(device)

        resume_filename = model_path
        checkpoint = torch.load(resume_filename, map_location=device)
        missing, unexpected = self.model.load_state_dict(checkpoint['state_dict'],
                                                    strict=False)
        self.logger.debug(f"Missing keys: {missing}")
        self.logger.debug(f"Unexpected keys: {unexpected}")
        self.model = nn.DataParallel(self.model)
        self.previous_points = None
        self.scaling_factor = 20

    def generate_descriptor(self, points: np.array) -> None:
        """
        Align the new point cloud with the global map using ICP and add it to the map.

        :param points: The new point cloud to add to the global map

        :return: The new point cloud transformed to align with the global map
        """
        self.model.eval()

        points_tensor = reformat_ndt_input(points, self.scaling_factor, self.logger)
        with torch.no_grad():
            out = self.model(points_tensor)

        normal_out = out.detach().cpu().numpy()

        self.logger.debug(
            f"output point cloud of shape {normal_out.shape}")

        self.model.train()

        return normal_out

def reformat_ndt_input(points: np.array, scaling_factor: float, logger, voxel_size: float = 0.1) -> torch.Tensor:
    """
    Reformat the input point cloud to be compatible with the model.

    :param points: The point cloud to reformat
    :param scaling_factor: The scaling factor to apply to the point cloud
        This helps NDT transformer since it is trained on a different scale
    :param voxel_size: The voxel size to apply to the point cloud

    :return: The reformatted point cloud to fit model input, must be [1, 1, 2000, 12]
        and the points must be passed through ndt_voxelize
    """
    logger.debug(
        f"Reformatting input point cloud of shape {points.shape}")
    points = points * scaling_factor
    points = ndt_voxelize(points, voxel_size=voxel_size * scaling_factor)

    feed_tensor = torch.from_numpy(points).float()  # [N, 12]
    feed_tensor = feed_tensor.unsqueeze(0).unsqueeze(1).to(
        device)  # → [1, 1, N, 12]

    logger.debug(
        f"Reformatting input point cloud of shape {feed_tensor.size()}")
    return feed_tensor


def ndt_voxelize(points, voxel_size=.1, num_voxels=2000, min_points_per_voxel=5):
    """
    Voxelize the point cloud and compute features for each voxel. This is necessary
    as per the NDT transformer paper and is not done by the model.

    :param points: Point cloud to voxelize
    :param voxel_size: The size of the voxel grid
    :param num_voxels: Number of points to output, model expects 2000,
        need to change source code to modify this
    :param min_points_per_voxel: The minimum number of points per voxel to compute features

    :return: The voxelized point cloud with features
    """
    voxel_grid = defaultdict(list)
    voxel_size, _ = downsample_to_target(points, 2000, 30, 20)
    # 1. Assign points to voxel grid cells
    voxel_indices = np.floor(points / voxel_size).astype(int)
    for idx, voxel_idx in enumerate(map(tuple, voxel_indices)):
        voxel_grid[voxel_idx].append(points[idx])

    features = []
    for voxel_pts in voxel_grid.values():
        pts = np.array(voxel_pts)
        if pts.shape[0] < min_points_per_voxel:
            continue  # skip sparse voxels

        mean = pts.mean(axis=0)  # [3]
        cov = np.cov(pts.T)  # [3, 3]
        if cov.shape != (3, 3):
            continue

        cov_flat = cov.flatten()[:9]  # [9]
        feature = np.concatenate([mean, cov_flat])  # [12]
        features.append(feature)

    features = np.array(features)  # [M, 12]

    # 2. Normalize count to exactly `num_voxels`
    if features.shape[0] > num_voxels:
        features = farthest_point_sample(features, num_voxels)
    elif features.shape[0] < num_voxels:
        pad = np.zeros((num_voxels - features.shape[0], 12),
                       dtype=np.float32)
        features = np.vstack([features, pad])

    return features.astype(np.float32)  # [num_voxels, 12]

def farthest_point_sample(points, num_samples):
    """
    Farthest point sampling to select a subset of points from the input point cloud.
    This is used to reduce the number of points to a fixed size.

    :param points: The input point cloud
    :param num_samples: The number of points to sample

    :return: The sampled points
    """
    N = points.shape[0]
    centroids = np.zeros((num_samples,), dtype=int)
    distances = np.ones((N,)) * 1e10
    farthest = np.random.randint(0, N)

    for i in range(num_samples):
        centroids[i] = farthest
        centroid = points[farthest, :3]  # sample in XYZ space only
        dist = np.sum((points[:, :3] - centroid) ** 2, axis=1)
        distances = np.minimum(distances, dist)
        farthest = np.argmax(distances)

    return points[centroids]
