import typing

import open3d as o3d
import numpy as np
from gtsam import Pose3, Rot3, Point3
from scripts.point_cloud_processors.ndt_transformer import \
    NDTTransformer

def pose_from_matrix(matrix: np.array) -> Pose3:
    """Convert a 4x4 numpy array to a GTSAM Pose3 object."""
    return Pose3(Rot3(matrix[:3, :3]), Point3(matrix[:3, 3]))


class Submap:
    def __init__(self,
        pose: typing.Union[Pose3, np.array] = None,
        point_cloud: o3d.geometry.PointCloud = o3d.geometry.PointCloud(),
    ):
        """"""
        if isinstance(pose, np.ndarray):
            pose = pose_from_matrix(pose)
        self.pose = pose
        self._delta_from_original_pose = None
        self.point_cloud = point_cloud
        self.descriptor = None

        #TODO: implement cached descriptor, and point_cloud for when submap hasnt' updated

    @property
    def pose(self) -> Pose3:
        return self._pose

    @pose.setter
    def pose(self, pose: typing.Union[Pose3, np.array]):
        if isinstance(pose, np.ndarray):
            pose = pose_from_matrix(pose)
        self._pose = pose

    @property
    def matrix(self) -> typing.Optional[np.ndarray]:
        if self._pose is None:
            return None
        return np.array(self._pose.matrix())

    @property
    def points(self) -> np.ndarray:
        return np.asarray(self.point_cloud.points)

    def __add__(self, other):
        if isinstance(other, Submap):
            new_point_clouds = self.point_cloud + other.point_cloud
            if self.pose is None:
                new_pose = other._pose
            else:
                new_pose = self._pose
        elif isinstance(other, o3d.geometry.PointCloud):
            new_point_clouds = self.point_cloud + other
            new_pose = self._pose
        else:
            raise TypeError("Can only add Submap, or pointcloud objects")
        return Submap(pose=new_pose, point_cloud=new_point_clouds)

    def outlier_rejection(self):
        """
        Remove outliers from the point cloud using statistical outlier removal.
        """
        if self.point_cloud.is_empty():
            return
        self.point_cloud, _ = self.point_cloud.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=3.0)


def compute_scan_context_descriptor(scan, num_angle_bins=60, num_radius_bins=20,
    max_range=80.0):
    """
    Computes a Scan Context descriptor for a point cloud.

    :param scan: Input point cloud of shape (N, 3) with columns [x, y, z].
    :param num_angle_bins: Number of bins for the azimuth angle.
    :param num_radius_bins: Number of bins for the radial distance.
    :param max_range: Maximum range to consider for binning.

    :return: Flattened descriptor vector.
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
