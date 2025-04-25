import copy
import typing

import open3d as o3d
import numpy as np
from gtsam import Pose3, Rot3, Point3
from scripts.pointcloud_processors.descriptor_generators.ndt_transformer import \
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

    def voxel_down_sample(self, voxel_size: float = 0.02):
        """
        Downsample the point cloud using voxel downsampling.
        """
        if self.point_cloud.is_empty():
            return
        return copy.deepcopy(self.point_cloud).voxel_down_sample(voxel_size)

    def convert_to_local_frame(self):
        self.point_cloud = self.point_cloud.transform(np.linalg.inv(self.matrix))

