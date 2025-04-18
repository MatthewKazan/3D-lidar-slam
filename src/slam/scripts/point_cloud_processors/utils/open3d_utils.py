import open3d as o3d
import numpy as np

def compute_icp_transformation(source_cloud, target_cloud,
    t_init, voxel_size=0.02, logger=None) -> np.ndarray:
    """
    Align two point clouds using RANSAC and then ICP.

    :param source_cloud: The new point cloud to align
    :param target_cloud: The global map to align the new point cloud with
    :param voxel_size: The voxel size for downsampling the point clouds

    :return: The 4x4 transformation matrix to align the new point cloud with the global map
    """

    source_cloud = source_cloud.voxel_down_sample(voxel_size)

    target_cloud = target_cloud.voxel_down_sample(voxel_size)

    # Align with ICP
    result_icp = o3d.pipelines.registration.registration_icp(
        source_cloud, target_cloud,
        max_correspondence_distance=voxel_size * 2.5,
        init=t_init,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=1000,
            relative_fitness=1e-6,
            relative_rmse=1e-6
        )
    )
    if logger:
        logger(f'ICP Fitness: {result_icp.fitness}, RMSE: {result_icp.inlier_rmse}')

    return result_icp.transformation


def np_from_o3d_point_cloud(point_cloud: o3d.geometry.PointCloud) -> np.ndarray:
    """
    Convert an Open3D point cloud to a numpy array.

    :param point_cloud: The Open3D point cloud to convert
    :return: A numpy array of shape (N, 3) containing the point cloud data
    """
    return np.asarray(point_cloud.points)

def o3d_from_np_point_cloud(points: np.ndarray) -> o3d.geometry.PointCloud:
    """
    Convert a numpy array to an Open3D point cloud.

    :param points: A numpy array of shape (N, 3) containing the point cloud data
    :return: An Open3D point cloud
    """
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud