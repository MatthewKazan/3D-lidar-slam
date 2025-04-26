import open3d as o3d
import numpy as np

def compute_icp_transformation(source_cloud, target_cloud,
    t_init, voxel_size=0.02, logger=None, max_iterations=500) -> o3d.pipelines.registration.RegistrationResult:
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
            max_iteration=max_iterations,
            relative_fitness=1e-6,
            relative_rmse=1e-6
        )
    )
    if logger:
        logger(f'ICP Fitness: {result_icp.fitness}, RMSE: {result_icp.inlier_rmse}')

    return result_icp

def compute_multiscale_icp_transformation(
    source_cloud: o3d.geometry.PointCloud,
    target_cloud: o3d.geometry.PointCloud,
    t_init: np.ndarray,
    voxel_size: float = 0.02,
    logger=None,
    max_iterations: int = 100
) -> o3d.pipelines.registration.RegistrationResult:
    # 1) Coarse downsample & normals
    coarse_src = source_cloud.voxel_down_sample(voxel_size * 5)
    coarse_tgt = target_cloud.voxel_down_sample(voxel_size * 5)
    coarse_src.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 10, max_nn=30)
    )
    coarse_tgt.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 10, max_nn=30)
    )

    # 2) Coarse point‐to‐plane ICP with wide threshold
    th_coarse = voxel_size * 20  # ~40 cm
    result_coarse = o3d.pipelines.registration.registration_icp(
        coarse_src, coarse_tgt, th_coarse, t_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )
    if logger:
        logger(f"[coarse] fitness={result_coarse.fitness:.4f} "
               f"RMSE={result_coarse.inlier_rmse:.4f}")

    # 3) Fine downsample & normals
    fine_src = source_cloud.voxel_down_sample(voxel_size)
    fine_tgt = target_cloud.voxel_down_sample(voxel_size)
    fine_src.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )
    fine_tgt.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30)
    )

    # 4) Fine point‐to‐plane ICP with tight threshold
    th_fine = voxel_size * 2.5  # ~5 cm
    result_fine = o3d.pipelines.registration.registration_icp(
        fine_src, fine_tgt, th_fine, result_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=max_iterations,
            relative_fitness=1e-6,
            relative_rmse=1e-6
        )
    )
    if logger:
        logger(f"[fine]   fitness={result_fine.fitness:.4f} "
               f"RMSE={result_fine.inlier_rmse:.4f}")

    return result_fine


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


def downsample_to_target(pcd: np.array,
                         target: int,
                         tol: int,
                         max_iters: int = 20) -> (float, np.array):
    """
    Find a voxel_size so that voxel_down_sample yields N in [target - tol, target + tol].

    Args:
      pcd:         your input PointCloud
      target:      k * c  (desired number of points)
      tol:         ±t tolerance
      max_iters:   how many binary‐search steps to do

    Returns:
      A voxel‐downsampled PointCloud with approx target points.
    """
    def n_pts(vs):
        return len(pcd.voxel_down_sample(vs).points)

    o3d_pcd = o3d.geometry.PointCloud()
    o3d_pcd.points = o3d.utility.Vector3dVector(pcd)
    pcd = o3d_pcd

    # 1) establish a bracket [low, high] where n_pts(low) >= target+t
    #    and              n_pts(high) <= target - t
    low, high = 0.0, 1.0
    # grow high until we drop below (target - tol)
    while n_pts(high) > target - tol:
        low = high
        high *= 2.0

    # 2) binary search
    best = pcd
    for _ in range(max_iters):
        mid = 0.5 * (low + high)
        down = pcd.voxel_down_sample(mid)
        N = len(down.points)

        if abs(N - target) <= tol:
            return mid, np.asarray(down.points)

        # since N(vs) is decreasing in vs:
        if N > target + tol:
            # too many points → need coarser grid → increase vs
            low = mid
        else:
            # too few points → need finer grid → decrease vs
            high = mid

        best = down

    return mid, np.asarray(best.points)  # best effort if exact target±tol never hit
