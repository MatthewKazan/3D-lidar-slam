import os
import random
import sys
import open3d as o3d

from scripts.paths import PATH_TO_BUILD_NDT, PATH_TO_BUILD_ARKIT
from scripts.point_cloud_processors.ndt_transformer import \
    NDTTransformer, ndt_voxelize1

from src.slam.scripts.point_cloud_processors.ndt_transformer import \
    ndt_voxelize1

sys.path.append(PATH_TO_BUILD_NDT)  # Ensure the path is in Python's search
sys.path.append(PATH_TO_BUILD_ARKIT)  # Ensure the path is in Python's search

import libs.NDT_Transformer.models.NDTNetVlad as PNV
import libs.NDT_Transformer.config as cfg

from libs.ARKitScenes.threedod.benchmark_scripts.utils.tenFpsDataLoader import TenFpsDataLoader
import libs.ARKitScenes.threedod.benchmark_scripts.utils.taxonomy as taxonomy

import numpy as np

def overlap_fraction(pc_a, pose_a, pc_b, pose_b, max_dist=0.1):
    # Transform into world frame
    pcd_a = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_a)).voxel_down_sample(.05)
    pcd_b = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_b)).voxel_down_sample(.05)
    pcd_a.transform(pose_a)
    pcd_b.transform(pose_b)

    # Use KDTree to count A→B correspondences
    tree = o3d.geometry.KDTreeFlann(pcd_b)
    pts_a = np.asarray(pcd_a.points)
    inliers = 0
    for p in pts_a:
        [_, idx, d2] = tree.search_knn_vector_3d(p, 1)
        if d2[0] < max_dist**2:
            inliers += 1
    return inliers / len(pts_a)

data_root = "/Users/mattkazan/PrivateFiles/NortheasternClasses/Year5/EECE5550/FinalProject/final_project/src/slam/scripts/deep_learning/3dod/Training/"

scene_id = "41254925"
data_path = os.path.join(data_root, scene_id,
                         f"{scene_id}_frames")
neg_loader = TenFpsDataLoader(
    dataset_cfg=None,
    class_names=taxonomy.class_names,
    root_path=data_path,
)



scene_id = "40777060"
data_path = os.path.join(data_root, scene_id,
                         f"{scene_id}_frames")
loader = TenFpsDataLoader(
    dataset_cfg=None,
    class_names=taxonomy.class_names,
    root_path=data_path,
)

pcs = []
poses = []

for i in range(len(loader)):
    frame = loader[i]

    print(f"{i}/{len(loader)}", frame["image_path"])
    image_path = frame["image_path"]
    pcd = ndt_voxelize1(frame["pcd"])  # in world coordinate
    pose = frame["pose"]
    rgb = frame["color"]
    pcs.append(pcd)
    poses.append(pose)

triplets = []
for i in range(len(pcs)):
    print(f"Triplet {i}/{len(pcs)}")
    for j in range(len(pcs)):
        if i - 20 < j < i + 20:
            continue
        if overlap_fraction(pcs[i][:, :3], poses[i], pcs[j][:, :3], poses[j]) > 0.3:
            print(f"Triplet {i} and {j} overlap")
            triplets.append((i, j, random.randint(0, len(neg_loader)-1)))
            # pc_a = pcs[i][:, :3]
            # pc_b = pcs[j][:, :3]
            #
            # pcd_a = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_a))
            # pcd_b = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_b))
            #
            # o3d.visualization.draw_geometries([pcd_a, pcd_b])
            break

for i, j, k in triplets:
    print(f"Triplet: {i}, {j}, {k}")
    pc_a = pcs[i][:, :3]
    pc_b = pcs[j][:, :3]
    pc_c = neg_loader[k]['pcd'][:, :3]

    pcd_a = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_a))
    pcd_b = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_b))
    pcd_c = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_c))

    o3d.visualization.draw_geometries([pcd_a, pcd_b, pcd_c])
