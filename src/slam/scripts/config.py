from dataclasses import dataclass

from scripts.algorithm_enum import AlgorithmType, DescriptorType

@dataclass
class PoseGraphConfig:
    voxel_size: float = 0.02
    do_pose_graph_optimization: bool = True
    point_thresh: int = 75000


@dataclass
class SLAMConfig:
    algorithm_type: AlgorithmType = AlgorithmType.ICP
    descriptor_type: DescriptorType = DescriptorType.NDT_T
    dgr_weights_path: str = ""
    ndt_weights_path: str = ""
    is_saving_inputs: bool = False
    voxel_size: float = 0.02
    pose_graph: PoseGraphConfig = PoseGraphConfig()

