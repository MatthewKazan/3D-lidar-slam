from dataclasses import dataclass
from enum import Enum

from scripts.algorithm_enum import AlgorithmType, DescriptorType

@dataclass
class PoseGraphConfig:
    enabled: bool = True
    voxel_size: float = 0.02
    do_pose_graph_optimization: bool = True
    point_thresh: int = 75000
    loop_closure_similarity_threshold: float = 0.97 # [.8,.9] works for scan context
    optimization_frequency: int = 2
    min_keyframe_gap: int = 5
    optimizer_verbosity: str = "ERROR"


@dataclass
class SLAMConfig:
    algorithm_type: AlgorithmType = AlgorithmType.ICP
    descriptor_type: DescriptorType = DescriptorType.NDT_T
    dgr_weights_path: str = ""
    ndt_weights_path: str = ""
    is_saving_inputs: bool = False
    voxel_size: float = 0.02
    pose_graph: PoseGraphConfig = PoseGraphConfig()
    downsample_freq: int = 10
    dgr_pc_scale_diff: float = 2
    rso_nb_neighbors: int = 45
    rso_std_ratio: float = 2.6


from multiprocessing import Manager
import dataclasses

def dataclass_to_namespace(dc, manager=None):
    if not dataclasses.is_dataclass(dc):
        raise ValueError("Expected a dataclass instance")

    if manager is None:
        manager = Manager()
    ns = manager.Namespace()

    for f in dataclasses.fields(dc):
        val = getattr(dc, f.name)
        if dataclasses.is_dataclass(val):
            setattr(ns, f.name,
                    dataclass_to_namespace(val, manager))
        elif isinstance(val, Enum):
            setattr(ns, f.name, val.value)
        else:
            setattr(ns, f.name, val)
    return ns
