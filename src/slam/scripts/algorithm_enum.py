from enum import Enum


class AlgorithmType(Enum):
    ICP = "ICP"
    DGR = "DGR"


class DescriptorType(Enum):
    SCAN_CONTEXT = "SCAN_CONTEXT"
    NDT_T = "NDT_T"

