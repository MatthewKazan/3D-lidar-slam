from enum import Enum

from scripts.point_cloud_processors import ICPProcessor

from scripts.point_cloud_processors import DGRProcessor


class AlgorithmType(Enum):
    ICP = "ICP"
    DGR = "DGR"

processor_constructor = {
    AlgorithmType.ICP: ICPProcessor,
    AlgorithmType.DGR: DGRProcessor
}