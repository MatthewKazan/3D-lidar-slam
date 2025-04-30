"""
This file isn't in algorithm_enum.py because at one point it caused
circular imports.
"""
from scripts.algorithm_enum import AlgorithmType
from scripts.pointcloud_processors.pointcloud_registration import \
    DGRProcessor, ICPProcessor, MultiscaleICPProcessor

processor_constructor = {
    AlgorithmType.ICP: ICPProcessor,
    AlgorithmType.MULTISCALE_ICP: MultiscaleICPProcessor,
    AlgorithmType.DGR: DGRProcessor
}