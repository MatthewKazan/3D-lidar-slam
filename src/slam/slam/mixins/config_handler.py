import dataclasses

from rclpy.node import Node

from scripts.config import SLAMConfig
from enum import Enum
import rclpy.logging

class ConfigHandlerMixin:
    """
    A class to handle configuration settings for the SLAM system.
    """

    def __init_config_handler__(self):

        self.declare_from_dataclass(SLAMConfig)
        self.config = self.read_into_dataclass(SLAMConfig)
        rclpy.logging.get_logger("processing_manager").info(
            f"ConfigHandlerMixin: {self.config}")

    def declare_from_dataclass(self, dc, prefix=''):
        for f in dataclasses.fields(dc):
            key = f"{prefix}{f.name}" if not prefix else f"{prefix}.{f.name}"
            # if it’s a nested dataclass:
            if dataclasses.is_dataclass(f.type):
                self.declare_from_dataclass(f.type, key)
            else:
                default = f.default
                if issubclass(f.type, Enum):
                    default = default.value
                self.declare_parameter(key, default)

    def read_into_dataclass(self, dc, prefix=''):
        kwargs = {}
        for f in dataclasses.fields(dc):
            key = f"{prefix}{f.name}" if not prefix else f"{prefix}.{f.name}"
            if dataclasses.is_dataclass(f.type):
                kwargs[f.name] = self.read_into_dataclass(f.type, key)
            else:
                raw = self.get_parameter(key).value
                if issubclass(f.type, Enum):
                    raw = f.type(raw.upper())
                kwargs[f.name] = raw
        return dc(**kwargs)

