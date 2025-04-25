import multiprocessing
from collections import defaultdict
from typing import Callable, Any

import yaml
import rclpy.logging
# from scripts.pointcloud_processors.utils.gtsam_utils import compute_scan_context_descriptor
# from scripts.pointcloud_processors.ndt_transformer import NDTTransformer
from scripts.algorithm_enum import AlgorithmType, DescriptorType

from scripts.paths import CONFIG_PATH

CONFIG_KEY_NAMES = [

]


class State:
    def __init__(self):
        with open(CONFIG_PATH, 'r') as file:
            self._config = yaml.safe_load(file)
        self._listeners = defaultdict(list)
        self._lock = multiprocessing.Lock()

        # self.descriptor_type = DescriptorType[self.config.get('descriptor_type', 'NDT_T')]
        # self.algorithm_type = AlgorithmType[self.config.get('descriptor_type', 'ICP')]


        rclpy.logging.get_logger("state").info(f"State Singleton Constructed")


    def set(self, key, value):
        with self._lock:
            old = self._config.get(key)
            if old == value:
                return  # no change, no callbacks
            self._config[key] = value

        # fire callbacks for *this* key only
        for cb in self._listeners[key]:
            try:
                cb(value)
            except Exception as e:
                rclpy.logging.get_logger("state").info(f"Error in callback for key {key}: {type(e)}: {e}")

    def get(self, key, default=None):
        with self._lock:
            return self._config.get(key, default)

    def subscribe(self, key, callback: Callable[Any, Any]):
        """
        callback: (old_value, new_value) -> None
        """
        self._listeners[key].append(callback)

# Reconstructed ever
state = State()
