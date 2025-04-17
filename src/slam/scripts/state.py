import yaml
import rclpy.logging
from scripts.point_cloud_processors.utils.gtsam_utils import compute_scan_context_descriptor
from scripts.point_cloud_processors.ndt_transformer import NDTTransformer

from scripts.paths import CONFIG_PATH


class State:
    def __init__(self):
        with open(CONFIG_PATH, 'r') as file:
            self.config = yaml.safe_load(file)
        self.ndt_transformer = NDTTransformer(self.config['ndt_config_path'])
        self.descriptor_fn = self.ndt_transformer.generate_descriptor

        rclpy.logging.get_logger("state").info(f"State Singleton Constructed")

state = State()
