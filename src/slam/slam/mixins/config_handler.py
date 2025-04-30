import dataclasses

from rcl_interfaces.msg import ParameterEvent
from rclpy.callback_groups import ReentrantCallbackGroup

from scripts.config import SLAMConfig
from enum import Enum
import rclpy.logging

class ConfigHandlerMixin:
    """
    A class to handle configuration settings for the SLAM system.
    """

    def __init_config_handler__(self):

        self.declare_from_dataclass(SLAMConfig)
        self.update_config_from_params(self.config, SLAMConfig)
        rclpy.logging.get_logger("processing_manager").info(
            f"ConfigHandlerMixin: {self.config}")
        # 1) Create a separate callback group
        self._param_cbg = ReentrantCallbackGroup()

        # 2) Subscribe to parameter events in that group
        self.create_subscription(
            ParameterEvent,
            '/parameter_events',  # global events topic
            self._on_params_changed,
            10,
            callback_group=self._param_cbg)

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
                self.get_logger().debug(f"Declared parameter default: {key} = {default!r}")

    def update_config_from_params(self, config, dc_type,
        prefix: str = "") -> None:
        """
        Recursively pull values from ROS2 parameters into `config` in place.

        :param config:    An existing dataclass instance to modify.
        :param dc_type:   The dataclass type (e.g. SLAMConfig or a nested type).
        :param prefix:    Dot‐separated prefix used in parameter names.
        """
        for f in dataclasses.fields(dc_type):
            # build the ROS param name
            key = f"{prefix}.{f.name}" if prefix else f.name

            # nested dataclass? recurse into it
            if dataclasses.is_dataclass(f.type):
                # get the child instance off `config`
                child = getattr(config, f.name)
                self.update_config_from_params(child, f.type, key)
            else:
                # pull the raw parameter value
                raw = self.get_parameter(key).value

                # if it’s an Enum field, convert the string back
                if isinstance(f.type, type) and issubclass(f.type, Enum):
                    try:
                        raw = f.type(raw.upper())
                    except ValueError as e:
                        self.get_logger().error(f"Bad enum for {key}: {raw}")
                        continue

                # finally, overwrite the existing attribute
                setattr(config, f.name, raw)
                self.get_logger().info(f"Config update: {key} → {raw!r}")

    def handle_params_changed(
            self, config, dc_type, changed_keys: set[str], prefix: str = ""
        ) -> None:
        """
        Recursively pull values from ROS2 parameters into `config`,
        but only update fields in `changed_keys`.

        :param config:       The config dataclass (e.g. SLAMConfig).
        :param dc_type:      The dataclass type.
        :param changed_keys: A set of fully qualified parameter names that have changed.
        :param prefix:       Dot‐separated prefix used in parameter names.
        """
        for f in dataclasses.fields(dc_type):
            key = f"{prefix}.{f.name}" if prefix else f.name

            if dataclasses.is_dataclass(f.type):
                # Recurse if any nested keys match the prefix
                nested_relevant = any(
                    k.startswith(f"{key}.") for k in changed_keys)
                if nested_relevant:
                    child = getattr(config, f.name)
                    self.handle_params_changed(child, f.type,
                                                   changed_keys, key)
                continue

            if key not in changed_keys:
                continue  # Skip keys that haven't changed

            try:
                raw = self.get_parameter(key).value
            except Exception:
                self.get_logger().warn(f"Parameter '{key}' not declared.")
                continue

            # Handle enums
            if isinstance(f.type, type) and issubclass(f.type, Enum):
                try:
                    raw = f.type(raw.upper())
                except ValueError:
                    self.get_logger().error(
                        f"Bad enum value for {key}: {raw}")
                    continue

            setattr(config, f.name, raw)
            self.get_logger().info(f"Config updated: {key} → {raw!r}")

    def _on_params_changed(self, event):
        if event.node != self.get_fully_qualified_name():
            return

        changed_keys = {p.name for p in event.changed_parameters}
        self.handle_params_changed(self.config, SLAMConfig,
                                       changed_keys)

        self.handle_param_specifics(event)

