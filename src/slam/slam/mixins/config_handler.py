import dataclasses

from rcl_interfaces.msg import ParameterType, ParameterEvent
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
                self.get_logger().info(f"Declared parameter: {key} = {default!r}")

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

    def _on_params_changed(self, event):
        if event.node != self.get_fully_qualified_name():
            return

        for p in event.changed_parameters:
            name = p.name  # e.g. "pose_graph.optimization_frequency"
            pv = p.value

            # Unpack into a plain Python value
            if pv.type == ParameterType.PARAMETER_BOOL:
                value = pv.bool_value
            elif pv.type == ParameterType.PARAMETER_INTEGER:
                value = pv.integer_value
            elif pv.type == ParameterType.PARAMETER_DOUBLE:
                value = pv.double_value
            elif pv.type == ParameterType.PARAMETER_STRING:
                value = pv.string_value
            elif pv.type == ParameterType.PARAMETER_BYTE_ARRAY:
                value = list(pv.byte_array_value)
            elif pv.type == ParameterType.PARAMETER_BOOL_ARRAY:
                value = list(pv.bool_array_value)
            elif pv.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                value = list(pv.integer_array_value)
            elif pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                value = list(pv.double_array_value)
            elif pv.type == ParameterType.PARAMETER_STRING_ARRAY:
                value = list(pv.string_array_value)
            else:
                self.get_logger().warn(
                    f"Unknown parameter type {pv.type} for '{name}'")
                continue

            # --- now handle nested field names ---
            parts = name.split('.')
            # top-level attr is parts[0]
            attr = parts[0]
            if not hasattr(self.config, attr):
                self.get_logger().warn(f"No config field '{attr}'")
                continue

            if len(parts) == 1:
                # simple case: top-level field
                setattr(self.config, attr, value)

            else:
                # nested: walk into the dataclass
                target = getattr(self.config, attr)
                for sub in parts[1:-1]:
                    target = getattr(target, sub)
                setattr(target, parts[-1], value)

            self.get_logger().info(f"Updated config.{name} = {value!r}")


