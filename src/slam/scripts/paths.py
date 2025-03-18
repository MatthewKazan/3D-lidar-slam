"""
This module contains utility functions for handling paths in the slam package.
"""
import os
import time

PATH_TO_WORKSPACE = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
PATH_TO_PACKAGE = os.path.abspath(os.path.join(PATH_TO_WORKSPACE, "src/slam/"))
PATH_TO_ROSBAGS = os.path.join(PATH_TO_PACKAGE, "rosbags/")
PATH_TO_CONFIG = os.path.join(PATH_TO_PACKAGE, "config/")
PATH_TO_BUILD = os.path.join(PATH_TO_WORKSPACE, "build/")
PATH_TO_BUILD_DGR = os.path.join(PATH_TO_BUILD, "libs/libs/DeepGlobalRegistration")
PATH_TO_BUILD_MINK = os.path.join(PATH_TO_BUILD, "libs/libs/MinkowskiEngine")


def generate_unique_bag_name(bag_prefix="global_map_bag") -> str:
    """
    Generates a unique bag file name by appending a timestamp.

    :param bag_prefix: The base name of the bag file
    """
    timestamp = time.strftime("%Y%m%d_%H%M%S")  # Format: YYYYMMDD_HHMMSS
    return f"{bag_prefix}_{timestamp}"


if __name__ == "__main__":
    pass