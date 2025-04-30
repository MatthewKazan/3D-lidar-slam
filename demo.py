"""
To run this file, create the given mamba env in environment.yml, and install the required packages,
open a terminal at the project root, activate the mamba env, source ROS2, and run:

python3 demo.py

This script launches the SLAM system and plays a ROS2 bag file. You should see a global map of a
room being slowly constructed in RViz.

If it fails, see the README.md for detailed setup and troubleshooting steps.
"""
import subprocess
import threading
import time
import os

# Paths to ROS2 and workspace setup files
workspace_setup = "install/setup.bash"

# Full setup command prefix
setup_cmd = f"source {workspace_setup} && "

# Commands
slam_cmd = setup_cmd + "ros2 launch slam slam.launch.py launch_rviz:=true config:=config.yaml"
rosbag_cmd = setup_cmd + "ros2 bag play src/slam/rosbags/input_bags/inputs_20250410_203244"

# Function to run a shell command in a thread
def run_command(command):
    subprocess.run(["bash", "-c", command])

# Launch SLAM system
print("Launching SLAM system with RViz...")
slam_thread = threading.Thread(target=run_command, args=(slam_cmd,))
slam_thread.start()

# Wait for the SLAM system to initialize
time.sleep(5)

# Play ROS2 bag
print("Playing ROS2 bag...")
rosbag_thread = threading.Thread(target=run_command, args=(rosbag_cmd,))
rosbag_thread.start()

# Wait for both threads to finish
slam_thread.join()
rosbag_thread.join()

print("Demo complete.")
