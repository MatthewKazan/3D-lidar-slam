# 3D SLAM using an iPhone camera

This project implements a **Simultaneous Localization and Mapping (SLAM)** system using an 
iPhone LiDAR scanner, ROS2, and various algorithms to build a 3D reconstruction of an 
indoor environment. ICP is the standard algorithm used for point cloud registration, but work 
is being done to implement a more robust algorithm. The project is designed to be modular,
allowing for easy integration of new algorithms and features. The system is capable of
localizing the iPhone in the environment and building a 3D map of the environment in real-time.

## Table of Contents

1. [Installation](#installation)
2. [Project Structure](#project-structure)
3. [The iPhone app](#the-iphone-app)
4. [Usage](#usage)
5. [Algorithms](#algorithms)
6. [Adding Algorithms](#adding-algorithms)
7. [Known Issues](#known-issues)


## Installation

### Prerequisites
Basic knowledge of ROS2, Python and python packaging is assumed
- [ROS2](https://docs.ros.org/en/humble/Installation.html) this was tested on Humble but other versions may work
- [Xcode](https://developer.apple.com/xcode/) for iPhone app development though other data collection methods 
would work as long as the data is in the correct format and sent to the correct topic
- [Mambda + Robostack](https://robostack.github.io/GettingStarted.html) follow the instructions to install mamba and robostack. 
For linux this shouldn't be necessary but for macOS it is the easiest way to install ROS2. 
Currently, the only tested configuration is on macOS with mamba and robostack.

---

### Setup Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/MatthewKazan/3D-lidar-slam.git
   cd iphone-lidar-slam
   ```

2. Install the required dependencies:

   *This step will only work for when ROS2 Humble was installed using mamba and robostack.*

   Once you have setup your mamba env following the instructions at https://robostack.github.io/GettingStarted.html, you need to
   install the required packages, run the following commands from the **root of the repository while in the ROS2 mamba environment**:
   ```bash
    mamba env update --file environment.yml --prune
   ```

3. Build the ROS2 workspace:
   ```bash
   colcon build
   ```

4. Set up environment:
   ```bash
   source install/setup.bash
   ```
    or
    ```bash
    source install/setup.zsh
    ```
   
5. Install the iPhone app:

   *You must have an Apple developer account to install the app on your iPhone along with Xcode*

   Open Xcode on your Mac.
   Open the Xcode project (from the .xcodeproj file in your local repo folder).
   In Xcode, choose File > Open, and select your project file.
   Then follow [these instructions](https://codewithchris.com/deploy-your-app-on-an-iphone/) to install and run the app on your iPhone.

---

## Project Structure

```
iphone-lidar-slam/
├── src/                  # Source code for the SLAM system
│   ├── slam/             # ROS2 node for ICP-based SLAM
│   │   ├── config/       # Various configuration files including vital calibration config
│   │   ├── launch/       # Launch files for the SLAM node
│   │   ├── rosbags/      # Directory for storing rosbag files, both input and global maps
│   │   ├── scripts/      # Python scripts for data processing, and other utilities
│   │   ├── slam/         # Main ROS2 nodes which publish and subscribe to various topics
│   ├── archive/          # Old useless code i'm too scared to delete and too lazy to move to a different branch
│   ├── misc/             # Miscellaneous files, including the writeup and other documents
├── lidar_ios_app/        # iOS app for capturing point clouds
├── environment.yml       # Conda environment file for setting up the required packages
└── README.md             # This file
```

---


## The iPhone app
The iPhone app is in the lidar directory, its written in Swift and uses the ARKit library to capture pointclouds. The app uploads
each scan to ROS2 via the rosbridge suite. This app is mostly a prototype and is not meant to look pretty.
- Use the **Set IP** button to set the IP address of the computer running ROS2.
- Use the **Start Scanning** button to start the scan. The app will start capturing pointclouds and sending them to ROS2.
- Use the **Stop Scanning** button to stop the scan. The app will stop capturing pointclouds.
- Use the **Clear DB** button to reset the global map. This will clear all pointclouds stored in the ROS2 nodes or topics fully resetting the system.
- Use the **Save Map** button to save the global map to a rosbag file.
- Use the **Save Inputs** button to save each scan the app captures to a rosbag file. 
Use this when working on processing algorithms to test on the same input data repeatedly.
Tap this button again to stop saving inputs.

To save multiple global maps use the save map button then reset the system using the clear db button.
This will save the global map to a rosbag file and reset the system so the next global map will save to a different file name.

---

## Usage

To build, and launch the ROS2 SLAM system with the visualizer
```bash
    colcon build --symlink-install
    source install/setup.<bash or zsh>
    ros2 launch slam slam.launch.py launch_rviz:=true config:=lidar_config.yaml
```
As scans come in from the iPhone app, the SLAM system will process them and publish the global map 
to the /global_map topic. 
Add that topic to rviz2 to visualize the global map with the following changes to default settings:
- PointCloud2
- Topic: /global_map
- Style: Points
- Color Transformer: AxisColor

### To view saved global maps
```bash
rviz2
```
Then in a new terminal

```bash
 ros2 bag play src/slam/rosbags/rosbags_<unique_id>/global/global_0.db3
 ```
For example
```bash
 ros2 bag play src/slam/rosbags/rosbags_20250213_190958/global/global_0.db3
 ```

### To use saved inputs instead of the iPhone app
Launch the ROS2 system as normal
```bash
    colcon build --symlink-install
    source install/setup.<bash or zsh>
    ros2 launch slam slam.launch.py launch_rviz:=true config:=lidar_config.yaml
```
Then in a new terminal, run the following command to play the rosbag file:
```bash
    ros2 bag play src/slam/rosbags/rosbags_<unique_id>/inputs/inputs_0.db3
```
For example
```bash
    ros2 bag play src/slam/rosbags/rosbags_20250213_190958/inputs/inputs_0.db3
```

And the inputs will be sent to the SLAM system as if they were coming from the iPhone app.

---

## Algorithms

### ICP-based SLAM

The core of the project initially uses the **Iterative Closest Point (ICP)** algorithm for matching LiDAR point clouds over time. 
The point clouds are aligned using ICP, and the system estimates the pose of the LiDAR scanner in each frame.

### Deep Learning Integration (Current Work)

Several deep learning techniques will be integrated into the SLAM system for the following purposes:

- **Loop Closure Detection**: Using neural networks to identify previously visited locations and correct drift in the map.
- **Outlier Rejection**: Using deep learning models to distinguish between valid scan points and outliers that can distort the map.
- **Initial Pose Estimation**: Applying deep learning to predict an initial guess for the pose before ICP optimization.

These features will be incorporated gradually, with the goal of enhancing SLAM accuracy and robustness.

---

## Adding Algorithms

The SLAM system is designed to be modular, allowing for easy integration of new algorithms and features.
To add a new algorithm, simply implement a new class that inherits from the abstract base class `ProcessPointClouds` and update the get_processor 
method in the `process_point_clouds.py` file to return an instance of the new class.

---

## Known Issues
- The iPhone app is not optimized for performance and absolutely destroys the battery life.
- ICP algorithm is not optimized for large environments and can be slow as global map grows. If the environment is too large or the RAM
  is too small, the system may crash. 
- ICP is not robust to loop closure and can drift over time.
- iPhone and ROS2 may not connect if the shared network does not allow LAN connections which is common on university networks.
