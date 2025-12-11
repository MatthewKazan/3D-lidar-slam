# 3D SLAM using an iPhone camera

This project implements a **Simultaneous Localization and Mapping (SLAM)** system using an 
iPhone LiDAR scanner, ROS2, and various algorithms to build a 3D reconstruction of an 
indoor environment. Currently ICP and DGR are implemented for point cloud registration and GTSAM for loop closure. 
The project is designed to be modular, allowing for easy integration of new algorithms and features. 
The system is capable of localizing the iPhone in the environment and building a 3D map of the environment in real-time.

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
- [Mamba + Robostack](https://robostack.github.io/GettingStarted.html) follow the instructions to install mamba and robostack. 
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
   colcon build --symlink-install
   ```

4. Set up environment:
   ```bash
   source install/setup.bash
   ```
    or
    ```zsh
    source install/setup.zsh
    ```
   
5. Install the iPhone app:

   *You must have an Apple developer account to install the app on your iPhone along with Xcode*

   Open Xcode on your Mac.
   Open the Xcode project (from the .xcodeproj file in your local repo folder).
   In Xcode, choose File > Open, and select your project file.
   Then follow [these instructions](https://codewithchris.com/deploy-your-app-on-an-iphone/) to install and run the app on your iPhone.

6. Install DGR and Minkowski Engine and ensure their dpendencies are installed. 
   Follow the instructions in their respective repositories to install them. 
   For Mac they are annoying, requires manually editing the `setup.py` files in Minkowski 
   to ensure the clang path is correct. Then DGR dependencies are also annoying to install,
   need to manually install `easydict` with conda/mamba and `open3d==0.17.0` with pip.

   To use the demo for DGR on mac you need to update the DeepGlobalRegistration call to add a `device` argument
   which is 'cpu' for mac. The default is 'cuda' which will throw an error on mac as it doesn't have a GPU.

   Setup Minkowski for Mac with
   ```bash
   python setup.py install --cpu_only
   ```
   If it fails with linker errors, its because of weird conflicting build env variables between pytorch and Minkowski: 
   uninstall pytorch then:
   ```bash
      mamba install -c conda-forge "pytorch=*"  
      brew install libomp
      brew install openblas
      export LDFLAGS="-L/opt/homebrew/opt/openblas/lib"                
      export CPPFLAGS="-I/opt/homebrew/opt/openblas/include"
      export CXXFLAGS="-Xclang -fopenmp"  
      export CXX=/opt/homebrew/opt/llvm/bin/clang++                    
      export CC=/opt/homebrew/opt/llvm/bin/clang
      export CMAKE_CXX_COMPILER="/opt/homebrew/opt/llvm/bin/clang++"
      export CMAKE_C_COMPILER="/opt/homebrew/opt/llvm/bin/clang"
   ```   
   No idea what portion of that is actually necessary, but it worked for me. If you already 
   have libomp and openblas installed, reinstall them.
            
   If you want to use the DGR demo, you need to run it once, it will have after redkitchen_010.ply
   kill it, then manually download the weights from the [here](https://node1.chrischoy.org/data/publications/fcgf/2019-08-16_19-21-47.pth)
   and place it in the same dir as the redkitchen_010.ply file. 
   Then run the demo again and it might work.

   If it hangs after Setting voxel size, run ```bash export OMP_NUM_THREADS=1``` and try again.

8. If colcon build fails, reinstall pytorch with the same command as above

      

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
each scan to ROS2 via the rosbridge suite.
- Use the **Start Scanning** button to start the scan. The app will start capturing pointclouds and sending them to ROS2.
- Use the **Stop Scanning** button to stop the scan. The app will stop capturing pointclouds.
- In the top right corner, tap the **Menu** button to open the a sidebar menu.
  - Use the **Reset** button to reset the global map. This will clear all pointclouds stored in the ROS2 nodes or topics fully resetting the system.
  - Use the **Save Global Map** button to save the global map to a rosbag file.
  - Use the **Start/Stop Saving Inputs** button to start/stop saving each scan the app captures to a rosbag file. Helpful for testing different algorithms on the same input data.
  - Use the **Select an Algorithm** button to select the point cloud registration algorithm to use for the SLAM system. 
  - Use the **Select a Descriptor Fn** button to select the descriptor to use for the loop closure detection.
  - Use the **Edit Other Parameters** button to edit any other parameters for the SLAM system. This includes the voxel size, and the number of keyframes to use for loop closure detection.

---

## Usage

To build, and launch the ROS2 SLAM system with the visualizer
```bash
    colcon build --symlink-install
    source install/setup.<bash or zsh>
    ros2 launch slam slam.launch.py launch_rviz:=true config:=config.yaml
```
As scans come in from the iPhone app, the SLAM system will process them and publish the global map 
to the /global_map topic. 
Add that topic to rviz2 to visualize the global map with the following changes to default settings:
```
- PointCloud2
- Topic: /global_map
- Style: Points
- Color Transformer: AxisColor
```
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
    ros2 launch slam slam.launch.py launch_rviz:=true config:=config.yaml
```
Then in a new terminal, run the following command to play the rosbag file:
```bash
    ros2 bag play src/slam/rosbags/input_bags/inputs_<unique_id>
```
For example
```bash
    ros2 bag play src/slam/rosbags/input_bags/inputs_20250401_220150/
```

And the inputs will be sent to the SLAM system as if they were coming from the iPhone app.

---
## Parameters

The SLAM system has several parameters that can be configured in the `config.yaml` file.
To see a full list, see the [config.yaml](src/slam/scripts/config.py) file.
You can configure these parameters at the start via the yaml file, or you can change them at runtime
by modifying the ros2 parameters via the iPhone app or via the command line.
```bash
ros2 param set /slam_processor <parameter_name> <value>
```

Set the parameters of the `/slam_processor` node, its the only node listening to the parameters updates.
It will automatically update the parameters of the `/pointclouds_subscriber` node.
---

## Algorithms

### ICP-based SLAM

The project can use the **Iterative Closest Point (ICP)** algorithm for matching LiDAR point clouds over time. 
The point clouds are aligned using ICP, and the system estimates the pose of the LiDAR scanner in each frame.

### Deep Learning Integration

Several deep learning techniques will be integrated into the SLAM system for the following purposes:

- **Loop Closure Detection**: Using neural networks to identify previously visited locations and correct drift in the map.
- **Outlier Rejection**: Using deep learning models to distinguish between valid scan points and outliers that can distort the map.
- **Initial Pose Estimation**: Applying deep learning to predict an initial guess for the pose before ICP optimization.

These features will be incorporated gradually, with the goal of enhancing SLAM accuracy and robustness.

Currently [Deep Global Registration (DGR)](https://arxiv.org/abs/2004.11540) is implemented as an alternative to ICP for point cloud registration.

---
## Loop Closure

Loop closure is a critical component of SLAM systems, as it helps to correct drift and improve the accuracy of the map.
The current implementation constructs keyframes out of segments of the global map, and adds them, their
pose, and descriptor to the factor graph. The factor graph is then optimized using Levenberg-Marquardt optimization
and the global map is updated with the optimized poses. 

Loop closures are detected using the cosine simularity of the descriptors for each keyframes. The descriptors 
can be constructed using the scan context descriptor function, or a descriptor constructed by an [NDT-Transformer model](https://arxiv.org/pdf/2103.12292).

The trained model used for NDT can be found [here](https://drive.google.com/file/d/1rJcswZsH05RZP3rMzfjWiwXXswikJgQd/view?usp=sharing)

---

## Adding Algorithms

The SLAM system is designed to be modular, allowing for easy integration of new algorithms and features.
To add a new algorithm, simply implement a new class that inherits from the abstract base class `ProcessPointClouds` and update the `AlgorithmType` Enum, and `processor_constructor` dict 
in [algorithm_enum.py](src/slam/scripts/algorithm_enum.py) and [algorithm_constructor.py](src/slam/scripts/algorithm_constructor.py) respectively.

Create new descriptor functions by modifying the `DescriptorType` Enum and updating the `set_descriptor` function in [process_pc_manager.py](src/slam/scripts/process_pc_manager.py).

New point cloud registration algorithms should inherit from the `ProcessPointClouds` class in [generic_point_cloud_processor.py](src/slam/scripts/pointcloud_registration/generic_point_cloud_processor.py) and implement the `construct_global_map` method.
New descriptor functions should inherit from the `GenericDescriptorGenerator` class in [generic_descriptor_generator.py](src/slam/scripts/descriptor_generators/generic_descriptor_generator.py) and implement the `generate_descriptor` method.
---

## Known Issues
- iPhone and ROS2 may not connect if the shared network does not allow LAN connections which is common on university networks.
-  Could NOT find PythonInterp (missing: PYTHON_EXECUTABLE) or some other cmake error related to rosidl_generate_interfaces.
   One possible fix is running
   ```bash
   rm -rf build install log
   colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=$(which python)
   ```
-  'Service type support not from this implementation. Got: Could not load library libcustom_interfaces__rosidl_typesupport_introspection_c.dylib...'
   This seems to be an issue with the mamba robostack env, copy the dylib files from the install directory to your mamba env with:
     ```bash
      cp -v install/**/lib/*.dylib <path to mamba env>/lib/
- ```bash
    from MinkowskiEngineBackend._C import (
     ImportError: dlopen(<path_to_mamba_env>/lib/python3.9/site-packages/MinkowskiEngineBackend/_C.cpython-39-darwin.so...
   ```
  This error like the error above is due to weird linker issues I think are specific to mac, haven't figured out
  exactly whats going on but this can be fixed by running the following commands:
  ```bash
  otool -l <path_to_mamba_env>/lib/python3.9/site-packages/MinkowskiEngineBackend/_C.cpython-39-darwin.so | grep -A2 LC_RPATH
   ```
  Then based on the lc paths it shows you delete all of them with
  ```bash
    install_name_tool -delete_rpath <LC_PATH> <path_to_mamba_env>/lib/python3.9/site-packages/MinkowskiEngineBackend/_C.cpython-39-darwin.so
    ```
    Then run the otool command again to ensure they are gone.
   Then add the correct rpath with
   ```bash
    install_name_tool -add_rpath <correct_path> <path_to_mamba_env>/lib/python3.9/site-packages/MinkowskiEngineBackend/_C.cpython-39-darwin.so
    ```
  The correct path for me was just `<path_to_mamba_env>/lib/python3.9/site-packages/MinkowskiEngineBackend/`

# Citations

```latex
@inproceedings{choy2020deep,
  title={Deep Global Registration},
  author={Choy, Christopher and Dong, Wei and Koltun, Vladlen},
  booktitle={CVPR},
  year={2020}
}

@inproceedings{choy2019fully,
  title = {Fully Convolutional Geometric Features},
  author = {Choy, Christopher and Park, Jaesik and Koltun, Vladlen},
  booktitle = {ICCV},
  year = {2019}
}

@inproceedings{choy20194d,
  title={4D Spatio-Temporal ConvNets: Minkowski Convolutional Neural Networks},
  author={Choy, Christopher and Gwak, JunYoung and Savarese, Silvio},
  booktitle={CVPR},
  year={2019}
}

@article{DBLP:journals/corr/abs-2103-12292,
  author       = {Zhicheng Zhou and
                  Cheng Zhao and
                  Daniel Adolfsson and
                  Songzhi Su and
                  Yang Gao and
                  Tom Duckett and
                  Li Sun},
  title        = {NDT-Transformer: Large-Scale 3D Point Cloud Localisation using the
                  Normal Distribution Transform Representation},
  journal      = {CoRR},
  volume       = {abs/2103.12292},
  year         = {2021},
  url          = {https://arxiv.org/abs/2103.12292},
  eprinttype    = {arXiv},
  eprint       = {2103.12292},
  timestamp    = {Tue, 08 Oct 2024 15:20:44 +0200},
  biburl       = {https://dblp.org/rec/journals/corr/abs-2103-12292.bib},
  bibsource    = {dblp computer science bibliography, https://dblp.org}
}
```
