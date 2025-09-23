# ros2_packages_vrx

This repository contains the ros2 packages created for VRX.

They were created using ROS2 Humble.

## Table of contents

- [ros2\_packages\_vrx](#ros2_packages_vrx)
  - [Table of contents](#table-of-contents)
  - [TODO list](#todo-list)
  - [Installation](#installation)
  - [Before starting](#before-starting)
  - [Usage](#usage)
    - [lidar\_config](#lidar_config)
    - [Packages](#packages)
      - [wamv\_wayfinding package](#wamv_wayfinding-package)
      - [nuc\_client](#nuc_client)
      - [plugin\_trajectory\_following](#plugin_trajectory_following)
      - [utils](#utils)


## TODO list
- [ ] Connect 'robot_controller' and 'pointcloud_saver' nodes.
- [ ] Improve the rotation of the robot.
- [ ] Launcher.
- [ ] Add options to the commandline (txt file, pcd file)
- [x] Update urdf with the new trajectory follower.
- [ ] Update nuc_client
  - [ ] Add smoother
  - [ ] Add mdbaf
  - [ ] Add walls and doors
- [x] Update wamv_wayfinding
  - [x] Add lidar_filter
  - [x] Update pointlocud saver
- [ ] Add nuc_ros2 modified (mdnuc_ros2)
- [x] Update trajectory follower
- [ ] Update utils to generate walls and doors
  
## Installation

First, clone the repository to your local machine:

```bash
git clone https://github.com/MaiLa24/ros2_packages_vrx.git
cd ros2_packages_vrx
```

Then compile the packages:

```bash
colcon build
```

Once the build is complete, source the workspace to add the environment variables to your session:

```bash
source install/setup.bash
```

## Before starting

The `urdf` folder contains the model used during the paper. The WAMV is equipped with a LiDAR system featuring an 180-degree aperture angle and 2,500 beams. This is done so that it can be used with the lidar_angle_controller node from the wams_wayfinding package. The LiDAR update rate of 4 is ideal for obtaining approximately one reading every 10 centimeters of travel, as long as the trajectory follower speed parameters are not changed. The URDF contains an example of how to call the trajectory follower plugin.

The `meshes` folder contains a mesh to use in with the nuc_client package.

The file `waypoints.txt` is an example of a file with waypoints for the path following. For each line you have to write the `x` and `y` of the coordinates where the waypoint is located.

## Usage

### lidar_config

In the `lidar_config` folder, you can found 4 files. The `lidar_config.yaml` file is an example of how to define a lidar that returns 50 points and has a width of 5 degrees.

The other 3 files must replace the corresponding file in the folder where the VRX repository was saved. The paths to the files are as follows:

- `lidar.xacro`: /path/to/vrx/vrx_urdf/wamv_gazebo/urdf/components/lidar.xacro
- `numeric.yaml`: /path/to/vrx/vrx_urdf/vrx_gazebo/config/wamv_config/component_compliance/numeric.yaml
- `wamv_planar_lidar.xacro`: /path/to/vrx/vrx_urdf/wamv_gazebo/urdf/components/wamv_planar_lidar.xacro

Once the files are replaced, you can run the VRX `generate_wamv.launch.py` script with the new lidar configuration to obtain a lidar with a single line of points, with the desired angle and samples. In the [VRX tutorial](https://github.com/osrf/vrx/wiki/generate_wamv_tutorial) you can find an example of how to use this script.

### Packages

Each package will have each Readme explaining in more detail the usage.

#### wamv_wayfinding package 

In this package, you will find useful ROS2 nodes for obtaining information from VRX simulations. The ROS2 nodes are:

- Lidar angle controller: To dynamically change the opening angle of the LiDAR simulating a multibeam echo sounder during runtime.
- Pointcloud saver: To get information from the LiDAR simulating a single-beam sonar. With this information, updates the grid of the mesh and saves the resulting pointcloud.
- Robot controller: To control the USV.

#### nuc_client

This package is meant to be used along with the package [mdnuc_ros2](./mdnuc_ros2) which is based on the package [nuc_ros2](https://github.com/ZJUTongYang/nuc_ros2).

In this package, you would find useful nodes to create the robot's path.

#### plugin_trajectory_following

This is a plugin that is based in the Gazebo TrajectoryFollower plugin. It has been modified in two aspects:

1. Instead of manually entering the waypoints one by one, a txt file is sent where each row indicates the coordinates of a waypoint (x, y).
2. When the robot reaches the last waypoint, a message is sent to the /save_pointcloud topic. This is used in the pointcloud_saver node in the wamv_wayinding package.
3. There is a new parameter called `doors_file`. This parameter is used in the lidar_angle_controller node of the wamv_wayfinding package to notify when to change the LiDAR opening angle.

#### utils

This folder contains utility Python scripts and helper functions that support the main functionality of the project. These scripts are not ROS nodes, but they provide reusable tools for tasks such as mesh processing and grid generation. You can import these utilities in your ROS nodes or use them as standalone scripts to streamline development and testing.