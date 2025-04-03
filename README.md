# ros2_packages_vrx

This repository contains the ros2 packages created for VRX.

They were created using ROS2 Humble.

## TODO list
- [ ] Connect 'robot_controller' and 'pointcloud_saver' nodes.
- [ ] Improve the rotation of the robot.
- [ ] Launcher.
- [ ] Add options to the commandline (txt file, pcd file)
  
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

The `urdf` folder contains the model that is required for the controller to work properly.

The file `waypoints.txt` is an example of a file with waypoints for the path following. For each line you have to write the `x` and `y` of the coordinates where the waypoint is located.

## Usage

### wamv_wayfinding package 

If you want to follow the path of the waypoints defined in the txt file:

```bash
ros2 run wamv_wayfinding robot_controller 
```

When the robot visits the last waypoint, it will stop moving.


If you want start accumulating the LiDAR data to get a point cloud from the seafloor:

```bash
ros2 run wamv_wayfinding pointcloud_saver 
```

To save the data on a PCD file:

```bash
ros2 topic pub --once /save_pointcloud std_msgs/msg/Bool "data: true"
```

By default, the data will be saved in the `pointcloud.pcd` file.

If you want to specify the file to save the data:

```bash
ros2 run wamv_wayfinding pointcloud_saver --ros-args -p output_file:=/path/to/file.pcd
```

