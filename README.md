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

The `urdf` folder contains the model that is required for the controller in the wamv_wayfinding package to work properly.

The `meshes` folder contains a mesh to use in with the nuc_client package.

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

### nuc_client

This package is meant to be used along with the package [nuc_ros2](https://github.com/ZJUTongYang/nuc_ros2).

If we have the nuc_ros server operational, we can run the nuc_client node to send the desired mesh and get a path to that mesh. By default the mesh is `meshes/pasaia_seafloor_small.stl`:

```bash
ros2 run nuc_client nuc_client
```

If you want to specify the mesh to send:

```bash
ros2 run nuc_client nuc_client --ros-args -p mesh_path:=/path/to/mesh.stl
```

It is recommended the use of rosbag to save the message send by the server, since calculations can be very slow depending on the mesh size. The easiest way would be:

```bash
ros2 bag record /nuc_coverage_path
```

To transfer this Path class message to a txt file, the path_reader node has been developed. With this node you can indicate the desired step between each point from the beginning of the path. By default, the step would be 1 and the path would be saved in `path.txt`:

```bash
ros2 run nuc_client path_reader --ros-args -p output_file:=/path/to/file.txt range:=desired_range
```

### plugin_trajectory_following

This is a plugin that is based in the Gazebo TrajectoryFollower plugin. It has been modified in two aspects:

1. Instead of manually entering the waypoints one by one, a txt file is sent where each row indicates the coordinates of a waypoint.
2. When the robot reaches the last waypoint, a message is sent to t /save_pointcloud topic. This is used in the pointcloud_saver node.

The plugin must be indicated in the urdf file of the robot. For example:

```
  <gazebo>
    <plugin name="gz::sim::systems::MyTrajectoryFollower" filename="libMyTrajectoryFollowerPlugin.so">
      <link_name>wamv/base_link</link_name>
      <force>600</force>
      <torque>400</torque>
      <waypoints_file>/path/to/waypoints.txt</waypoints_file>
    </plugin>
  </gazebo>
```

In the `urdf` folder, you can find a robot with the plugin already defined. Maybe you need to change the path to the txt file.