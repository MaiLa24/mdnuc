# Introduction

This is a plugin that is based in the Gazebo TrajectoryFollower plugin. It has been modified in two aspects:

1. Instead of manually entering the waypoints one by one, a txt file is sent where each row indicates the coordinates of a waypoint (x, y).
2. When the robot reaches the last waypoint, a message is sent to the /save_pointcloud topic. This is used in the pointcloud_saver node in the wamv_wayinding package.
3. There is a new parameter called `doors_file`. This parameter is used in the lidar_angle_controller node of the wamv_wayfinding package to notify when to change the LiDAR opening angle.

# Usage

The plugin must be indicated in the urdf file of the robot. For example:

```
  <gazebo>
    <plugin name="gz::sim::systems::MyTrajectoryFollower" filename="libMyTrajectoryFollowerPlugin.so">
      <link_name>wamv/base_link</link_name>
      <force>600</force>
      <torque>400</torque>
      <waypoints_file>/path/to/waypoints.txt</waypoints_file>
      <doors_file>/path/to/doors.txt</doors_file>
    </plugin>
  </gazebo>
```

In the `urdf` folder, you can find a robot with the plugin already defined. Maybe you need to change the paths to the txt files.

Both the file with the waypoints and the file with the doors are created in the nuc_client package.
