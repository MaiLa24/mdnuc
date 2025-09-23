# Introduction

In this package, you will find useful ROS2 nodes for obtaining information from VRX simulations. The ROS2 nodes are:

- Lidar angle controller: To dynamically change the opening angle of the LiDAR simulating a multibeam echo sounder during runtime.
- Pointcloud saver: To get information from the LiDAR. With this information, updates the grid of the mesh and saves the resulting pointcloud.
- Robot controller: To control the USV. This controlled has not been used in the paper.

# Commands

In the `launcher` folder you can found launcher files to launch multiple nodes at the same time.

- filtered_pointcloud_robot launches the 3 nodes.
- filtered_pointcloud launches pointcloud saver and lidar angle controller.
- pointcloud_robot launches pointcloud saver and robot controller.


If you want to execute a launch file:

```bash
ros2 launch wamv_wayfinding <launcher_name>.py
```

If you want to modify the values of any parameter:

```bash
ros2 launch wamv_wayfinding <launcher_name>.py <parameter_name>:=<new_value> <parameter_name>:=<new_value> ... 
```

If you only want to call a node:

```bash
ros2 run wamv_wayfinding <node_name>
```

If you want to modify the values of any parameter inside the node:

```bash
ros2 run wamv_wayfinding <node_name> --ros-args -p <parameter_name>:=<new_value> -p <parameter_name>:=<new_value> ... 
```


# Parameters

The parameters of each node can be modified when the command is called, either from the launcher or individually.

## Lidar angle controller:

This node has 4 parameters:

- `num_beams`: The number of beams to keep in the filtered PointCloud2. It needs to be an integer. The default value is 256.
- `desired_angle_deg`: The desired angle in degrees at the start of execution (if 0.0, it will be set dynamically). It needs to be a float. The default value is 0.0.
- `distance_ranges`: The distance (depth) ranges as a list of tuples in string format. The default value is ["(0.0, 10.0)", "(10.0, 20.0)"].
- `angles_deg`: The angles corresponding to each distance range as a list of floats. The default value is [10.0, 30.0]. The length of the array must be equal to the distance_ranges.

## Pointcloud saver:

This node has 3 parameters:

- `pointcloud_waypoint_file`: The output file where the pointcloud will be saved. It should be a .csv file. The default value is 'pointcloud.csv'. The csv file will have 4 columns: x, y, z, and timestamp.
- `grid_file`: The grid file where the coverage and overlapping data will be saved. It should be a .csv file. If you do not want to save this data, leave it empty (""). To create a grid file, use the functions in `utils/mesh_utils.py`. The default value is "".
- `desired_pointcloud_topic`: The PointCloud2 topic to be subscribed to. It should be a PointCloud2 message. The default value is the topic published by lidar_angle_controller, except in the pointcloud_robot launcher, where the default value is the VRX wamv topic /wamv/sensors/lidars/multibeam_sensor_lidar_wamv/points.

## Robot controller:

This node only has 1 parameter: `waypoint_file`.

It's default value is 'waypoints.txt'.

The `waypoints.txt` file that you can find in the root directory is an example of the type of file that the node expects. When the robot visits the last waypoint, it will stop moving.

# Other info

To save the data from pointcloud_saver node on a PCD file:

```bash
ros2 topic pub --once /save_pointcloud std_msgs/msg/Bool "data: true"
```

If you use the trajectory follower plugin, this would be done automatically when the USV reaches the final waypoint.