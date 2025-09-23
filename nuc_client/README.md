# Introduction

This package is meant to be used along with the package [nuc_ros2](../nuc_ros2) which is based on the package [nuc_ros2](https://github.com/ZJUTongYang/nuc_ros2).

In this package, you would find useful nodes to create the robot's path.


# Usage

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