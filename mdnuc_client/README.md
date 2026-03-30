# Introduction

This package is meant to be used along with the package [mdnuc_ros2](../mdnuc_ros2) which is based on the package [nuc_ros2](https://github.com/ZJUTongYang/nuc_ros2).

In this package, you would find useful nodes to create the robot's path.

# Commands

In the `launcher` folder you can found the launcher file to launch NUC and MDNUC path generation. The nodes are:

- mdnuc_client: The node client that sends to the server the data required for the path generation.
- mdnuc_ros2: The node server that calculates the path.

If you want to execute the launch file:

```bash
ros2 launch mdnuc_client mdnuc_path_generator.py
```

If you want to modify the values of any parameter:

```bash
ros2 launch wamv_wayfinding mdnuc_path_generator.py <parameter_name>:=<new_value> <parameter_name>:=<new_value> ... 
```

# Parameters

The parameters of mdnuc_client node can be changed in the command line. These are the different parameters:

- `mesh`: The mesh file representing the environment. It should be a .obj file. The default value is "mesh.obj".
- `blocked_edges`: The file containing the blocked edges. It should be a .txt file with each line formatted as "(x1, y1, z1) - (x2, y2, z2)". If you don\'t want to set any blocked edges, leave it empty. The default value is "".
- `door_edges`: The file containing the door edges. It should be a .txt file with each line formatted as "(x1, y1, z1) - (x2, y2, z2)". If you don\'t want to set any door edges, leave it empty. The default value is "".
- `start_point`: The starting point for the path. It should be a string of a list of three floats representing the coordinates [x, y, z]. If you don\'t want to set a start point, leave it empty. The default value is "".
- `spacing`: The spacing between waypoints in the generated path. It should be a float value representing the distance in meters. The default value is 0.1.
- `output_file`: The output file where the smoothed path will be saved. It should be a .txt file. The txt with the door information will be saved with the same name + _flag. The default value is smoothed_path.txt.

To generate the path with the original NUC method instead of using MDNUC, simply indicate both blocked_edges and door_edges as empty in the launcher parameters. This will produce the same result.

# Input files

All input files are obtained using the mesh_utils functions in the utils folder.

The mesh obj file expected by the node is obtained using the `plane_and_remesh` function. Do NOT send the original mesh, only the remeshed one.

To obtain the txt files with the blocked and door edges, use the following functions:

- `label_flattened_by_original_height` to obtain the submeshes for each depth range.
- `get_shared_edges_by_position` to obtain the sets with the blocked and door edges between two submeshes
- `save_shared_edges_to_txt` to save the sets in txt files.

# Output

In addition to returning two txt files, one with the smoothed path and the other with the flags for the trajectory follower plugin, the mdnuc_client node publishes in two topics:

`/mdnuc_original_path`: Publishes the original path.
`/mdnuc_smooth_path`: Publishes the smoothed path.

This way, the paths can be visualized in rviz.
