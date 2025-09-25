from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # PATH GENERATOR PARAMETERS
        DeclareLaunchArgument(
            'mesh',
            default_value='mesh.obj',
            description='The mesh file representing the environment. It should be a .obj file.'),
        DeclareLaunchArgument(
            'blocked_edges',
            default_value='""',
            description='The file containing the blocked edges. It should be a .txt file with each line formatted as "(x1, y1, z1) - (x2, y2, z2)". If you don\'t want to set any blocked edges, leave it empty.'),
        DeclareLaunchArgument(
            'door_edges',
            default_value='""',
            description='The file containing the door edges. It should be a .txt file with each line formatted as "(x1, y1, z1) - (x2, y2, z2)". If you don\'t want to set any door edges, leave it empty.'),
        DeclareLaunchArgument(
            'start_point',
            default_value='""',
            description='The starting point for the path. It should be a list of three floats representing the coordinates [x, y, z]. If you don\'t want to set a start point, leave it empty.'),
        DeclareLaunchArgument(
            'spacing',
            default_value='0.1',
            description='The spacing between waypoints in the generated path. It should be a float value representing the distance in meters.'),
        DeclareLaunchArgument(
            'output_file',
            default_value='smoothed_path.txt',
            description='The output file where the smoothed path will be saved. It should be a .txt file. The txt with the door information will be saved with the same name + _flag.'),
    
        Node(
            package='mdnuc_client', 
            executable='mdnuc_client',
            parameters=[{
                'mesh': LaunchConfiguration('mesh'),
                'blocked_edges': LaunchConfiguration('blocked_edges'),
                'door_edges': LaunchConfiguration('door_edges'),
                'start_point': LaunchConfiguration('start_point'),
                'spacing': LaunchConfiguration('spacing'),
                'output_file': LaunchConfiguration('output_file')
            }]
            ),

        Node(
            package='mdnuc_ros2',
            executable='nuc_node')
    
    ])

