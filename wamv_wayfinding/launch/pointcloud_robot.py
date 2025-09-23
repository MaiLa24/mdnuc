from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ROBOT CONTROLLER PARAMETERS
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='waypoints.txt',
            description='The file where the waypoints are stored. It should be a .txt file.'),
        
        # POINTCLOUD SAVER PARAMETERS
        DeclareLaunchArgument(
            'pointcloud_waypoint_file',
            default_value='pointcloud.csv',
            description='The output file where the pointcloud will be saved. It should be a .csv file.'),
        DeclareLaunchArgument(
            'grid_file',
            default_value="",
            description='The grid file where the coverage and overlapping data will be saved. It should be a .csv file. If you do not want to save this data, leave it empty.'),
        DeclareLaunchArgument(
            'desired_pointcloud_topic',
            default_value='/wamv/sensors/lidars/multibeam_sensor_lidar_wamv/points',
            description='The PointCloud2 topic to be subscribed to. It should be a PointCloud2 message. The default topic is the one from the VRX wamv robot LiDAR sensor. If you want the filtered topic, use \\filtered_pointcloud.'),

        Node(
            package='wamv_wayfinding', 
            executable='robot_controller',
            parameters=[LaunchConfiguration('waypoint_file')]
            ),

        Node(
            package='wamv_wayfinding', 
            executable='pointcloud_saver',
            parameters=[LaunchConfiguration('pointcloud_waypoint_file'), LaunchConfiguration('grid_file'), LaunchConfiguration('desired_pointcloud_topic')]
            )
        
        

    ])