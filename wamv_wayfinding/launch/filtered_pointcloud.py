from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # LIDAR ANGLE CONTROLLER PARAMETERS
        DeclareLaunchArgument(
            'num_beams',
            default_value='256',
            description='The number of beams to keep in the filtered PointCloud2.'),
        DeclareLaunchArgument(
            'desired_angle_deg',
            default_value='0.0',
            description='The desired angle in degrees at the start of execution (if 0.0, it will be set dynamically).'),
        DeclareLaunchArgument(
            'distance_ranges',
            default_value='["(0.0, 10.0)", "(10.0, 20.0)"]',
            description='The distance ranges as a list of tuples in string format.'),
        DeclareLaunchArgument(
            'angles_deg',
            default_value='[10.0, 30.0]',
            description='The angles corresponding to each distance range as a list of floats.'),

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
            executable='lidar_angle_controller',
            parameters=[LaunchConfiguration('num_beams'), LaunchConfiguration('desired_angle_deg'), LaunchConfiguration('distance_ranges'), 
                        LaunchConfiguration('angles_deg')]
            ),
        Node(
            package='wamv_wayfinding', 
            executable='pointcloud_saver',
            parameters=[LaunchConfiguration('pointcloud_waypoint_file'), LaunchConfiguration('grid_file'), LaunchConfiguration('desired_pointcloud_topic')]
            )
        
        

    ])