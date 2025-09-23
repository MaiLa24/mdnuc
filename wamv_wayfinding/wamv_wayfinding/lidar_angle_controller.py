#!/usr/bin/env python3
import rclpy
import struct
import numpy as np
import ast
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool  
from laser_geometry import LaserProjection

class DynamicScanFilter(Node):
    def __init__(self):
        super().__init__('dynamic_scan_filter')

        # Number of beams to keep in the filtered PointCloud2
        self.declare_parameter('num_beams', 256)
        self.num_beams = self.get_parameter('num_beams').get_parameter_value().integer_value

        # Desired angle in degrees (if None, it will be set dynamically)
        self.declare_parameter('desired_angle_deg', 0.0)
        self.desired_angle_deg = self.get_parameter('desired_angle_deg').get_parameter_value().double_value

        # The distance ranges
        self.declare_parameter('distance_ranges', ["(0.0, 10.0)", "(10.0, 20.0)"])
        distance_ranges_string = self.get_parameter('distance_ranges').get_parameter_value().string_array_value
        self.distance_ranges = [ast.literal_eval(item) for item in distance_ranges_string]

        # The angles corresponding to each distance range
        self.declare_parameter('angles_deg', [10.0, 30.0])
        self.angles_deg = self.get_parameter('angles_deg').get_parameter_value().double_array_value

        if len(self.distance_ranges) != len(self.angles_deg):
            self.get_logger().error("The length of distance_ranges must be equal to the length of angles_deg")
            raise ValueError("The length of distance_ranges must be equal to the length of angles_deg")
        
        # The boolean that indicates whether the angle should be changed in the next reading
        if self.desired_angle_deg == 0.0:
            self.change_angle = True
            self.last_distance_range = ()
        else:
            self.change_angle = False
            if self.desired_angle_deg in self.angles_deg:
                self.last_distance_range = self.distance_ranges[self.angles_deg.index(self.desired_angle_deg)]

    
        # Publish the filtered PointCloud2 message
        self.pub = self.create_publisher(PointCloud2, '/filtered_pointcloud', 10)
        
        # Subscribe to the LaserScan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/wamv/sensors/lidars/multibeam_sensor_lidar_wamv/scan', 
            self.scan_callback, 
            10
        )
        
        # Subscribe to the trigger topic from the trajectory follower plugin
        self.door_crossed = self.create_subscription(
            Bool,
            '/door_crossed',
            self.trigger_callback,
            1
        )

        # Object to convert from LaserScan to PointCloud2
        self.laser_projector = LaserProjection()
        
        self.get_logger().info("Dynamic LIDAR filter node started.")
        self.get_logger().info("Listening for trigger messages on '/door_crossed'...")

    def trigger_callback(self, msg):
        """
        Callback that is activated when the plugin's trigger message is received.
        When the message is received, the boolean is updated to update the sensor's opening angle in the next reading.
        """
        self.change_angle = True
        self.get_logger().info("Trigger message received. Adjusting opening angle...")
        

    def scan_callback(self, msg):
        """
        Main callback that filters the LaserScan.
        First, it checks whether the sensor's opening angle needs to be modified. If so, it updates to the new value. 
        After that, it filters the points to return a PointCloud2 message with only the desired points.
        """

        if self.change_angle:
            # Step 1: Obtain the central index of the scan
            center_index = len(msg.ranges) // 2
            center_distance = msg.ranges[center_index]

            # Verify if the central reading is valid
            if center_distance < msg.range_min or center_distance > msg.range_max:
                print("The central reading isn't valid.")
            else:
                print(f"Center distance: {center_distance:.2f} meters")

                # Step 2: Find which range center_distance belongs to
                current_range_idx = None
                for i, (min_dist, max_dist) in enumerate(self.distance_ranges):
                    if min_dist <= center_distance < max_dist:
                        current_range_idx = i
                        break

                if current_range_idx is None:
                    self.get_logger().warn(f"center_distance {center_distance:.2f} is out of defined ranges!")
                    return

                current_range = self.distance_ranges[current_range_idx]

                # If the current range is the same as the last one, find the closest different range
                if current_range == self.last_distance_range:
                    min_distance = float('inf')
                    new_range_idx = current_range_idx 

                    for i, (min_dist, max_dist) in enumerate(self.distance_ranges):
                        if self.distance_ranges[i] == self.last_distance_range:
                            continue  # Ignore same range

                        # Compare the distance from center_distance to the edge of the range
                        if center_distance < min_dist:
                            distance = min_dist - center_distance
                        elif center_distance >= max_dist:
                            distance = center_distance - max_dist
                        else:
                            # If center_distance falls outside the range (possible due to float inf), use distance to nearest edge
                            distance = min(center_distance - min_dist, max_dist - center_distance)

                        if distance < min_distance:
                            min_distance = distance
                            new_range_idx = i

                    # Update to the nearest different range
                    self.last_distance_range = self.distance_ranges[new_range_idx]
                    self.desired_angle_deg = self.angles_deg[new_range_idx]
                    self.get_logger().info(
                        f"center_distance {center_distance:.2f} in same range {current_range}, switching to closest different range {self.last_distance_range}, angle: {self.desired_angle_deg:.2f}°"
                    )

                else:
                    # current_range is different from last_distance_range, so we can update normally
                    self.last_distance_range = current_range
                    self.desired_angle_deg = self.angles_deg[current_range_idx]
                    self.get_logger().info(
                        f"center_distance {center_distance:.2f} -> new range {current_range}, angle: {self.desired_angle_deg:.2f}°"
                    )

                self.change_angle = False




        # FIlter the scan based on the desired angle
        # Step 1: Convert the entire LaserScan to PointCloud2
        try:
            cloud_out = self.laser_projector.projectLaser(msg)
        except Exception as e:
            self.get_logger().error(f"Error projecting laser scan: {e}")
            return

        # Step 2: Filter the PointCloud2 according to desired_angle_deg
        angle_filtered_points = []
        angle_rad = np.radians(self.desired_angle_deg)
        half_angle_rad = angle_rad / 2.0
        
        # Unpack the points from the PointCloud2 message
        points_data = cloud_out.data
        point_step = cloud_out.point_step
        
        for i in range(0, len(points_data), point_step):
            # Unpack x, y, z coordinates
            x, y, z = struct.unpack('<fff', points_data[i:i+12])
            
            # Calculate the angle of the point in the XY plane
            point_angle = np.arctan2(y, x)
            
            # If the point is within the desired angle, keep it
            if abs(point_angle) <= half_angle_rad:
                angle_filtered_points.append({
                    'x': x,
                    'y': y,
                    'z': z,
                    'angle': point_angle,
                    'data': points_data[i:i+point_step]
                })

        # Step 3: Sort the points by angle to filter the desired number of beams.
        angle_filtered_points.sort(key=lambda p: p['angle'])

        # Step 4: Apply equiangular filter (n beams within the angle)
        final_filtered_data = []

        if len(angle_filtered_points) == 0:
            self.get_logger().warn("No points found within desired angle range.")
        else:
            total_points = len(angle_filtered_points)

            # Select points equiangularly, using a maximum of total_points
            num_selected_points = min(self.num_beams, total_points)
            indices = np.linspace(0, total_points - 1, num_selected_points, dtype=int)
            final_filtered_data = [angle_filtered_points[i]['data'] for i in indices]


        # Step 5: Create and publish the new filtered PointCloud2 message
        filtered_cloud = PointCloud2()
        filtered_cloud.header = cloud_out.header
        filtered_cloud.height = 1
        filtered_cloud.width = len(final_filtered_data)
        filtered_cloud.is_dense = True
        filtered_cloud.point_step = point_step
        filtered_cloud.row_step = point_step * filtered_cloud.width
        filtered_cloud.fields = cloud_out.fields
        filtered_cloud.data = b''.join(final_filtered_data)

        self.pub.publish(filtered_cloud)
        
def main(args=None):
    rclpy.init(args=args)
    node = DynamicScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()