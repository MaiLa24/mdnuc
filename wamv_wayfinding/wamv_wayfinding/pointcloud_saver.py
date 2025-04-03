import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
import math
from pyquaternion import Quaternion

LIDAR_ROLL = math.pi # Roll angle of the LiDAR
LIDAR_PITCH = math.pi/ 2 # Pitch angle of the LiDAR
LIDAR_YAW = 0 # YAW angle of the LiDAR


class PoincloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')

        self.pointcloud_data = np.empty((0, 3), dtype=float)

        # The file where we want to save the pointcloud
        self.declare_parameter('output_file', 'pointcloud.pcd')
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # Subscription to the desired topics: 
        self.subscription = self.create_subscription(
            PointCloud2,
            '/wamv/sensors/lidars/multibeam_sensor_lidar_wamv/points',
            self.lidar_callback,
            10)
        self.create_subscription(Bool, '/save_pointcloud', self.save_pointcloud_callback, 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/wamv/sensors/position/ground_truth_odometry',
            self.odom_callback,
            10
        )
        self.get_logger().info("Node subscribed to topics")

        # The matrix used to convert from the lidars coordinates system to the global coordinates system.
        self.T = None


    def odom_callback(self, msg):
        '''
         Callback to obtain robot's actual position and orientation on the map and calculate the transformation matrix.
        '''
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # The position of the robot
        t = np.array([position.x, position.y, 1]) # Z is always 1, because the water is at height 0.
        
        # The rotation of the robot
        q = Quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
        # Calculate the RPY of the robot
        yaw, pitch, roll = q.yaw_pitch_roll
        
        # Calculate the rotation matrix of the sensor and the robot
        R_lidar = np.dot(self.Rz(LIDAR_YAW), np.dot(self.Ry(LIDAR_PITCH), self.Rx(LIDAR_ROLL)))
        R_wamv = np.dot(self.Rz(yaw), np.dot(self.Ry(pitch), self.Rx(roll)))

        # Combine the rotation matrix for pitch and yaw
        R = np.dot(R_wamv, R_lidar)

        # Create the homogeneous matrix transformation.
        self.T = np.vstack((np.hstack((R, t.reshape(-1, 1))), np.array([0, 0, 0, 1])))

    def transform_coordinates(self, local_coords, T):
        '''
        Calculates the coordinate transformation from the local lidar coordinate system to the global coordinate system.
        '''
        # Convert local coordinates to homogeneous coordinates (add the ones column)
        local_coords_homogeneous = np.hstack((local_coords, np.ones((local_coords.shape[0], 1))))
        
        # Apply the transformation
        global_coords_homogeneous = local_coords_homogeneous.dot(T.T)
        # Delete the last column which is 1 
        global_coords = global_coords_homogeneous[:, :-1]
        
        return global_coords

    def Rx(self, roll):
        '''
        Returns the rotation matrix in the x axis
        '''
        return np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]
                        ])


    def Ry(self, pitch):
        '''
        Returns the rotation matrix in the y axis
        '''
        return np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]
                        ])


    def Rz(self, yaw):
        '''
        Returns the rotation matrix in the z axis
        '''
        return np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]
                        ])

    def lidar_callback(self, msg):
        '''
        Callback to obtain the information about the points detected by the lidar.
        It stores this information and accumulates it in a matrix that later will be saved as a point cloud file.
        '''
        if self.T is None:
            # We do not yet have a transformation matrix.
            return

        # We convert the message PointCloud2 to an array of numpy
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc_data = np.array([list(elem) for elem in pc_data])

        # Verify we got data from the LiDAR
        if pc_data.shape[0] == 0:
            self.get_logger().warn("No data from LIDAR")
            return     

        # Perform transformation of local coordinates to global coordinates
        pc_data = self.transform_coordinates(pc_data, self.T)

        # We accumulate the data from the point cloud
        self.pointcloud_data = np.vstack([self.pointcloud_data, pc_data])        
        self.get_logger().info(f"Accumulated data: {len(self.pointcloud_data)} points")


    def save_pointcloud_callback(self, msg: Bool):
        '''
        If any True message is received in this topic, we save the point cloud in a file.
        '''

        if msg.data:
        # If the bool message is True, we keep the point cloud
            self.get_logger().info("Saving point cloud to PCD file...")
            
            # Create an Open3D PointCloud object
            pcd = o3d.geometry.PointCloud()
            
            # We convert the numpy array to a Open3D PointCloud
            pcd.points = o3d.utility.Vector3dVector(self.pointcloud_data)
            
            # Save the PCD file
            o3d.io.write_point_cloud(self.output_file, pcd)
            self.get_logger().info("PCD file successfully saved.")
            
            # LWe clean the accumulated data.
            self.pointcloud_data = np.empty((0, 3), dtype=float)

def main(args=None):
    rclpy.init(args=args)
    pointcloud_bag_saver = PoincloudSaver()

    # Ejecutar el nodo y mantenerlo corriendo
    rclpy.spin(pointcloud_bag_saver)

    # Cerramos el nodo correctamente
    pointcloud_bag_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
