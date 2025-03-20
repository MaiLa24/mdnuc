import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2


class PoincloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')

        self.pointcloud_data = np.empty((0, 3), dtype=float)

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
        q = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        
        # We transform the quartenion to euler (roll, pitch, yaw), but we only want to know yaw.
        _, _, yaw = self.quaternion_to_euler(q)
        
        # The rotation matrix for yaw
        R_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # The rotation matrix for pitch. It's always 90º anticlockwise.
        R_pitch = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])

        # Combine the rotation matrix for pitch and yaw
        R = np.dot(R_pitch, R_yaw)

        # Create the homogeneous matrix transformation.
        self.T = np.vstack((np.hstack((R, t.reshape(-1, 1))), np.array([0, 0, 0, 1])))

    def quaternion_to_euler(self, q):
        '''
        Converts a quaternion (x y z w) to Euler angles (roll pitch yaw)
        '''
        x, y, z, w = q
        
        # Roll (rotation around the X axis)

        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))
        
        # Pitch (rotation around the Y axis)
        pitch = np.arcsin(2.0 * (w * y - z * x))
        
        # Yaw (rotation around the Z axis)
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))
        
        return roll, pitch, yaw

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
            o3d.io.write_point_cloud("output.pcd", pcd)
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
