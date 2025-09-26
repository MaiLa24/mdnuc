#!/usr/bin/python3

import sys
import ast
import trimesh
import rclpy
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point, PoseStamped
from mdnuc_msgs.srv import GetNuc, GetNucWithGivenStart
from mdnuc_msgs.msg import PointArray
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time
from std_msgs.msg import UInt8MultiArray
import numpy as np
from scipy.interpolate import make_interp_spline

class MDNUCClient(Node):

    def __init__(self):
        super().__init__('mdnuc_client')

        # Parameters
        self.declare_parameter('mesh', "mesh.obj")
        self.mesh = self.get_parameter('mesh').get_parameter_value().string_value

        self.declare_parameter('blocked_edges', "blocked_edges.txt")
        blocked_edges_file = self.get_parameter('blocked_edges').get_parameter_value().string_value

        self.declare_parameter('door_edges', "door_edges.txt")
        door_edges_file = self.get_parameter('door_edges').get_parameter_value().string_value

        if blocked_edges_file == "" or door_edges_file == "":
            self.get_logger().error("blocked_edges and door_edges not provided. Generating path with NUC.")
            self.blocked_edges = set()
            self.door_edges = set()
        else:
            self.get_logger().info(f"Loading blocked edges from {blocked_edges_file} and door edges from {door_edges_file}")
            self.get_logger().info("Generating path with MDNUC.")
            self.blocked_edges = load_shared_edges_from_txt(blocked_edges_file)
            self.door_edges = load_shared_edges_from_txt(door_edges_file)

        self.declare_parameter('start_point', "")
        self.start_point = self.get_parameter('start_point').get_parameter_value().string_value

        self.declare_parameter('spacing', 0.1)
        self.spacing = self.get_parameter('spacing').get_parameter_value().double_value

        self.declare_parameter('output_file', "smoothed_path.txt")
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        if self.start_point != "":
            # Parse the start_point parameter safely. Accept formats like:
            # "(x, y, z)", "[x, y, z]", "x, y, z" or "x,y,z"
            parsed = None
            try:
                parsed = ast.literal_eval(self.start_point)
            except Exception:
                # Try forgiving comma-separated parse as a fallback
                try:
                    cleaned = self.start_point.strip().lstrip('(').rstrip(')')
                    parts = [p.strip() for p in cleaned.split(',') if p.strip() != '']
                    parts = [float(p) for p in parts]
                    parsed = tuple(parts)
                except Exception as e:
                    self.get_logger().error(f"Could not parse start_point parameter '{self.start_point}': {e}")

            # Validate parsed value is a 3-element numeric sequence
            if parsed is not None and isinstance(parsed, (list, tuple)) and len(parsed) == 3:
                try:
                    self.start_point = (float(parsed[0]), float(parsed[1]), float(parsed[2]))
                    self.start_point_msg = PoseStamped()
                    self.start_point_msg.header.frame_id = 'map'
                    self.start_point_msg.pose.position.x = self.start_point[0]
                    self.start_point_msg.pose.position.y = self.start_point[1]
                    self.start_point_msg.pose.position.z = self.start_point[2]
                    self.start_point_msg.pose.orientation.w = 1.0

                    self.client = self.create_client(GetNucWithGivenStart, 'get_nuc_with_start')
                    # Wait till the service is available
                    while not self.client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('Service not available, waiting...')
                    self.request = GetNucWithGivenStart.Request()
                except Exception as e:
                    self.get_logger().error(f'Invalid numeric values for start_point: {e}')
                    # Fall back to no start point
                    self.start_point_msg = None
                    self.client = self.create_client(GetNuc, "get_nuc")
                    while not self.client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('Service not available, waiting...')
                    self.request = GetNuc.Request()
            else:
                self.get_logger().error("start_point parameter must be a 3-element sequence; ignoring and using default service.")
                self.start_point_msg = None
                self.client = self.create_client(GetNuc, "get_nuc")
                # Wait till the service is available
                while not self.client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Service not available, waiting...')
                self.request = GetNuc.Request()
        else:
            self.start_point_msg = None
            self.client = self.create_client(GetNuc, "get_nuc")
            # Wait till the service is available
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting...')
            self.request = GetNuc.Request()
        
        # Publishers
        self.path_publisher = self.create_publisher(Path, '/mdnuc_original_path', 10)
        self.smooth_publisher = self.create_publisher(Path, '/mdnuc_smooth_path', 10)

        self.mesh_data = trimesh.load(self.mesh)

        self.send_request()


    def create_mesh_request(self):
        # Create the Mesh message
        mesh_msg = Mesh()

        # Filling the vertices (vertices)
        for vertex in self.mesh_data.vertices:
            point = Point()
            point.x = vertex[0]
            point.y = vertex[1]
            point.z = vertex[2]
            mesh_msg.vertices.append(point)

        # Fill in the faces (triangles)
        for face in self.mesh_data.faces:
            tri = MeshTriangle()
            tri.vertex_indices[0] = face[0]
            tri.vertex_indices[1] = face[1]
            tri.vertex_indices[2] = face[2]
            mesh_msg.triangles.append(tri)

        self.get_logger().info("Mesh msg created")
        self.get_logger().info(f'Mesh with {len(mesh_msg.vertices)} vertices and {len(mesh_msg.triangles)} triangles.')

        # Map the Mesh to the appropriate frame in the request
        self.request.mesh = mesh_msg
        self.request.frame_id = "map"

        shared_edges_msg = []
        self.get_logger().info(self.blocked_edges)
        for edge in self.blocked_edges:
            v1, v2 = list(edge)
            pt1 = Point(x=v1[0], y=v1[1], z=v1[2])
            pt2 = Point(x=v2[0], y=v2[1], z=v2[2])
            
            pa = PointArray()
            pa.p1 = pt1
            pa.p2 = pt2

            shared_edges_msg.append(pa)
        self.request.shared_edges = shared_edges_msg

        door_edges_msg = []
        for edge in self.door_edges:
            v1, v2 = list(edge)
            pt1 = Point(x=v1[0], y=v1[1], z=v1[2])
            pt2 = Point(x=v2[0], y=v2[1], z=v2[2])
            
            pa = PointArray()
            pa.p1 = pt1
            pa.p2 = pt2

            door_edges_msg.append(pa)
        self.request.door_edges = door_edges_msg

        if self.start_point_msg != None:
            self.request.start_pose = self.start_point_msg

    def send_request(self):
        self.create_mesh_request()

        # Calling the service asynchronously
        future = self.client.call_async(self.request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            msg = UInt8MultiArray()
            msg.data = response.is_door_path
            self.publish_path(response.coverage)
            self.smooth_path(response.coverage, response.is_door_path)

        except Exception as e:
            self.get_logger().error(f'Error when calling for service: {e}')

    def extract_path_points(self, path_poses):
        x = [pose.pose.position.x for pose in path_poses]
        y = [pose.pose.position.y for pose in path_poses]
        if len(path_poses) == 3:
            x.append( path_poses[-1].pose.position.x)
            y.append( path_poses[-1].pose.position.y)
        else:
            x.append( path_poses[0].pose.position.x)
            y.append( path_poses[0].pose.position.y)
        return np.array(x), np.array(y)
        
    def publish_path(self, coverage):
        # Create the Path message and fill with the received Path data
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        # Copy points from Path (poses) to Path message
        path_msg.poses = coverage.poses

        # Publish Path in the topic
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f'Publishing Path with {len(path_msg.poses)} poses.')

    def smooth_path_interp(self, x, y, degree=3):
        t = np.linspace(0, 1, len(x))
        spline_x = make_interp_spline(t, x, k=degree)
        spline_y = make_interp_spline(t, y, k=degree)

        t_dense = np.linspace(0, 1, 1000)
        x_dense = spline_x(t_dense)
        y_dense = spline_y(t_dense)
        points_dense = np.stack((x_dense, y_dense), axis=-1)

        deltas = np.diff(points_dense, axis=0)
        dists = np.linalg.norm(deltas, axis=1)
        cumdist = np.insert(np.cumsum(dists), 0, 0)
        total_length = cumdist[-1]

        num_points = int(np.floor(total_length / self.spacing)) + 1
        target_dists = np.linspace(0, self.spacing * (num_points - 1), num_points)

        t_new = np.interp(target_dists, cumdist, t_dense)

        x_smooth = spline_x(t_new)
        y_smooth = spline_y(t_new)

        return x_smooth, y_smooth

    def create_path_msg(self, x_smooth, y_smooth, frame_id='map', stamp=None):
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = stamp if stamp else self.get_clock().now().to_msg()

        for xi, yi in zip(x_smooth, y_smooth):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = xi
            pose.pose.position.y = yi
            pose.pose.orientation.w = 1.0  # orientación neutral
            path_msg.poses.append(pose)

        return path_msg

    def smooth_path(self, coverage, door_path):
        # Get all the poses from the path
        x, y = self.extract_path_points(coverage.poses)

        # Door information
        door_info = list(door_path)

        # Each point that is True in door_info is a door point
        true_indices = [i for i, flag in enumerate(door_info) if flag]
        # With these indices, get the points
        true_points = np.array([[x[i], y[i]] for i in true_indices])

        x_smooth, y_smooth = self.smooth_path_interp(x, y)
        smooth_path_msg = self.create_path_msg(x_smooth, y_smooth)

        self.smooth_publisher.publish(smooth_path_msg)
        poses = smooth_path_msg.poses

        # Compare each smoothed point with the original `True` points and mark the closest one.
        smooth_points = np.stack((x_smooth, y_smooth), axis=-1)
        new_flags = np.zeros(len(smooth_points), dtype=np.uint8)

        for pt in true_points:
            dists = np.linalg.norm(smooth_points - pt, axis=1)
            closest_index = np.argmin(dists)
            new_flags[closest_index] = 1

        # We save the smoothed path 
        with open(self.output_file, 'w') as file: 
            for i in range(0, len(poses), 1):
                pose_to_save = poses[i]
                position = pose_to_save.pose.position
                file.write(f"{position.x} {position.y}\n")

        # We save the flags in a separate file
        flags_file = self.output_file.replace('.txt', '_flags.txt')
        print(flags_file)
        with open(flags_file, 'w') as file:
            for flag in new_flags:
                file.write(f"{flag}\n")
        
        self.get_logger().info(f'Smoothed path saved in {self.output_file}')


def load_shared_edges_from_txt(filename="shared_edges.txt"):
    """
    Lee un archivo de aristas guardadas en formato:
    (x1, y1, z1) - (x2, y2, z2)
    y devuelve un set de frozensets con las posiciones como tuplas de floats.
    """
    shared_edges = set()

    with open(filename, "r") as f:
        for line in f:
            # Remove spaces and separate with “ - ”
            part1, part2 = line.strip().split(" - ")

            # Remove parentheses and convert to float
            v1 = tuple(map(float, part1.strip("()").split(",")))
            v2 = tuple(map(float, part2.strip("()").split(",")))

            shared_edges.add(frozenset([v1, v2]))

    return shared_edges


def main(args=None):

    rclpy.init(args=args)

    # Create the node
    mdnuc_client = MDNUCClient()

    rclpy.spin(mdnuc_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
