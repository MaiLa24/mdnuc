#!/usr/bin/python3

import sys
import trimesh
import rclpy
from rclpy.node import Node
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point
from nuc_msgs.srv import GetNuc
from nav_msgs.msg import Path

class NucClient(Node):

    def __init__(self):
        super().__init__('nuc_client')

        # The file where we want to save the path
        self.declare_parameter('mesh_path', "meshes/pasaia_seafloor_small.stl")
        self.mesh_path = self.get_parameter('mesh_path').get_parameter_value().string_value



        self.client = self.create_client(GetNuc, 'get_nuc')
        
        # Publishe
        self.path_publisher = self.create_publisher(Path, '/nuc_coverage_path', 10)


        # Wait till the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.request = GetNuc.Request()

    def create_mesh_request(self, mesh_data):
        # Create the Mesh message
        mesh_msg = Mesh()

        # Filling the vertices (vertices)
        for vertex in mesh_data.vertices:
            point = Point()
            point.x = vertex[0]
            point.y = vertex[1]
            point.z = vertex[2]
            mesh_msg.vertices.append(point)

        # Filling the faces (triangles)
        for face in mesh_data.faces:
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

    def send_request(self):

        mesh_data = trimesh.load_mesh(mesh_path)
        self.create_mesh_request(mesh_data)

        # Calling the service asynchronously
        future = self.client.call_async(self.request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.publish_path(response.coverage)

        except Exception as e:
            self.get_logger().error(f'Error when calling for service: {e}')

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


def main(args=None):

    rclpy.init(args=args)

    # Crete the node
    nuc_client = NucClient()

    nuc_client.send_request()

    rclpy.spin(nuc_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
