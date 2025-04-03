import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathReader(Node):

    def __init__(self):
        super().__init__('path_reader')

        # The file where we want to save the path
        self.declare_parameter('output_file', 'path.txt')
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # The range of the poses in the path. The step we want to make between the points instead of collecting the whole path.
        self.declare_parameter('range', 1)
        self.range = self.get_parameter('range').get_parameter_value().string_value

        # Subscription to the topic where the Path is sended.
        self.subscription = self.create_subscription(
            Path,
            '/nuc_coverage_path', 
            self.listener_callback,
            10 
        )
        self.get_logger().info('Node subscribed to the topic /nuc_coverage_path')

    def listener_callback(self, msg):
        # Get all poses of the path
        poses = msg.poses

        with open(self.output_file, 'w') as file: 

            # We intercept to take the positions in a range
            for i in range(0, len(poses), self.range):
                pose_to_save = poses[i]

                # We save the pose in the txt file
                position = pose_to_save.pose.position
                file.write(f"{position.x} {position.y}\n")

        self.get_logger().info(f'Points saved in {self.output_file}')

def main(args=None):
    rclpy.init(args=args)
    path_reader = PathReader()
    rclpy.spin(path_reader)
    path_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
