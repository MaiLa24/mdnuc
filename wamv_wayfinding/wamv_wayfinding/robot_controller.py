import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

THETA_THRESHOLD = 0.15      # Threshold of the angle difference between the current and desired orientation of the robot
DISTANCE_THRESHOLD = 1.5    # Threshold of the distance between the robot and the waypoint
ROTATION_THRESHOLD = 4.0    # Threshold of the distance between the robot and the waypoint where the robot stops trying to rotate
VELOCITY = 200.0            # Velocity of the robot when moving forward
TIMER_PERIOD = 0.2          # The period (s) of the timer that calls the function move_to_point()


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Declare the file parameter with the waypoints
        self.declare_parameter('waypoint_file', 'waypoints.txt')
        self.file_path = self.get_parameter('waypoint_file').get_parameter_value().string_value

        # Subscribe to the odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/wamv/sensors/position/ground_truth_odometry',
            self.odom_callback,
            10
        )
        
        # Thrust publishers
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        
        # USV info
        self.cur_x = None
        self.cur_y = None
        self.cur_theta = None

        # Waypoints info
        self.wp_count = 0
        self.wps_pos_x = []
        self.wps_pos_y = []

        # Call to the function to read the waypoints from the text file
        self.read_waypoints_from_file(self.file_path)
        self.get_logger().info("Waypoints read")

        # Define the index of the next waypoint and create a timer to call to the function for robot movement
        self.wp_index = 0 
        self.timer = self.create_timer(TIMER_PERIOD, self.move_to_point)

    def odom_callback(self, msg: Odometry):
        '''
        Callback to obtain robot's actual position and orientation on the map.
        '''
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        # Calculate the robot's yaw orientation in radians.
        self.cur_theta = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))

        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

    def move_to_point(self):
        '''
        The main function of the node. It's called periodically. It determines the next point to move and controls the robot's movement.
        First turns to the direction of the waypoint, then it goes forward until it reaches the desired position.
        '''
        if self.wp_index < self.wp_count:
            # If there is still waypoints to visit.

            if self.cur_x is None or self.cur_y is None or self.cur_theta is None:
                # If there is still no information about the position of the robot, return.
                self.get_logger().warn("Waiting odometry info...")
                return

            # Calculate the angle between the robot and the waypoint
            delta_x = self.wps_pos_x[self.wp_index] - self.cur_x
            delta_y = self.wps_pos_y[self.wp_index] - self.cur_y
            desired_angle = math.atan2(delta_y, delta_x)
            
            # Calculate the difference between the desired and the actual orientation.
            angle_diff = self.calculate_angular_difference(self.cur_theta, desired_angle)

            # Calculate the distance between the robot and the waypoint.
            distance_to_target = math.sqrt((self.wps_pos_x[self.wp_index] - self.cur_x)**2 + (self.wps_pos_y[self.wp_index] - self.cur_y)**2)

            if distance_to_target > DISTANCE_THRESHOLD:
                # If the robot is not close enough to the waypoint.
                if abs(angle_diff) > THETA_THRESHOLD and distance_to_target > ROTATION_THRESHOLD:
                    # If the robot orientation is not close enough to the desired orientation, the robot rotates.
                    self.get_logger().info("Rotating...")
                    self.turn_to_angle(angle_diff)
                else:
                    # If the robot orientation is close enough to the desired orientation, the robot moves forward.
                    self.get_logger().info("Moving forward...")
                    self.move_forward()
            else:
                # The robot got to the desired location. It stops and changes the waypoint index to start searching the next point.
                self.get_logger().info("Waypoint visited")
                self.left_thrust_pub.publish(Float64(data=0.0))
                self.right_thrust_pub.publish(Float64(data=0.0))
                self.wp_index += 1
        else:
            # All the waypoints have been visited.
            self.get_logger().info("All waypoints visited")

    def turn_to_angle(self, angle_diff):
        '''
        This function is designed to get the angle difference between the actual and desired orientation of the robot and make it
        turn right or left.
        '''
        if angle_diff > 0:
            # Turn right
            self.left_thrust_pub.publish(Float64(data=100.0))
            self.right_thrust_pub.publish(Float64(data=0.0))
        else:
            # Turn left
            self.left_thrust_pub.publish(Float64(data=0.0))
            self.right_thrust_pub.publish(Float64(data=100.0))

    def move_forward(self):
        '''
        This function is designed to move the robot forward.
        '''
        self.left_thrust_pub.publish(Float64(data=VELOCITY)) 
        self.right_thrust_pub.publish(Float64(data=VELOCITY))

    def calculate_angular_difference(self, cur_theta, desired_angle):
        '''
        Given the current yaw orientation and the desired one, calculates the difference between them.
        '''
        # Normalize angles in the range [0, 2pi].
        cur_theta_normalized = (cur_theta + 2 * math.pi) % (2 * math.pi)
        desired_angle_normalized = (desired_angle + 2 * math.pi) % (2 * math.pi)
        
        # Calculate the angular difference
        diff = (desired_angle_normalized - cur_theta_normalized + math.pi) % (2 * math.pi) - math.pi
        
        return diff

    def read_waypoints_from_file(self, file_path):
        '''
        This function is called when the node is initialized.
        It opens the desired text file and reads the waypoints defined in it.
        Each row defines a waypoint as follows: “x y”
        An example would be:
        -540 233
        -550 230
        Where we have 2 waypoints.
        '''
        try:
            with open(file_path, "r") as file:
                for line in file:
                    if not line.strip():
                        # The line is empty
                        continue
                    
                    #  Convert the line in a list of values
                    data = line.split()
                    
                    if len(data) == 3:
                        pos_x = float(data[0])  # X coordinate
                        pos_y = float(data[1])  # Y coordinate
                        
                        # Adds the coordinates to the node parameters
                        self.wps_pos_x.append(pos_x)
                        self.wps_pos_y.append(pos_y)
                        self.wp_count += 1

        except FileNotFoundError:
            self.get_logger().error(f"File {file_path} cannot be found.")
        except Exception as e:
            self.get_logger().error(f"Error when trying to read the file: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
