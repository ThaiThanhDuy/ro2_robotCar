import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import serial  # Ensure you have the pyserial package installed
import time
from sensor_msgs.msg import LaserScan

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # Create a buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize serial port (replace with your actual port)
        self.character_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)  # For sending characters
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Adjust as necessary
        self.command_in_progress = False

        # Timer to periodically get the transformation
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 second interval

        # Goal parameters (x, y, yaw)
        self.goal = (0.0, 0.0, 0.0)  # Initialize goal (x, y, yaw)
        self.state = 'WAIT_FOR_P'  # Initial state

        # Define the sequence of goals
        self.goals = [
            (1.0, 0.0, 0.0),  # Point A
            (2.0, 0.0, 0.0),  # Point B
            (0.0, 0.0, 0.0),  # Point C
        ]
        self.current_goal_index = 0  # Start with the first goal
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Initialize distances dictionary
        self.distances = {
            'front': {'static': float('inf')},
            'back': {'static': float('inf')},
            'left': {'static': float('inf')},
            'right': {'static': float('inf')},
            'Northeast': {'static': float('inf')},
            'Southwest': {'static': float('inf')},
            'Northwest': {'static': float('inf')},
            'Southeast': {'static': float('inf')},
        }
	
        # Flag to indicate if obstacles were detected
        self.obstacles_detected = False
        self.obstacle_threshold = 0.5

    def scan_callback(self, msg):
        # Get the number of ranges
        num_ranges = len(msg.ranges)

        # Define the angles for each direction
        directions = {
            'front': num_ranges // 2,                # 0 degrees
            'back': 0,   # 180 degrees
            'left': (3 * num_ranges) // 4,   # 90 degrees
            'right': num_ranges // 4 , # 270 degrees
            'Northeast': num_ranges // 8,                # 45 degrees
            'Southwest': 5 * num_ranges // 8,             # 225 degrees
            'Northwest': 3 * num_ranges // 8,             # 135 degrees
            'Southeast': 7 * num_ranges // 8             # 315 degrees
        }

        # Calculate distances for each direction
        for direction, index in directions.items():
            distance = msg.ranges[index]
            if distance < 1.0:  # Check if the distance is less than 1 meter
                self.distances[direction]['static'] = min(self.distances[direction]['static'], distance)
                self.obstacles_detected = True  # Set flag to indicate obstacles are detected
            else:
                self.distances[direction]['static'] = float('inf')

    def set_goal(self, x, y, yaw):
        """Set the goal position and orientation."""
        self.goal = (x, y, yaw)
        self.get_logger().info(f"Goal set to: x={x}, y={y}, yaw={yaw}")

    def timer_callback(self):
        try:
            # Get the transformation from 'odom' to 'base_link'
            trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            self.process_transform(trans)
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

    def process_transform(self, trans: TransformStamped):
        # Extract translation
        current_x = trans.transform.translation.x
        current_y = trans.transform.translation.y
        current_yaw = self.get_yaw_from_quaternion(trans.transform.rotation)

        # Control the robot based on the current state
        self.control_robot(current_x, current_y, current_yaw)

    def get_yaw_from_quaternion(self, quaternion):
        """Convert quaternion to yaw angle."""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        # Calculate yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_robot(self, current_x, current_y, current_yaw):
        goal_x, goal_y, goal_yaw = self.goal
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        threshold = 0.15  # Threshold for x and y
        yaw_threshold = 0.1  # Threshold for yaw radian

        if self.state == 'WAIT_FOR_P':
            self.get_logger().info("Waiting for 'P' to start moving to Point A...")
            self.wait_for_response('P')  # Wait for 'P'
            self.state = 'ALIGN_YAW'  # Transition to aligning yaw after receiving 'P'
            return  # Exit to prevent further processing

        elif self.state == 'ALIGN_YAW':
            desired_yaw = goal_yaw
            yaw_difference = desired_yaw - current_yaw
            if yaw_difference > math.pi:
                yaw_difference -= 2 * math.pi
            elif yaw_difference < -math.pi:
                yaw_difference += 2 * math.pi

            if abs(yaw_difference) > yaw_threshold:
                angular_z = 0.5 if yaw_difference > 0 else -0.5
            else:
                angular_z = 0.0
                self.state = 'MOVE_X'  # Transition to moving in x direction

        elif self.state == 'MOVE_X':
            if self.distances['front']['static'] < 0.5:
                self.get_logger().info("Obstacle detected in front! Stopping the robot.")
                linear_x = 0.0
                self.send_command(linear_x, linear_y, angular_z)
                return

            distance_to_goal_x = goal_x - current_x
            if abs(distance_to_goal_x) < threshold:
                linear_x = 0.0
                self.state = 'MOVE_Y'  # Transition to moving in y direction
            else:
        # Move towards the goal in x direction
                if distance_to_goal_x > 0:
                    if distance_to_goal_x < 0.1:
                        linear_x = 0.05
                    else:
                        linear_x = 0.06
                else:
                    if distance_to_goal_x > -0.1:
                        linear_x = -0.05
                    else:
                        linear_x = -0.06

            linear_y = 0.0
            angular_z = 0.0

        elif self.state == 'MOVE_Y':
            distance_to_goal_y = goal_y - current_y
            if abs(distance_to_goal_y) < threshold:
                linear_y = 0.0
                self.send_command(0.0, 0.0, 0.0)  # Stop the robot
                self.get_logger().info("Reached goal. Checking final position and yaw...")

                if (abs(current_x - goal_x) < threshold and 
                    abs(current_y - goal_y) < threshold and 
                    abs(current_yaw - goal_yaw) < yaw_threshold):
                    self.get_logger().info(f"Robot has reached goal at Point {'A' if self.current_goal_index == 0 else 'B' if self.current_goal_index == 1 else 'C'}.")
                    self.send_character('A' if self.current_goal_index == 0 else 'B' if self.current_goal_index == 1 else 'C')
                    self.wait_for_response('N' if self.current_goal_index < 2 else 'M')
                    self.current_goal_index += 1
                    if self.current_goal_index < len(self.goals):
                        next_goal = self.goals[self.current_goal_index]
                        self.set_goal(*next_goal)
                        self.state = 'ALIGN_YAW'  # Re-align yaw for the next goal
                else:
                    self.get_logger().info("Robot is not aligned. Re-evaluating position...")
                    self.state = 'ALIGN_YAW'  # Re-evaluate yaw alignment
                return
            else:
                # Move towards the goal in y direction
                if distance_to_goal_y > 0:
                    if self.distances['left']['static'] < 0.15:
                        self.get_logger().info("Obstacle detected on the left! Stopping the robot.")
                        linear_y = 0.0
                        linear_x = 0.0
                        angular_z = 0.0
                        self.send_command(linear_x, linear_y, angular_z)  # Send stop command
                        return  # Exit the function to prevent further movement
                    else:
                        if distance_to_goal_y < 0.1:
                            linear_y = 0.04
                        else:
                            linear_y = 0.07 
                else:
                    if self.distances['right']['static'] < 0.15:
                        self.get_logger().info("Obstacle detected on the right! Stopping the robot.")
                        linear_y = 0.0
                        linear_x = 0.0
                        angular_z = 0.0
                        self.send_command(linear_x, linear_y, angular_z)  # Send stop command
                        return  # Exit the function to prevent further movement
                    else :
                        if distance_to_goal_y > -0.1:
                            linear_y = -0.04
                        else:
                            linear_y = -0.07
                linear_x = 0.0
                angular_z = 0.0
                 

        self.send_command(linear_x, linear_y, angular_z)

    def send_command(self, linear_x, linear_y, angular_z):
        """Send movement commands to the robot."""
        command = f"{linear_x},{linear_y},{angular_z}\n"
        self.serial_port.write (command.encode('utf-8'))
        self.get_logger().info(f"Sent command: {command.strip()}")

    def send_character(self, character):
        """Send a character to the robot via serial."""
        self.character_port.write(character.encode('utf-8'))
        self.get_logger().info(f"Sent character: {character}")

    def wait_for_response(self, expected_response):
         """Wait for a specific response from the character serial port."""
        self.get_logger().info(f"Waiting for response '{expected_response}'...")
        while True:
            try:
                with open("/home/pi4/code/uart_data.txt", "r") as file:
                    response = file.read()
                    if response == expected_response:
                        self.get_logger().info(f"Received '{expected_response}', proceeding with the next steps.")
                        break
                    else:
                        self.get_logger().warn(f"Unexpected response: {response}. Waiting for '{expected_response}'...")
            except FileNotFoundError:
                pass
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
