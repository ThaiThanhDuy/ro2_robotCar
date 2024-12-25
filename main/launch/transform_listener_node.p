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
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Adjust as necessary
        self.command_in_progress = False

        # Timer to periodically get the transformation
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 second interval

        # Goal parameters (x, y, yaw)
        self.goal = (0.0, 0.0, 0.0)  # Initialize goal (x, y, yaw)
        self.state = 'ALIGN_YAW'  # Initial state

        # Define the sequence of goals
        self.goals = [
            (0.0, 0.0, 0.0),  # Point A
            (1.0, 1.0, 0.0),  # Point B
            (2.0, 0.0, 0.0)   # Point C
        ]
        self.current_goal_index = 0  # Start with the first goal

        # Subscribe to the LaserScan data
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
        }

    def scan_callback(self, msg):
        # Get the number of ranges
        num_ranges = len(msg.ranges)

        # Define the angles for each direction
        directions = {
            'front': num_ranges // 2,                # 0 degrees
            'back': 0,   # 180 degrees
            'left': (3 * num_ranges) // 4,   # 90 degrees
            'right': num_ranges // 4,  # 270 degrees
        }

        # Calculate distances for each direction
        for direction, index in directions.items():
            distance = msg.ranges[index]
            if distance < 1.0:  # Check if the distance is less than 1 meter
                # Here, we would need additional logic to classify the obstacle
                # For demonstration, we will assume all detected obstacles are static
                self.distances[direction]['static'] = min(self.distances[direction]['static'], distance)
            else:
                # Reset the distance if no obstacle is detected
                self.distances[direction]['static'] = float('inf')

    def timer_callback(self):
        try:
            # Get the transformation from 'odom' to 'base_link'
            trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            self.process_transform(trans)
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

    def process_transform(self, trans: TransformStamped):
        # Extract translation
        x = -trans.transform.translation.x
        y = -trans.transform.translation.y
        
        # Convert quaternion to yaw
        yaw = -self.quaternion_to_yaw(trans.transform.rotation)

        # Log the current position
        self.get_logger().info(f'Current Position: x={x}, y={y}, Yaw={yaw}')

        # Control the robot based on the goal
        self.control_robot(x, y, yaw)

    def control_robot(self, current_x, current_y, current_yaw):
        goal_x, goal_y, goal_yaw = self.goal

        # Define thresholds
        threshold = 0.15  # Threshold for x and y
        yaw_threshold = 0.2  # Threshold for yaw
        obstacle_threshold = 0.5  # Threshold for obstacle detection

        if self.state == 'ALIGN_YAW':
            # Calculate the desired yaw angle based on the goal yaw
            desired_yaw = goal_yaw

            # Normalize the yaw difference to be within -π to π
            yaw_difference = desired_yaw - current_yaw
            if yaw_difference > math.pi:
                yaw_difference -= 2 * math.pi
            elif yaw_difference < -math.pi:
                yaw_difference += 2 * math.pi

            # Control logic for yaw adjustment
            if abs(yaw_difference) > yaw_threshold:  # Threshold to avoid oscillation
                if yaw_difference > 0:
                    angular_z = 0.2  # Left spin speed
                else:
                    angular_z = -0.2  # Right spin speed
                linear_x = 0.0
                linear_y = 0.0
            else:
                # Stop spinning if within threshold
                angular_z = 0.0
                self.state = 'MOVE_X'  # Transition to moving in x direction

        elif self.state == 'MOVE_X':
            time.sleep(1.5)
            # Check for obstacles in front
            if self.distances['front']['static'] < obstacle_threshold:
                self.get_logger().info("Obstacle detected in front! Stopping the robot.")
                linear_x = 0.0
                linear_y = 0.0
                angular_z = 0.0
                self.send_command(linear_x, linear_y, angular_z)  # Send stop command
                return  # Exit the function to prevent further movement

            # Calculate the distance to the goal in x
            distance_to_goal_x = goal_x - current_x

            # Check if within threshold for x
            if abs(distance_to_goal_x) < threshold:
                # Stop if within threshold for x
                linear_x = 0.0
                self.state = 'MOVE_Y'  # Transition to moving in y direction
            else:
                # Move towards the goal in x direction
                if distance_to_goal_x > 0:
                    linear_x = 0.06  # Move forward
                else:
                    linear_x = -0.06  # Move backward
                linear_y = 0.0
                angular_z = 0.0

        elif self.state == 'MOVE_Y':
            time.sleep(1)
            # Check for obstacles based on the direction of movement
            if goal_y > current_y:  # Moving in positive y direction
                if self.distances['left']['static'] < obstacle_threshold:
                    self.get_logger().info("Obstacle detected on the left! Stopping the robot.")
                    linear_y = 0.0
                    linear_x = 0.0
                    angular_z = 0.0
                    self.send_command(linear_x, linear_y, angular_z)  # Send stop command
                    return  # Exit the function to prevent further movement
            else:  # Moving in negative y direction
                if self.distances['right']['static'] < obstacle_threshold:
                    self.get_logger().info("Obstacle detected on the right! Stopping the robot.")
                    linear_y = 0.0
                    linear_x = 0.0
                    angular_z = 0.0
                    self.send_command(linear_x, linear_y, angular_z)  # Send stop command
                    return  # Exit the function to prevent further movement

            # Calculate the distance to the goal in y
            distance_to_goal_y = goal_y - current_y

            # Check if within threshold for y
            if abs(distance_to_goal_y) < threshold:
                # Stop if within threshold for y
                linear_y = 0.0
                self.send_command(0.0, 0.0, 0.0)  # Stop the robot
                
                self.get_logger().info("Reached goal. Checking final position and yaw...")
                
                # Check if the robot is at the goal and aligned
                if abs(current_x - goal_x) < threshold and abs(current_y - goal_y) < threshold and abs(current_yaw - goal_yaw) < yaw_threshold:
                    self.get_logger().info(f"Robot has reached goal at Point {'A' if self.current_goal_index == 0 else 'B' if self.current_goal_index == 1 else 'C'}.")
                    if self.current_goal_index == 0:  # If at Point A again
                        self.send_command(0.0, 0.0, 0.0)  # Send stop command
                    self.current_goal_index += 1  # Move to the next goal
                    if self.current_goal_index < len(self.goals):
                        next_goal = self.goals[self.current_goal_index]
                        self.set_goal(*next_goal)  # Set the next goal
                        if self.current_goal_index == 1:  # Point B
                            time.sleep(5)  # Wait for 5 seconds at Point B
                        if self.current_goal_index == 2:  # Point C
                            time.sleep(3)  # Wait for 3 seconds at Point C
                    else:
                        self.get_logger().info("All goals reached. Returning to Point A.")
                        self.set_goal(0.0, 0.0, 0.0)  # Return to Point A
                else:
                    self.get_logger().info("Robot is not aligned. Re-evaluating position...")
                    self.state = 'ALIGN_YAW'  # Re-evaluate yaw alignment
                return
            else:
                # Move towards the goal in y direction
                if distance_to_goal_y > 0:
                    linear_y = 0.07  # Move forward in y
                else:
                    linear_y = -0.07  # Move backward in y
                linear_x = 0.0
                angular_z = 0.0

        # Send command to the robot
        self.send_command(linear_x, linear_y, angular_z)

    def send_command(self, linear_x, linear_y, angular_z):
        # Construct the command string and send it to the robot
        command = f"{linear_x},{linear_y},{angular_z}\n"
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Command sent: {command.strip()}")

    def quaternion_to_yaw(self, quaternion):
        # Convert quaternion to yaw angle
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
