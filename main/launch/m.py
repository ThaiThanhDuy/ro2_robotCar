import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import serial  # Ensure you have the pyserial package installed
import time
from sensor_msgs.msg import LaserScan
import numpy as np

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
        self.state = 'ALIGN_YAW'  # Initial state

        # Define the sequence of goals
        self.goals = [
            (1.0, 0.0, 0.0),  # Point A
            (2.0, 0.0, 0.0),  # Point B
            (0.0, 0.0, 0.0),   # Point C
          
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
	self.goal_x=0.0
        # Flag to indicate if obstacles were detected
        self.obstacles_detected = False
        self.obstacle_threshold = 0.5
        self.auto = 0
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
                # Here, we would need additional logic to classify the obstacle
                # For demonstration, we will assume all detected obstacles are static
                self.distances[direction]['static'] = min(self.distances[direction]['static'], distance)
                self.obstacles_detected = True  # Set flag to indicate obstacles are detected
            else:
                # Reset the distance if no obstacle is detected
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
        x = -trans.transform.translation.x
        y = -trans.transform.translation.y
        
        # Convert quaternion to yaw
        yaw = -self.quaternion_to_yaw(trans.transform.rotation)

        # Log the current position
        self.get_logger().info(f'Current Position: x={x}, y={y}, Yaw={yaw}')

        # Control the robot based on the goal
        self.control_robot(x, y, yaw)

    def control_robot(self, current_x, current_y, current_yaw):
        self.get_logger().info(f'Could not get transform: {self.goal}')
        self.goal_x  = self.goal[0]
        goal_y = self.goal[1]
        goal_yaw = self.goal[2]
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        self.get_logger().info(f'Could not get transform: {self.goal}')
        # Define thresholds
        threshold = 0.15  # Threshold for x and y
        yaw_threshold = 0.1  # Threshold for yaw radian

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
                    angular_z = 0.5  # Left spin speed
                else:
                    angular_z = -0.5  # Right spin speed
                linear_x = 0.0
                linear_y = 0.0
            else:
                # Stop spinning if within threshold
                angular_z = 0.0
                self.state = 'MOVE_X'  # Transition to moving in x direction
		
        elif self.state == 'MOVE_X':
            time.sleep(1.5)
             # Check for obstacles in front
            if self.distances['front']['static'] < 0.5:
                self.get_logger().info("Obstacle detected in front! Stopping the robot.")
                linear_x = 0.0
                linear_y = 0.0
                angular_z = 0.0
                self.send_command(linear_x, linear_y, angular_z)  # Send stop command
                return  # Exit the function to prevent further movement
            else:
                distance_to_goal_x = self.goal_x - current_x
                if abs(distance_to_goal_x) < threshold:
        # Stop if within threshold for x
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
            time.sleep(1)
            # Calculate the distance to the goal in y


            distance_to_goal_y = goal_y - current_y

            # Check if within threshold for y
            if abs(distance_to_goal_y) < threshold:
                # Stop if within threshold for y
                linear_y = 0.0
                self.send_command(0.0, 0.0, 0.0)  # Stop the robot
                
                self.get_logger().info("Reached goal. Checking final position and yaw...")
                
                # Check if the robot is at the goal and aligned
                if abs(current_x - self.goal_x) < threshold and abs(current_y - goal_y) < threshold and abs(current_yaw - goal_yaw) < yaw_threshold:
                    self.get_logger().info(f"Robot has reached goal at Point {'A' if self.current_goal_index == 0 else 'B' if self.current_goal_index == 1 else 'C'}.")
     
                    
                    if self.current_goal_index == 0:  # If at Point A again
                        self.send_command(0.0, 0.0, 0.0)
                        self.get_logger().info("Waiting 'P'")
                        self.wait_for_response('P')  # Wait for 'N'
                    self.current_goal_index += 1  # Move to the next goal
                        
                    if self.current_goal_index < len(self.goals):
                        next_goal = self.goals[self.current_goal_index]
                        self.set_goal(*next_goal)
                        d =abs(current_x - self.goal_x) 
                        time.sleep(4)
                        self.get_logger().info(f'ZZ:{d}')
                        if self.current_goal_index == 1:  # Point B
                         
                            time.sleep(1)  # Wait for 5 seconds at Point B
                            self.current_goal_index += 1  # Move to the next goal
                            

                        if self.current_goal_index == 2:  # Point C
                            time.sleep(1)  # Wait for 3 seconds at Point C
                            self.get_logger().info("Reached goal B. Sending 'A'...")
                            self.send_character('A')  # Send 'B' to the character port
                            
                            self.get_logger().info("Waiting 'N'")
                            self.wait_for_response('N')  # Wait for 'N'
                            self.get_logger().info("Received 'N', proceeding to Point C.")
                         
                            self.current_goal_index += 1  # Move to the next goal
                        if self.current_goal_index == 3:  # Point C
                            time.sleep(1)  # Wait for 3 seconds at Point C
                            self.get_logger().info("Reached goal C. Sending 'C'...")
                            self.send_character('I')
                            self.get_logger().info("Waiting 'M'")
                            self.wait_for_response('M')
                            self.get_logger().info("Received 'M', proceeding to Point A.")
                           
                            self.current_goal_index += 1
                       
                            
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

        # Send command to the robot
        self.send_command(linear_x, linear_y, angular_z)

    def send_command(self, linear_x, linear_y, angular_z):
        if self.command_in_progress:
            self.get_logger().warn("Previous command still in progress. Please wait.")
            return

        # Check if the serial port is open
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn("Cannot send command: Port is not open.")
            return

        self.command_in_progress = True
        try:
            wheel_radius = 0.0485  # Adjust to your robot's wheel radius
            wheel_base = 0.38     # Distance between front and rear wheels

            # Calculate individual wheel velocities
            front_left_velocity = (linear_x - linear_y - (wheel_base * angular_z / 2))
            front_right_velocity = (linear_x + linear_y + (wheel_base * angular_z / 2))
            rear_left_velocity = (linear_x + linear_y - (wheel_base * angular_z / 2))
            rear_right_velocity = (linear_x - linear_y + (wheel_base * angular_z / 2))

           # Check if velocities are within the range -0.27 to 0.27
            if (abs(front_left_velocity) > 0.27 or
                abs(front_right_velocity) > 0.27 or
                abs(rear_left_velocity) > 0.27 or
                abs(rear_right_velocity) > 0.27):
                self.get_logger().warn("One or more velocities are out of range. Command not sent.")
                return  # Skip sending the command if any velocity is out of range

            # Scale velocities if necessary
            front_left_velocity *= 2.0
            front_right_velocity *= 2.0
            rear_left_velocity *= 2.0
            rear_right_velocity *= 2.0

            # Round velocities to two decimal places
            front_left_velocity = round(front_left_velocity, 2)
            front_right_velocity = round(front_right_velocity, 2)
            rear_left_velocity = round(rear_left_velocity, 2)
            rear_right_velocity = round(rear_right_velocity, 2)

            # Send the new command to the STM32
            command = f"c:{front_left_velocity},{front_right_velocity},{rear_left_velocity},{rear_right_velocity}\n"
            self.serial_port.write(command.encode())
          
            self.get_logger().info(f"Sending command: {command.strip()}")

        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")
        finally:
            self.command_in_progress = False  # Reset the state variable
    def send_character(self, character):
        """Send a character to the character serial port."""
        self.character_port.write(character.encode('utf-8'))
        self.get_logger().info(f"Character sent: {character.strip()}")

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
       

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle."""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        # Calculate yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
