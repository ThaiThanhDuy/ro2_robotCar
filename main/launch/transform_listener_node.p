import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import serial  # Ensure you have the pyserial package installed
import time
import tkinter as tk  # Import tkinter for GUI
from threading import Thread  # Import Thread for running the robot control in a separate thread

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # Create a buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize serial ports
        self.command_serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # For movement commands
        # self.character_serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)  # For sending characters
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
        goal_x, goal_y, goal_yaw = self.goal

        # Define thresholds
        threshold = 0.15  # Threshold for x and y
        yaw_threshold = 0.2  # Threshold for yaw

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
                angular_z = 0.2 if yaw_difference > 0 else -0.2  # Left or right spin speed
                linear_x = 0.0
                linear_y = 0.0
            else:
                # Stop spinning if within threshold
                angular_z = 0.0
                self.state = 'MOVE_X'  # Transition to moving in x direction
		
        elif self.state == 'MOVE_X':
            time.sleep(1.5)
            # Calculate the distance to the goal in x
            distance_to_goal_x = goal_x - current_x

            # Check if within threshold for x
            if abs(distance_to_goal_x) < threshold:
                # Stop if within threshold for x
                linear_x = 0.0
                self.state = 'MOVE_Y'  # Transition to moving in y direction else:
                # Move towards the goal in x direction
                linear_x = 0.06 if distance_to_goal_x >= 0.1 else 0.05 if distance_to_goal_x > 0 else -0.06 if distance_to_goal_x <= -0.1 else -0.05
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
                if abs(current_x - goal_x) < threshold and abs(current_y - goal_y) < threshold and abs(current_yaw - goal_yaw) < yaw_threshold:
                    self.get_logger().info(f"Robot has reached goal at Point {'A' if self.current_goal_index == 0 else 'B' if self.current_goal_index == 1 else 'C'}.")
                    self.current_goal_index += 1  # Move to the next goal
                    if self.current_goal_index < len(self.goals):
                        next_goal = self.goals[self.current_goal_index]
                        self.set_goal(*next_goal)  # Set the next goal
                    else:
                        self.get_logger().info("All goals reached. Returning to Point A.")
                        self.set_goal(0.0, 0.0, 0.0)  # Return to Point A
                else:
                    self.get_logger().info("Robot is not aligned. Re-evaluating position...")
                    self.state = 'ALIGN_YAW'  # Re-evaluate yaw alignment
                return
            else:
                # Move towards the goal in y direction
                linear_y = 0.07 if distance_to_goal_y > 0.1 else 0.04 if distance_to_goal_y > 0 else -0.07 if distance_to_goal_y < -0.1 else -0.04
                linear_x = 0.0
                angular_z = 0.0

        # Send command to the robot
        self.send_command(linear_x, linear_y, angular_z)

    def send_command(self, linear_x, linear_y, angular_z):
        if self.command_in_progress:
            self.get_logger().warn("Previous command still in progress. Please wait.")
            return

        # Check if the serial port is open
        if self.command_serial_port is None or not self.command_serial_port.is_open:
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
            self.command_serial_port.write(command.encode())
          
            self.get_logger().info(f"Sending command: {command.strip()}")

        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")
        finally:
            self.command_in_progress = False  # Reset the state variable

    # def send_character(self, char):
    #     """Send a character to the character serial port."""
    #     if self.character_serial_port is None or not self.character_serial_port.is_open:
    #         self.get_logger().warn("Cannot send character: Character port is not open.")
    #         return
    #     self.character_serial_port.write(char.encode())
    #     self.get_logger().info(f"Sent character: {char}")

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

    def run_sequence(self):
        """Run the sequence from A to B to C and back to A."""
        for goal in self.goals:
            self.set_goal(*goal)
            while self.state != 'ALIGN_YAW':
                time.sleep(0.1)  # Wait until the robot reaches the goal

class GUI:
    def __init__(self, node):
        self.node = node
        self.window = tk.Tk()
        self.window.title("Robot Control")

        # Input fields for points B and C
        tk.Label(self.window, text="Point B (x, y):").grid(row=0, column=0)
        self.point_b_x = tk.Entry(self.window)
        self.point_b_y = tk.Entry(self.window)
        self.point_b_x.grid(row=0, column=1)
        self.point_b_y.grid(row=0, column=2)

        tk.Label(self.window, text="Point C (x, y):").grid(row=1, column=0)
        self.point_c_x = tk.Entry(self.window)
        self.point_c_y = tk.Entry(self.window)
        self.point_c_x.grid(row=1, column=1)
        self.point_c_y.grid(row=1, column=2)

        # Button to start the sequence
        self.start_button = tk.Button(self.window, text="Start Sequence", command=self.start_sequence)
        self.start_button.grid(row=2, column=0, columnspan=3)

    def start_sequence(self):
        # Get points B and C from input fields
        try:
            b_x = float(self.point_b_x.get())
            b_y = float(self.point_b_y.get())
            c_x = float(self.point_c_x.get())
            c_y = float(self.point_c_y.get())
            self.node.goals[1] = (b_x, b_y, 0.0)  # Update Point B
            self.node.goals[2] = (c_x, c_y, 0.0)  # Update Point C
            self.node.current_goal_index = 0  # Reset to start from Point A
            Thread(target=self.node.run_sequence).start()  # Run the sequence in a separate thread
        except ValueError:
            print("Please enter valid numeric values for points.")

    def run(self):
        self.window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    gui = GUI(node)
    gui.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
