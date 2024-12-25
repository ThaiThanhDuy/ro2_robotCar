import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import serial
import time
import tkinter as tk
from tkinter import messagebox, ttk
import threading

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # Create a buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize serial port (replace with your actual port)
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.command_in_progress = False

        # Timer to periodically get the transformation
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Goal parameters (x, y, yaw)
        self.goal = (0.0, 0.0, 0.0)
        self.state = 'ALIGN_YAW'
        self.is_running = False  # Flag to control robot movement

        # Define the sequence of goals
        self.goals = {
            "Point A": (0.0, 0.0, 0.0),
            "Point B": (1.0, 1.0, 0.0),
            "Point C": (2.0, 0.0, 0.0)
        }
        self.current_goal_index = 0

        # Initialize GUI in a separate thread
        self.gui_thread = threading.Thread(target=self.init_gui)
        self.gui_thread.start()

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("Robot Goal Setter")

        # Create a dropdown menu for selecting goals
        tk.Label(self.root, text="Select Goal:").pack()
        self.goal_selection = ttk.Combobox(self.root, values=list(self.goals.keys()))
        self.goal_selection.pack()
        self.goal_selection.current(0)  # Set default selection to Point A

        # Create input fields for goals
        tk.Label(self.root, text="Goal X:").pack()
        self.goal_x_entry = tk.Entry(self.root)
        self.goal_x_entry.pack()

        tk.Label(self.root, text="Goal Y:").pack()
        self.goal_y_entry = tk.Entry(self.root)
        self.goal_y_entry.pack()

        tk.Label(self.root, text="Goal Yaw:").pack()
        self.goal_yaw_entry = tk.Entry(self.root)
        self.goal_yaw_entry.pack()

        # Create a button to set the goal
        self.set_goal_button = tk.Button(self.root, text="Set Goal", command=self.set_goal_from_input)
        self.set_goal_button.pack()

        # Create a button to run the robot
        self.run_button = tk.Button(self.root, text="Run", command=self.run_robot)
        self.run_button.pack()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Start the Tkinter main loop
        self.root.mainloop()

    def set_goal_from_input(self):
        selected_goal = self.goal_selection.get()
        if selected_goal in self.goals:
            x, y, yaw = self.goals[selected_goal]
            self.set_goal(x, y, yaw)
        else:
            messagebox.showerror("Input Error", "Please select a valid goal.")

    def run_robot(self):
        self.is_running = True  # Set the flag to indicate the robot should run
        if self.current_goal_index < len(self.goals):
            next_goal = list(self.goals.values())[self.current_goal_index]
            self.set_goal(*next_goal)
            self.get_logger().info(f"Running to goal: {next_goal}")
        else:
            messagebox.showinfo("Info", "All goals have been reached.")

    def set_goal(self, x, y, yaw):
        self.goal = (x, y, yaw)
        self.get_logger().info(f"Goal set to: x={x}, y={y}, yaw={yaw}")

    def timer_callback(self):
        if self.is_running:  # Only process transforms if the robot is running
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
                self.process_transform(trans)
            except Exception as e:
                self.get_logger().info(f'Could not get transform: {e}')

    def process_transform(self, trans: TransformStamped):
        x = -trans.transform.translation.x
        y = -trans.transform.translation.y
        yaw = -self.quaternion_to_yaw(trans.transform.rotation)

        self.get_logger().info(f'Current Position: x={x}, y={y}, Yaw={yaw}')
        self.control_robot(x, y, yaw)

    def control_robot(self, current_x, current_y, current_yaw):
        if not self.is_running:  # Do not control the robot if not running
            return

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
                if abs(current_x - goal_x) < threshold and abs(current_y - goal_y) < threshold and abs(current_yaw - goal_yaw) < yaw_threshold:
                    self.get_logger().info(f"Robot has reached goal at Point {'A' if self.current_goal_index == 0 else 'B' if self.current_goal_index == 1 else 'C'}.")
                    if self.current_goal_index == 0:  # If at Point A again
                        self.send_command(0.0, 0.0, 0.0)  # Send stop command
                    self.current_goal_index += 1  # Move to the next goal
                    if self.current_goal_index < len(self.goals):
                        next_goal = list(self.goals.values())[self.current_goal_index]
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
                    if distance_to_goal_y < 0.1:
                        linear_y = 0.04
                    else:
                        linear_y = 0.07 
                else:
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

    def send_character(self, char):
        """Send a character to the serial port."""
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn("Cannot send character: Port is not open.")
            return
        self.serial_port.write(char.encode())
        self.get_logger().info(f"Sent character: {char}")

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

    def on_closing(self):
        self.is_running = False  # Stop the robot when closing the GUI
        self.root.destroy()
        self.get_logger().info("GUI closed. Shutting down node.")
        rclpy.shutdown()

def main(args=None):

    rclpy.init(args=args)
    node = TransformListenerNode()
    node.set_goal(0.0, 0.0, 0.0)  # Start at Point A
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
