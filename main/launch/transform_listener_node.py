import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import serial
import time
from sensor_msgs.msg import LaserScan
import numpy as np
import tkinter as tk
from tkinter import messagebox

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # Create a buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize serial port (replace with your actual port)
        self.character_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.serial_port = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
        self.command_in_progress = False

        # Timer to periodically get the transformation
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Goal parameters (x, y, yaw)
        self.goals = [
            (0.0, 0.0, 0.0),  # Point A
            (1.0, 0.0, 0.0),  # Point B
            (2.0, 0.0, 0.0)   # Point C
        ]
        self.current_goal_index = 0
        
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

        self.obstacles_detected = False
        self.obstacle_threshold = 0.5

        # Initialize GUI
        self.init_gui()

    def init_gui(self):
        """Initialize the GUI for setting goals."""
        self.root = tk.Tk()
        self.root.title("Set Goals")

        # Create labels and entry boxes for three goals
        for i in range(3):
            tk.Label(self.root, text=f"Goal {i + 1} - X:").grid(row=i, column=0)
            setattr(self, f'x_entry_{i}', tk.Entry(self.root))
            getattr(self, f'x_entry_{i}').grid(row=i, column=1)

            tk.Label(self.root, text=f"Goal {i + 1} - Y:").grid(row=i, column=2)
            setattr(self, f'y_entry_{i}', tk.Entry(self.root))
            getattr(self, f'y_entry_{i}').grid(row=i, column=3)

            tk.Label(self.root, text=f"Goal {i + 1} - Yaw:").grid(row=i, column=4)
            setattr(self, f'yaw_entry_{i}', tk.Entry(self.root))
            getattr(self, f'yaw_entry_{i}').grid(row=i, column=5)

        set_goal_button = tk.Button(self.root, text="Set New Goals", command=self.set_new_goals)
        set_goal_button.grid(row=3, columnspan=6)

        run_button = tk.Button(self.root, text="Run", command=self.run)
        run_button.grid(row=4, columnspan=6)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def set_new_goals(self):
        """Set new goals based on user input."""
        new_goals = []
        for i in range(3):
            try:
                x = float(getattr(self, f'x_entry_{i}').get())
                y = float(getattr(self, f'y_entry_{i}').get())
                yaw = float(getattr(self, f'yaw_entry_{i}').get())
                new_goals.append((x, y, yaw))
            except ValueError:
                messagebox.showerror("Invalid Input", f"Please enter valid numeric values for Goal {i + 1}.")
                return

        self.goals = new_goals  # Set new goals
        self.current_goal_index = 0  # Reset to the first goal
        messagebox.showinfo("Success", f"New goals set: {self.goals}")

    def run(self):
        """Run the node with the current goals."""
        self.get_logger().info("Running with current goals...")
        self.set_goal(*self.goals[self.current_goal_index])  # Set the first goal
        rclpy.spin(self)  # Start the ROS 2 event loop

    def on_closing(self):
        """Handle the closing of the GUI."""
        self.root.destroy()
        self.get_logger().info("GUI closed. Shutting down node.")
        rclpy.shutdown()

    def scan_callback(self, msg):
        # Get the number of ranges
        num_ranges = len(msg.ranges)

        # Define the angles for each direction
        directions = {
            'front': num_ranges // 2,
            'back': 0,
            'left': (3 * num_ranges) // 4,
            'right': num_ranges // 4,
            'Northeast': num_ranges // 8,
            'Southwest': 5 * num_ranges // 8,
            'Northwest': 3 * num_ranges // 8,
            'Southeast': 7 * num_ranges // 8
        }

        # Calculate distances for each direction
        for direction, index in directions.items():
            distance = msg.ranges[index]
            if distance < 1.0:
                self.distances[direction]['static'] = min(self.distances[direction]['static'], distance)
                self.obstacles_detected = True
            else:
                self.distances[direction]['static'] = float('inf')

    def set_goal(self, x, y, yaw):
        """Set the goal position and orientation."""
        self.goal = (x, y, yaw)
        self.get_logger().info(f"Goal set to: x={x}, y={y}, yaw={yaw}")

    def timer_callback(self):
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
        goal_x, goal_y, goal_yaw = self.goal
        threshold = 0.15
        yaw_threshold = 0.2

        if self.state == 'ALIGN_YAW':
            desired_yaw = goal_yaw
            yaw_difference = desired_yaw - current_yaw
            if yaw_difference > math.pi:
                yaw_difference -= 2 * math.pi
            elif yaw_difference < -math.pi:
                yaw_difference += 2 * math.pi

            if abs(yaw_difference) > yaw_threshold:
                angular_z = 0.2 if yaw_difference > 0 else -0.2
                linear_x = 0.0
                linear_y = 0.0
            else:
                angular_z = 0.0
                self.state = 'MOVE_X'

        elif self.state == 'MOVE_X':
            time.sleep(1.5)
            if self.distances['front']['static'] < 0.5:
                self.get_logger().info("Obstacle detected in front! Stopping the robot.")
                linear_x = 0.0
                linear_y = 0.0
                angular_z = 0.0
                self.send_command(linear_x, linear_y, angular_z)
                return
            else:
                distance_to_goal_x = goal_x - current_x
                if abs(distance_to_goal_x) < threshold:
                    linear_x = 0.0
                    self.state = 'MOVE_Y'
                else:
                    linear_x = 0.06 if distance_to_goal_x > 0.1 else 0.05 if distance_to_goal_x > 0 else -0.06 if distance_to_goal_x < -0.1 else -0.05
                linear_y = 0.0
                angular_z = 0.0

        elif self.state == 'MOVE_Y':
            time.sleep(1)
            distance_to_goal_y = goal_y - current_y
            if abs(distance_to_goal_y) < threshold:
                linear_y = 0.0
                self.send_command(0.0, 0.0, 0.0)
                self.get_logger().info(" Reached goal. Checking final position and yaw...")
                if abs(current_x - goal_x) < threshold and abs(current_y - goal_y) < threshold and abs(current_yaw - goal_yaw) < yaw_threshold:
                    self.get_logger().info(f"Robot has reached goal at Point {'A' if self.current_goal_index == 0 else 'B' if self.current_goal_index == 1 else 'C'}.")
                    self.get_logger().info("Waiting 'P'")
                    self.wait_for_response('P')
                    if self.current_goal_index == 0:
                        self.send_command(0.0, 0.0, 0.0)
                        self.current_goal_index += 1
                    if self.current_goal_index < len(self.goals):
                        next_goal = self.goals[self.current_goal_index]
                        self.set_goal(*next_goal)
                        if self.current_goal_index == 1:
                            time.sleep(1)
                            self.get_logger().info("Reached goal B. Sending 'B'...")
                            self.send_character('A')
                            self.get_logger().info("Waiting 'N'")
                            self.wait_for_response('N')
                            self.get_logger().info("Received 'N', proceeding to Point C.")
                            self.current_goal_index += 1
                        if self.current_goal_index == 2:
                            time.sleep(1)
                            self.get_logger().info("Reached goal C. Sending 'C'...")
                            self.send_character('I')
                            self.get_logger().info("Waiting 'M'")
                            self.wait_for_response('M')
                            self.get_logger().info("Received 'M', proceeding to Point A.")
                            self.current_goal_index += 1
                    else:
                        self.get_logger().info("All goals reached. Returning to Point A.")
                        self.set_goal(0.0, 0.0, 0.0)
                else:
                    self.get_logger().info("Robot is not aligned. Re-evaluating position...")
                    self.state = 'ALIGN_YAW'
                return
            else:
                if distance_to_goal_y > 0:
                    if self.distances['left']['static'] < 0.15:
                        self.get_logger().info("Obstacle detected on the left! Stopping the robot.")
                        linear_y = 0.0
                        linear_x = 0.0
                        angular_z = 0.0
                        self.send_command(linear_x, linear_y, angular_z)
                        return
                    else:
                        linear_y = 0.07 if distance_to_goal_y > 0.1 else 0.04
                else:
                    if self.distances['right']['static'] < 0.15:
                        self.get_logger().info("Obstacle detected on the right! Stopping the robot.")
                        linear_y = 0.0
                        linear_x = 0.0
                        angular_z = 0.0
                        self.send_command(linear_x, linear_y, angular_z)
                        return
                    else:
                        linear_y = -0.07 if distance_to_goal_y < -0.1 else -0.04
                linear_x = 0.0
                angular_z = 0.0

        self.send_command(linear_x, linear_y, angular_z)

    def send_command(self, linear_x, linear_y, angular_z):
        if self.command_in_progress:
            self.get_logger().warn("Previous command still in progress. Please wait.")
            return

        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn("Cannot send command: Port is not open.")
            return

        self.command_in_progress = True
        try:
            wheel_radius = 0.0485
            wheel_base = 0.38

            front_left_velocity = (linear_x - linear_y - (wheel_base * angular_z / 2))
            front_right_velocity = (linear_x + linear_y + (wheel_base * angular_z / 2))
            rear_left_velocity = (linear_x + linear_y - (wheel_base * angular_z / 2))
            rear_right_velocity = (linear_x - linear_y + (wheel_base * angular_z / 2))

            if (abs(front_left_velocity) > 0.27 or
                abs(front_right_velocity) > 0.27 or
                abs(rear_left_velocity) > 0.27 or
                abs(rear_right_velocity) > 0.27):
                self.get_logger().warn("One or more velocities are out of range. Command not sent.")
                return

            front_left_velocity *= 2.0
            front_right_velocity *= 2.0
            rear_left_velocity *= 2.0
            rear_right_velocity *= 2.0

            front_left_velocity = round(front_left_velocity, 2)
            front_right_velocity = round(front_right_velocity, 2)
            rear_left_velocity = round(rear_left_velocity, 2)
            rear_right_velocity = round(rear_right_velocity, 2)

            command = f"c:{front_left_velocity},{front_right_velocity},{rear_left_velocity},{rear_right_velocity}\n"
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sending command: {command.strip()}")

        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")
        finally:
            self.command_in_progress = False

    def send_character(self, character):
        """Send a character to the character serial port."""
        self.character_port.write(character.encode('utf-8'))
        self.get_logger().info(f"Character sent: {character.strip()}")

    def wait_for_response(self, expected_response):
        """Wait for a specific response from the character serial port."""
        self.get_logger().info(f"Waiting for response '{expected_response}'...")
        while True:
            if self.character_port.in_waiting > 0:
                response = self.character_port.read_until(b'\n').decode('utf-8').strip()
                self.get_logger().info(f"Received response: {response}")
                if response == expected_response:
                    self.get_logger().info(f"Received '{expected_response}', proceeding with the next steps.")
                    break
                else:
                    self.get_logger().warn(f"Unexpected response: {response}. Waiting for '{expected_response}'...")

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle."""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    node.set_goal(0.0, 0.0, 0.0)  # Start at Point A
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
