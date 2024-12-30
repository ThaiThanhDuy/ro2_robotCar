import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import serial  # Ensure you have the pyserial package installed
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import heapq

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

        # Define the sequence of goals
        self.goals = [
            (0.0, 0.0, 0.0),  # Point A
            (1.0, 0.0, 0.0),  # Point B
            (2.0, 0.0, 0.0)   # Point C
        ]
        self.current_goal_index = 0  # Start with the first goal
        self.goal = self.goals[self.current_goal_index]  # Initialize goal

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Initialize distances dictionary
        self.distances = {
            'front': float('inf'),
            'back': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
        }

        # Flag to indicate if obstacles were detected
        self.obstacles_detected = False
        self.obstacle_threshold = 0.5

        # Subscribers
        self.local_costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            'local_costmap/costmap',
            self.local_costmap_callback,
            10
        )

        # Publisher for the path
        self.path_publisher = self.create_publisher(String, 'path', 10)

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize the cost map
        self.cost_map = None

        self.current_position = (0.0, 0.0)  # Example starting position

    def scan_callback(self, msg):
        # Get the number of ranges
        num_ranges = len(msg.ranges)

        # Define the angles for each direction
        directions = {
            'front': num_ranges // 2,                # 0 degrees
            'back': 0,   # 180 degrees
            'left': (3 * num_ranges) // 4,   # 90 degrees
            'right': num_ranges // 4 , # 270 degrees
        }

        # Calculate distances for each direction
        for direction, index in directions.items():
            distance = msg.r anges[index]
            if distance < 1.0:  # Check if the distance is less than 1 meter
                self.distances[direction] = min(self.distances[direction], distance)
                self.obstacles_detected = True  # Set flag to indicate obstacles are detected
            else:
                self.distances[direction] = float('inf')

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
        
        # Update current position
        self.current_position = (x, y)

        # Control the robot based on the goal
        self.control_robot(x, y, yaw)

    def control_robot(self, current_x, current_y, current_yaw):
        goal_x, goal_y, goal_yaw = self.goal

        self.get_logger().info(f'Current Position: {self.current_position}, Goal Position: {self.goal}')

        # Call Dijkstra's algorithm to compute the path
        path = self.compute_dijkstra(self.current_position, self.goal)

        if path:
            self.path_publisher.publish(String(data=str(path)))
            self.follow_path(path)

    def compute_dijkstra(self, start, goal):
        # Check if the cost map is available
        if self.cost_map is None:
            self.get_logger().warn('Cost map not available')
            return []

        rows, cols = self.cost_map.shape
        distances = {start: 0}  # Distance from start to each node
        priority_queue = [(0, start)]  # Priority queue for exploring nodes
        came_from = {}  # To reconstruct the path

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            # If we reached the goal, reconstruct and return the path
            if current_node == goal:
                return self.reconstruct_path(came_from, current_node)

            # Explore neighbors (4 possible directions)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current_node[0] + dx, current_node[1] + dy)

                # Check if the neighbor is within bounds and not an obstacle
                if (0 <= neighbor[0] < rows and
                        0 <= neighbor[1] < cols and
                        self.cost_map[neighbor] < 255):  # Check if not an obstacle
                    distance = current_distance + 1  # Increment distance

                    # If this path to neighbor is shorter, record it
                    if neighbor not in distances or distance < distances[neighbor]:
                        distances[neighbor] = distance
                        came_from[neighbor] = current_node
                        heapq.heappush(priority_queue, (distance, neighbor))

        return []  # Return an empty path if no path is found

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]  # Return reversed path

    def follow_path(self, path):
        # Generate velocity commands to follow the path
        if len(path) < 2:
            return  # No movement if the path is too short

        # Get the next position in the path
        next_position = path[1]  # The next position to move towards
        current_x, current_y = self.current_position

        # Calculate the direction to the next position
        target_x, target_y = next_position
        direction_x = target_x - current_x
        direction_y = target_y - current_y

        # Calculate the distance to the next position
        distance = np.sqrt(direction_x**2 + direction_y**2)

        # Create a Twist message for velocity commands
        cmd_vel = Twist()

        # Set linear velocity (move towards the target)
        max_linear_speed = 0.5  # Maximum speed
        min_distance = 0.1  # Minimum distance to consider "close enough"

        if distance > min_distance:
            # Set linear velocities in both x and y directions
            cmd_vel.linear.x = min(distance, max_linear_speed) * (direction_x / distance)  # Proportional control in x
            cmd_vel.linear.y = min(distance, max_linear_speed) * (direction_y / distance)  # Proportional control in y
        else:
            cmd_vel.linear.x = 0.0  # Stop if close enough to the target
            cmd_vel.linear.y = 0.0  # Stop if close enough to the target

        # Set angular velocity (turn towards the target)
        if distance > 0:
            angle_to_target = np.arctan2(direction_y, direction_x)
            cmd_vel.angular.z = angle_to_target - current_yaw  # Set angular velocity to turn towards the target
        else:
            cmd_vel.angular.z = 0.0  # No rotation if already at the target

        # Log the cmd_vel values
        self.get_logger().info(f'cmd_vel - Linear X: {cmd_vel.linear.x}, Linear Y: {cmd_vel.linear.y}, Angular Z: {cmd_vel.angular.z}')

        # Publish the velocity command
        self.send_command(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)

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
