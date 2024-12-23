import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import serial  # Ensure you have the pyserial package installed

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # Create a buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize serial port (replace with your actual port)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust as necessary
        self.command_in_progress = False

        # Timer to periodically get the transformation
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval

        # List to store goal points
        self.goal_points = [(3.0, 0.0), (5.0, 0.0)]  # Example goal points

    def timer_callback(self):
        try:
            # Get the transformation from 'odom' to 'base_link'
            trans = self.tf_buffer.lookup_transform('base_link', ' odom', rclpy.time.Time())
            self.process_transform(trans)
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

    def process_transform(self, trans: TransformStamped):
        # Extract translation
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        
        # Convert quaternion to yaw
        yaw = self.quaternion_to_yaw(trans.transform.rotation)

        # Log the current position
        self.get_logger().info(f'Current Position: x={x}, y={y}, Yaw={yaw}')

        # Control the robot based on the goal points
        for goal in self.goal_points:
            self.control_robot(x, y, yaw, goal)

    def control_robot(self, current_x, current_y, current_yaw, goal):
        goal_x, goal_y = goal

        # Calculate the distance to the goal
        distance_to_goal_x = goal_x - current_x
        distance_to_goal_y = goal_y - current_y

        # Calculate the desired yaw angle based on the goal position
        desired_yaw = math.atan2(distance_to_goal_y, distance_to_goal_x)

        # Normalize the yaw difference to be within -π to π
        yaw_difference = desired_yaw - current_yaw
        if yaw_difference > math.pi:
            yaw_difference -= 2 * math.pi
        elif yaw_difference < -math.pi:
            yaw_difference += 2 * math.pi

        # Set linear and angular velocities
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        # Control logic for yaw adjustment
        if abs(yaw_difference) > 0.1:  # Threshold to avoid oscillation
            if yaw_difference > 0:
                # Spin left
                angular_z = 0.5  # Left spin speed
            else:
                # Spin right
                angular_z = -0.5  # Right spin speed
        else:
            # Stop spinning if within threshold
            angular_z = 0.0

            # Now that yaw is aligned, move towards the goal
            if distance_to_goal_x > 0:
                # Move forward
                linear_x = 0.5  # Forward speed
            elif distance_to_goal_x < 0:
                # Move backward
                linear_x = -0.5  # Backward speed
            else:
                # Stop if at the goal in x direction
                linear_x = 0.0

            # Control logic for y direction
            if distance_to_goal_y > 0:
                # Move left (positive y)
                linear_y = 0.5  # Left speed
            elif distance_to_goal_y < 0:
                # Move right (negative y)
                linear_y = -0.5  # Right speed
            else:
                # Stop if at the goal in y direction
                linear_y = 0.0

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
            wheel_base = 0.38       # Distance between front and rear wheels

            # Calculate individual wheel velocities
            front_left_velocity = (linear_x - linear_y - (wheel_base * angular_z / 2)) / wheel_radius
            front_right_velocity = (linear_x + linear_y + (wheel_base * angular_z / 2)) / wheel_radius
            rear_left_velocity = (linear_x + linear_y - (wheel_base * angular_z / 2)) / wheel_radius
            rear_right_velocity = (linear_x - linear_y + (wheel_base * angular_z / 2)) / wheel_radius

            # Check if velocities are within the range -0.27 to 0.27
            if (abs(front_left_velocity) > 0.27 or
                abs(front_right_velocity) > 0.27 or
                abs(rear_left_velocity) > 0.27 or
                abs(rear_right_velocity) > 0.27):
                self.get_logger().warn("One or more velocities are out of range. Command not sent.")
                return  # Skip sending the command if any velocity is out of range

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
        # Convert quaternion to yaw (Z-axis rotation)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
