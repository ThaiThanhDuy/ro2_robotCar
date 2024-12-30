import rclpy
from rclpy.node import Node
from nav2_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class DWA(Node):
    def __init__(self):
        super().__init__('custom_dwa_node')

        # Parameters
        self.max_vel_x = 0.5
        self.min_vel_x = -0.5
        self.max_vel_y = 0.5  # Maximum lateral velocity
        self.min_vel_y = -0.5  # Minimum lateral velocity
        self.max_vel_theta = 1.0
        self.min_vel_theta = -1.0
        self.acc_lim_x = 1.0
        self.acc_lim_y = 1.0  # Lateral acceleration limit
        self.acc_lim_theta = 1.0
        self.yaw_goal_tolerance = 0.1
        self.xy_goal_tolerance = 0.1

        # Create a buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.global_costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.global_costmap_callback,
            10
        )

        # Timer to update the robot's position and orientation
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Initialize state
        self.current_position = [0, 0]  # Use a list to allow updates
        self.current_orientation = 0  # In radians
        self.goal_position = (1, 1)  # Example goal position

        # Condition flag
        self.condition_met = False  # Set this based on your logic

    def timer_callback(self):
        try:
            # Get the transformation from 'odom' to 'base_link'
            trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            self.process_transform(trans)
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

        # Check the condition and call the global_costmap_callback if met
        if self.condition_met:
            # Create a dummy OccupancyGrid message for demonstration
            dummy_msg = OccupancyGrid()
            # Populate dummy_msg with appropriate data here
            self.global_costmap_callback(dummy_msg)

    def process_transform(self, trans: TransformStamped):
        # Extract translation
        x = -trans.transform.translation.x
        y = -trans.transform.translation.y
        self.current_position[0] = x
        self.current_position[1] = y
        # Convert quaternion to yaw
        yaw = -self.quaternion_to_yaw(trans.transform.rotation)
        self.current_orientation = yaw  # Update orientation

        # Log the current position
        self.get_logger().info(f'Current Position: x={x}, y={y}, Yaw={yaw}')

    def global_costmap_callback(self, msg):
        # Process the received global cost map
        self.get_logger().info('Received global cost map')
        cost_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # Call the DWA algorithm to compute the velocity command
        linear_x, linear_y, angular_z = self.compute_dwa(cost_data)

        # Log the computed velocities
        self.get_logger().info(f'Publishing velocities: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}')

        # Here you would publish the velocities to the appropriate topic if needed
        self.send_command(linear_x, linear_y, angular_z)

    def compute_dwa(self, cost_data):
        # Sample velocities
        best_cmd = (0.0, 0.0, 0.0)  # (linear_x, linear_y, angular_z)
        best_score = float('- inf')

        for vx in np.arange(self.min_vel_x, self.max_vel_x, 0.1):
            for vy in np.arange(self.min_vel_y, self.max_vel_y, 0.1):
                for vtheta in np.arange(self.min_vel_theta, self.max_vel_theta, 0.1):
                    # Simulate the trajectory
                    trajectory = self.simulate_trajectory(vx, vy, vtheta)

                    # Evaluate the trajectory
                    score = self.evaluate_trajectory(trajectory, cost_data)

                    if score > best_score:
                        best_score = score
                        best_cmd = (vx, vy, vtheta)

        return best_cmd

    def simulate_trajectory(self, vx, vy, vtheta):
        # Simulate the robot's trajectory based on the given velocities
        trajectory = []
        for t in np.linspace(0, 1, num=10):  # Simulate for 1 second
            x = self.current_position[0] + vx * t * np.cos(self.current_orientation) - vy * t * np.sin(self.current_orientation)
            y = self.current_position[1] + vx * t * np.sin(self.current_orientation) + vy * t * np.cos(self.current_orientation)
            theta = self.current_orientation + vtheta * t
            trajectory.append((x, y, theta))
        return trajectory

    def evaluate_trajectory(self, trajectory, cost_data):
        # Evaluate the trajectory based on the cost map
        score = 0
        for (x, y, theta) in trajectory:
            grid_x = int(x)  # Convert to grid coordinates
            grid_y = int(y)
            if 0 <= grid_x < cost_data.shape[1] and 0 <= grid_y < cost_data.shape[0]:
                cost = cost_data[grid_y, grid_x]
                if cost == 0:  # Free space
                    score += 1
                elif cost == 255:  # Occupied space
                    score -= 10  # Penalize for hitting obstacles
        return score

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
            self.command_in_progress = False

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
    dwa_node = DWA()
    rclpy.spin(dwa_node)
    dwa_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
