import rclpy
from rclpy.node import Node
from nav2_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import numpy as np

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

        # Subscribers
        self.global_costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            'global_costmap/costmap',
            self.global_costmap_callback,
            10
        )

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize state
        self.current_position = (0, 0)
        self.current_orientation = 0  # In radians
        self.goal_position = (1, 1)  # Example goal position

    def global_costmap_callback(self, msg):
        # Process the received global cost map
        self.get_logger().info('Received global cost map')
        cost_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # Call the DWA algorithm to compute the velocity command
        cmd_vel = self.compute_dwa(cost_data)

        # Log the computed cmd_vel
        self.get_logger().info(f'Publishing cmd_vel: linear.x={cmd_vel.linear.x}, linear.y={cmd_vel.linear.y}, angular.z={cmd_vel.angular.z}')

        self.cmd_vel_publisher.publish(cmd_vel)

    def compute_dwa(self, cost_data):
        # Sample velocities
        best_cmd = Twist()
        best_score = float('-inf')

        for vx in np.arange(self.min_vel_x, self.max_vel_x, 0.1):
            for vy in np.arange(self.min_vel_y, self.max_vel_y, 0.1):
                for vtheta in np.arange(self.min_vel_theta, self.max_vel_theta, 0.1):
                    # Simulate the trajectory
                    trajectory = self.simulate_trajectory(vx, vy, vtheta)

                    # Evaluate the trajectory
                    score = self.evaluate_trajectory(trajectory, cost_data)

                    if score > best_score:
                        best_score = score
                        best_cmd.linear.x = vx
                        best_cmd.linear.y = vy
                        best_cmd.angular.z = vtheta

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

def main(args=None):
    rclpy.init(args=args)
    dwa_node = DWA()
    rclpy.spin(dwa_node)
    dwa_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
