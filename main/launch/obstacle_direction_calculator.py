import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class ObstacleDirectionCalculator(Node):
    def __init__(self):
        super().__init__('obstacle_direction_calculator')
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid,
            'global_costmap',  # Replace with your global costmap topic
            self.costmap_callback,
            10
        )
        self.robot_position = np.array([0.0, 0.0])  # Robot's position in base_link frame
        self.robot_orientation = 0.0  # Robot's orientation in radians (0 is facing positive Y)

    def costmap_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Define directions (in grid cells)
        directions = {
            'front': (0, 1),   # Positive Y direction
            'back': (0, -1),   # Negative Y direction
            'left': (-1, 0),   # Negative X direction
            'right': (1, 0)    # Positive X direction
        }

        # Initialize distances
        distances = {key: float('inf') for key in directions.keys()}

        # Iterate through the costmap data
        for direction, (dx, dy) in directions.items():
            for step in range(1, 10):  # Check up to 10 cells in each direction
                x_index = int((self.robot_position[0] + dx * step) / resolution)
                y_index = int((self.robot_position[1] + dy * step) / resolution)

                if 0 <= x_index < width and 0 <= y_index < height:
                    index = x_index + y_index * width
                    cost_value = msg.data[index]

                    # Check for obstacles
                    if cost_value == 100:  # Assuming 100 indicates an obstacle
                        obstacle_x = origin_x + x_index * resolution
                        obstacle_y = origin_y + y_index * resolution
                        obstacle_position = np.array([obstacle_x, obstacle_y])

                        # Calculate distance to the robot
                        distance = np.linalg.norm(obstacle_position - self.robot_position)
                        distances[direction] = min(distances[direction], distance)
                        break  # Stop checking further in this direction

        # Log distances to obstacles in each direction
        for direction, distance in distances.items():
            if distance == float('inf'):
                self.get_logger().info(f'No obstacle detected in {direction} direction.')
            else:
                self.get_logger().info(f'Distance to obstacle in {direction} direction: {distance:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    obstacle_direction_calculator = ObstacleDirectionCalculator()
    rclpy.spin(obstacle_direction_calculator)
    obstacle_direction_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
