import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDistanceCalculator(Node):
    def __init__(self):
        super().__init__('obstacle_distance_calculator')
        
        # Subscribe to the LaserScan data
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
            'right': float('inf')
        }

    def scan_callback(self, msg):
        # Get the number of ranges
        num_ranges = len(msg.ranges)

        # Define the angles for each direction
        directions = {
            'front': 0,                # 0 degrees
            'back': num_ranges // 2,   # 180 degrees
            'left': num_ranges // 4,   # 90 degrees
            'right': (3 * num_ranges) // 4  # 270 degrees
        }

        # Calculate distances for each direction
        for direction, index in directions.items():
            distance = msg.ranges[index]
            if distance < 1.0:  # Check if the distance is less than 1 meter
                self.distances[direction] = min(self.distances[direction], distance)

        # Log the distances
        self.log_distances()

    def log_distances(self):
        for direction, distance in self.distances.items():
            if distance == float('inf'):
                self.get_logger().info(f'No obstacle detected in {direction} direction.')
            else:
                self.get_logger().info(f'Distance to obstacle in {direction} direction: {distance:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    obstacle_distance_calculator = ObstacleDistanceCalculator()
    rclpy.spin(obstacle_distance_calculator)
    obstacle_distance_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
