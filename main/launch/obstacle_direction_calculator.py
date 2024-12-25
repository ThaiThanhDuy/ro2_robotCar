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

        # Timer to check for obstacles every second
        self.timer = self.create_timer(1.0, self.check_for_obstacles)

        # Initialize distances dictionary
        self.distances = {
            'front': {'static': float('inf'), 'dynamic': float('inf')},
            'back': {'static': float('inf'), 'dynamic': float('inf')},
            'left': {'static': float('inf'), 'dynamic': float('inf')},
            'right': {'static': float('inf'), 'dynamic': float('inf')}
        }

        # Flag to indicate if obstacles were detected
        self.obstacles_detected = False

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
                # Here, we would need additional logic to classify the obstacle
                # For demonstration, we will assume all detected obstacles are static
                self.distances[direction]['static'] = min(self.distances[direction]['static'], distance)
                self.obstacles_detected = True  # Set flag to indicate obstacles are detected
            else:
                # Reset the distance if no obstacle is detected
                self.distances[direction]['static'] = float('inf')

        # Log the distances
        self.log_distances()

    def check_for_obstacles(self):
        if not self.obstacles_detected:
            self.get_logger().info('No obstacles detected. Performing update...')
            # Here you can add logic to perform when no obstacles are detected
            self.perform_update()
        else:
            self.obstacles_detected = False  # Reset the flag for the next check

    def perform_update(self):
        # Logic to perform when no obstacles are detected
        self.get_logger().info('Performing obstacle detection update...')

    def log_distances(self):
        for direction, types in self.distances.items():
            for obstacle_type, distance in types.items():
                if distance == float('inf'):
                    self.get_logger().info(f'No {obstacle_type} obstacle detected in {direction} direction.')
                else:
                    self.get_logger().info(f'Distance to {obstacle_type} obstacle in {direction} direction: {distance:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    obstacle_distance_calculator = ObstacleDistanceCalculator()
    rclpy.spin(obstacle_distance_calculator)
    obstacle_distance_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
