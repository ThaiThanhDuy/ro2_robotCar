import rclpy
from rclpy.node import Node
from nav2_costmap_2d import Costmap2DROS
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class ObstacleDistance(Node):
    def __init__(self):
        super().__init__('obstacle_distance')
        self.costmap = Costmap2DROS('/global_costmap/costmap', self)
        self.costmap.start()

        # Subscribe to the robot's pose
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # Change this to your pose topic if different
            self.pose_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.check_obstacle_distances)  # Check every second

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def check_obstacle_distances(self):
        if not hasattr(self, 'current_pose'):
            self.get_logger().warn('Current pose not available yet.')
            return

        # Get the robot's position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Define directions (front, back, left, right)
        directions = {
            'front': (robot_x + 1.0, robot_y),  # 1 meter in front
            'back': (robot_x - 1.0, robot_y),   # 1 meter behind
            'left': (robot_x, robot_y + 1.0),   # 1 meter to the left
            'right': (robot_x, robot_y - 1.0)    # 1 meter to the right
        }

        for direction, (x, y) in directions.items():
            distance = self.get_distance_to_obstacle(x, y)
            self.get_logger().info(f'Distance to {direction}: {distance:.2f} meters')

    def get_distance_to_obstacle(self, x, y):
        # Get the cost at the specified position
        cost = self.costmap.getCost(x, y)

        # If the cost is greater than 0, there is an obstacle
        if cost > 0:
            return 0.0  # Obstacle is at this position

        # If no obstacle, return a large distance (or calculate the distance to the nearest obstacle)
        return float('inf')  # Placeholder for actual distance calculation

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
