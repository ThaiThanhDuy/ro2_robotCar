import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # Create a buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically get the transformation
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval

    def timer_callback(self):
        try:
            # Get the transformation from 'odom' to 'base_link'
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.process_transform(trans)
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

    def process_transform(self, trans: TransformStamped):
        # Extract translation
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        
        # Convert quaternion to yaw
        yaw = self.quaternion_to_yaw(trans.transform.rotation)

        # Log the results
        self.get_logger().info(f'Position: x={x}, y={y}, Yaw={yaw}')

    def quaternion_to_yaw(self, rotation):
        # Extract quaternion components
        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w
        
        # Calculate yaw from quaternion
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
