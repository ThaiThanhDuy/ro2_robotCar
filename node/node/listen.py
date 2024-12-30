import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import threading

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')
        
        # Create a buffer and listener for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically get the transformation
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Thread for processing transforms
        self.processing_thread = threading.Thread(target=self.process_transforms)
        self.processing_thread.start()

        self.latest_transform = None  # Store the latest transform

    def timer_callback(self):
        try:
            # Get the transformation from 'odom' to 'base_link'
            trans = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            self.latest_transform = trans  # Store the latest transform
        except Exception as e:
            self.get_logger().info(f'Could not get transform: {e}')

    def process_transforms(self):
        while rclpy.ok():
            if self.latest_transform is not None:
                self.process_transform(self.latest_transform)

    def process_transform(self, trans: TransformStamped):
        # Extract translation and rotation
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        rotation = trans.transform.rotation

        # Log the transformation
        self.get_logger().info(f'Transform from odom to base_link: x={x}, y={y}, z={z}, rotation={rotation}')

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
