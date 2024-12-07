import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from urdf_parser_py.urdf import URDF
import os

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Create a timer to publish transforms at a fixed rate
        self.timer = self.create_timer(0.01, self.publish_transforms)  # 10 Hz

    def publish_transforms(self):
        # Get the current time
        current_time = self.get_clock().now().to_msg()
         # Publish transform from map to odom
        self.publish_transform(current_time, 'map', 'odom', 0.0, 0.0, 0.0)

        # Publish transform from odom to base_footprint
      #  self.publish_transform(current_time, 'odom', 'base_link', 0.0, 0.0, 0.0)

        # Publish transform from base_footprint to base_link
    #    self.publish_transform(current_time, 'base_link', 'base_footprint', 0.0, 0.0, 0.0)

   
    def publish_transform(self, current_time, parent_frame, child_frame, x, y, z):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = current_time
        transform_msg.header.frame_id = parent_frame
        transform_msg.child_frame_id = child_frame

        # Set translation
        transform_msg.transform.translation.x = x
        transform_msg.transform.translation.y = y
        transform_msg.transform.translation.z = z

        # Set rotation (identity quaternion)
        transform_msg.transform.rotation.x = 0.0
        transform_msg.transform.rotation.y = 0.0
        transform_msg.transform.rotation.z = 0.0
        transform_msg.transform.rotation.w = 1.0

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform_msg)

def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TFPublisher()
    rclpy.spin(tf_publisher)
    tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()