import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.transform_broadcaster = TransformBroadcaster(self)

        # Update joint names to the specified names
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['left_wheel_joint_1', 'left_wheel_joint_2', 'right_wheel_joint_1', 'right_wheel_joint_2']
        self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]  # Initial positions
        self.joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0]  # Initial velocities

        self.timer = self.create_timer(0.1, self.publish_joint_state)  # Publish every 0.1 seconds

        # Initial base_link position and orientation
        self.base_link_x = 0.0
        self.base_link_y = 0.0
        self.base_link_z = 0.0
        self.orientation = 0.0  # Orientation in radians

        # Robot parameters
        self.wheel_base = 0.38  # Distance between left and right wheels (in meters)
        self.track_width = 0.3  # Distance between front and rear wheels (in meters)
        self.wheel_radius = 0.48  # Radius of the wheels (in meters)

    def publish_joint_state(self):
        # Here you can set desired linear and angular velocities
        linear_x = 0.1  # Forward speed in meters per second
        linear_y = 0.0  # Strafe speed in meters per second
        angular_z = 0.0  # Rotation speed in radians per second (change this to test rotation)

        # Calculate wheel speeds based on the desired velocities
        front_left_speed = (linear_x - linear_y - (self.wheel_base * angular_z)) / self.wheel_radius
        front_right_speed = (linear_x + linear_y + (self.wheel_base * angular_z)) / self.wheel_radius
        rear_left_speed = (linear_x + linear_y - (self.wheel_base * angular_z)) / self.wheel_radius
        rear_right_speed = (linear_x - linear_y + (self.wheel_base * angular_z)) / self.wheel_radius

        # Update joint positions based on calculated speeds
        self.joint_state_msg.position[0] += front_left_speed * 0.1  # Increment left wheel joint 1 position
        self.joint_state_msg.position[1] += rear_left_speed * 0.1  # Increment left wheel joint 2 position
        self.joint_state_msg.position[2] += front_right_speed * 0.1  # Increment right wheel joint 1 position
        self.joint_state_msg.position[3] += rear_right_speed * 0.1  # Increment right wheel joint 2 position

        # Update velocities
        self.joint_state_msg.velocity[0] = front_left_speed
        self.joint_state_msg.velocity[1] = rear_left_speed
        self.joint_state_msg.velocity[2] = front_right_speed
        self.joint_state_msg.velocity[3] = rear_right_speed

        # Update the timestamp
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the joint state message
        self.publisher_.publish(self.joint_state_msg)

        # Update base_link position and orientation based on wheel movements
        self.update_base_link_position(linear_x, linear_y, angular_z)

        # Create and publish the transform
        self.publish_transform()

        self.get_logger().info(f'Publishing: Position: {self.joint_state_msg.position}, Velocity: {self.joint_state_msg.velocity}, Base Link Position: ({self.base_link_x}, {self.base_link_y}, {self.base_link_z}), Orientation: {self.orientation}')

    def update_base_link_position(self, linear_x, linear_y, angular_z):
        # Update base_link position based on the desired linear and angular velocities
        dt = 0.1  # Time step
        self.orientation += angular_z * dt  # Update orientation
        self.base_link_x += (linear_x * math.cos(self.orientation) - linear_y * math.sin(self.orientation)) * dt
        self.base_link_y += (linear_x * math.sin(self.orientation) + linear_y * math.cos(self.orientation)) * dt
        # Assuming constant height for simplicity
        self.base_link_z = 0.0

    def publish_transform(self):
        # Create a TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame
        t.child_frame_id = 'base_link'  # Child frame
        t.transform.translation.x = self.base_link_x
        t.transform.translation.y = self.base_link_y
        t.transform.translation.z = self.base_link_z
        
        # Set rotation based on the current orientation
        quaternion = self.euler_to_quaternion(0.0, 0.0, self.orientation)
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Send the transform
        self.transform_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()