import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import tkinter as tk
from threading import Thread

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

        # Initialize velocities
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        # Start the GUI in a separate thread
        self.gui_thread = Thread(target=self.start_gui)
        self.gui_thread.start()

    def start_gui(self):
        # Create a simple GUI
        self.root = tk.Tk()
        self.root.title("Robot Control")

        tk.Label(self.root, text="Linear X:").grid(row=0, column=0)
        self.linear_x_entry = tk.Entry(self.root)
        self.linear_x_entry.grid(row=0, column=1)

        tk.Label(self.root, text="Linear Y:").grid(row=1, column=0)
        self.linear_y_entry = tk.Entry(self.root)
        self.linear_y_entry.grid(row=1, column=1)

        tk.Label(self.root, text="Angular Z:").grid(row=2, column=0)
        self.angular_z_entry = tk.Entry(self.root)
        self.angular_z_entry.grid(row=2, column=1)

        tk.Button(self.root, text="Update", command=self.update_velocities).grid(row=3, columnspan=2)

        self.root.mainloop()

    def update_velocities(self):
        # Update velocities from GUI input
        try:
            self.linear_x = float(self.linear_x_entry.get())
            self.linear_y = float(self.linear_y_entry.get())
            self.angular_z = float(self.angular_z_entry.get())
        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")

    def publish_joint_state(self):
        # Calculate wheel speeds based on the desired velocities
        front_left_speed = (self.linear_x - self.linear_y - (self.wheel_base * self.angular_z)) / self.wheel_radius
        front_right_speed = (self.linear_x + self.linear_y + (self.wheel_base * self.angular_z)) / self.wheel_radius
        rear_left_speed = (self.linear_x + self.linear_y - (self.wheel_base * self.angular_z)) / self.wheel_radius
        rear_right_speed = (self.linear_x - self.linear_y + (self.wheel_base * self.angular_z)) / self.wheel_radius

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
        self.update_base_link_position(self.linear_x, self.linear_y, self.angular_z)

        # Create and publish the transforms
        self.publish_transform()

        self.get_logger().info(f'Publishing: Position: {self.joint_state_msg.position}, Velocity: {self.joint_state_msg.velocity}, Base Link Position: ({self.base_link_x}, {self.base_link_y}, {self.base_link_z}), Orientation: {self.orientation}')

    def update_base_link_position(self, linear_x, linear_y, angular_z):
        # Update base_link position based on the desired linear and angular velocities
        dt = 0.1  # Time step
        self.orientation += angular_z * dt  # Accumulate orientation
        self.base_link_x += (linear_x * math.cos(self.orientation) - linear_y * math.sin(self.orientation)) * dt
        self.base_link_y += (linear_x * math.sin(self.orientation) + linear_y * math.cos(self.orientation)) * dt
        # Assuming constant height for simplicity
        self.base_link_z = 0.0

    def publish_transform(self):
        # Create a TransformStamped message for base_link
        t_base_link = TransformStamped()
        t_base_link.header.stamp = self.get_clock().now().to_msg()
        t_base_link.header.frame_id = 'odom'  # Parent frame
        t_base_link.child_frame_id = 'base_link'  # Child frame
        t_base_link.transform.translation.x = self.base_link_x
        t_base_link.transform.translation.y = self.base_link_y
        t_base_link.transform.translation.z = self.base_link_z
        
        # Set rotation based on the current orientation
        quaternion = self.euler_to_quaternion(0.0, 0.0, self.orientation)
        t_base_link.transform.rotation.x = quaternion[0]
        t_base_link.transform.rotation.y = quaternion[1]
        t_base_link.transform.rotation.z = quaternion[2]
        t_base_link.transform.rotation.w = quaternion[3]

        # Send the base_link transform
        self.transform_broadcaster.sendTransform(t_base_link)

        # Create a TransformStamped message for odom
        t_odom = TransformStamped()
        t_odom.header.stamp = self.get_clock().now().to_msg()
        t_odom.header.frame_id = 'map'  # Parent frame
        t_odom.child_frame_id = 'odom'  # Child frame
        t_odom.transform.translation.x = 0.0  # Assuming the odom frame is at the origin of the map
        t_odom.transform.translation.y = 0.0
        t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation.x = 0.0
        t_odom.transform.rotation.y = 0.0
        t_odom.transform.rotation.z = 0.0
        t_odom.transform.rotation.w = 1.0  # No rotation

        # Send the odom transform
        self.transform_broadcaster.sendTransform(t_odom)

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