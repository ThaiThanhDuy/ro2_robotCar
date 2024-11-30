import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.transform_broadcaster = TransformBroadcaster(self)

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['left_wheel_joint_1', 'left_wheel_joint_2', 'right_wheel_joint_1', 'right_wheel_joint_2']
        self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]
        self.joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0]

        self.timer = self.create_timer(0.1, self.publish_joint_state)

        self.base_link_x = 0.0
        self.base_link_y = 0.0
        self.base_link_z = 0.0
        self.orientation = 0.0
        
        self.linear_x = 0.0
        self.linear_y = 0.0 
        self.angular_z = 0.0

        self.wheel_base = 0.38
        self.track_width = 0.3
        self.wheel_radius = 0.48

        self.serial_port = None
        self.timer_uart = None

        # Initialize GUI
        self.init_gui()

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("Joint State Publisher")

        # COM Port Selection
        self.com_port_label = tk.Label(self.root, text="Select COM Port:")
        self.com_port_label.pack()

        self.com_ports = [port.device for port in serial.tools.list_ports.comports()]
        self.com_port_var = tk.StringVar(value=self.com_ports[0] if self.com_ports else "")
        self.com_port_dropdown = ttk.Combobox(self.root, textvariable=self.com_port_var, values=self.com_ports)
        self.com_port_dropdown.pack()

        self.open_button = tk.Button(self.root, text="Open Port", command=self.open_port)
        self.open_button.pack()

        # Linear and Angular Velocity Inputs
        self.linear_x_entry = tk.Entry(self.root)
        self.linear_x_entry.pack()
        self.linear_x_entry.insert(0, "Linear X")

        self.linear_y_entry = tk.Entry(self.root)
        self.linear_y_entry.pack()
        self.linear_y_entry.insert(0, "Linear Y")

        self.angular_z_entry = tk.Entry(self.root)
        self.angular_z_entry.pack()
        self.angular_z_entry.insert(0, "Angular Z")

        self.set_velocity_button = tk.Button(self.root, text="Set Velocity", command=self.set_velocity)
        self.set_velocity_button.pack()

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.after(100, self.update_gui)  # Update GUI periodically

    def open_port(self):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        
        try:
            self.serial_port = serial.Serial(self.com_port_var.get(), 115200, timeout=1)
            self.timer_uart = self.create_timer(0.1, self.read_uart)
            self.get_logger().info(f"Opened port: {self.com_port_var.get()}")
        except Exception as e:
            self.get_logger().error(f"Failed to open port: {e}")

    def set_velocity(self):
        try:
            self.linear_x = float(self.linear_x_entry.get())
            self.linear_y = float(self.linear_y_entry.get())
            self.angular_z = float(self.angular_z_entry.get())
            self.get_logger().info(f"Set velocities: Linear X: {self.linear_x}, Linear Y: {self.linear_y}, Angular Z: {self.angular_z}")
        except ValueError:
            self.get_logger().error("Invalid input for velocities. Please enter numeric values.")

    def update_gui(self):
        self.root.update_idletasks()
        self.root.update()
        self.get_logger().info("GUI updated.")
        self.root.after(100, self.update_gui)  # Continue updating the GUI

    def on_closing(self):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()

    def publish_joint_state(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state_msg)

        self.update_base_link_position(self.linear_x, self.linear_y, self.angular_z)
        self.publish_transform()

        self.get_logger().info(f'Publishing: Position: {self.joint_state_msg.position}, Velocity: {self.joint_state_msg.velocity}, Base Link Position: ({self.base_link_x}, {self.base_link_y}, {self.base_link_z}), Orientation: {self.orientation}')

    def read_uart(self):
        if self.serial_port and self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f"Received data: {data}")

            try:
                parts = data.split()
                pos = []
                vel = []

                for part in parts:
                    if part.startswith('pos'):
                        pos_value = float(part.split(':')[1])
                        pos.append(pos_value)
                    elif part.startswith('vel'):
                        vel_value = float(part.split(':')[1])
                        vel.append(vel_value)

                if len(pos) == 4 and len(vel) == 4:
                    self.joint_state_msg.position = [round(p, 6) for p in pos]
                    self.joint_state_msg.velocity = [round(v, 6) for v in vel]
                    self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                    self.linear_x = (vel[0] + vel[1] + vel[2] + vel[3]) / (4 * self.wheel_radius)
                    self.linear_y = (-vel[0] + vel[1] + vel[2] - vel[3]) / (4 * self.wheel_radius)
                    self.angular_z = (-vel[0] + vel[1] - vel[2] + vel[3]) / (4 * self.wheel_base)

                    self.publish_joint_state()
                else:
                    self.get_logger().error("Received data does not contain exactly 4 positions and 4 velocities.")

            except ValueError as e:
                self.get_logger().error(f"Error parsing data: {e}")

    def update_base_link_position(self, linear_x, linear_y, angular_z):
        dt = 0.1
        self.orientation += angular_z * dt
        self.base_link_x += (linear_x * math.cos(self.orientation) - linear_y * math.sin(self.orientation)) * dt
        self.base_link_y += (linear_x * math.sin(self.orientation) + linear_y * math.cos(self.orientation)) * dt
        self.base_link_z = 0.0

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.base_link_x
        t.transform.translation.y = self.base_link_y
        t.transform.translation.z = self.base_link_z
        
        quaternion = self.euler_to_quaternion(0.0, 0.0, self.orientation)
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.transform_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw /2)
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