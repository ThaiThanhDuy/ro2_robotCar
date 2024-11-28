import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import messagebox
import serial

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.serial_connection = None

        # Automatically connect to UART
        if not self.connect_uart():
            self.get_logger().error("Failed to connect to UART on /dev/ttyUSB0")

    def publish_twist(self):
        self.publisher.publish(self.twist)
        self.get_logger().info(f'Published twist: linear.x={self.twist.linear.x}, linear.y={self.twist.linear.y}, angular.z={self.twist.angular.z}')

    def set_twist(self, linear_x, linear_y, angular_z):
        self.twist.linear.x = linear_x
        self.twist.linear.y = linear_y
        self.twist.angular.z = angular_z

    def connect_uart(self, port='/dev/ttyUSB0', baudrate=9600):
        try:
            self.serial_connection = serial.Serial(port, baudrate)
            self.get_logger().info(f'Connected to UART on {port}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to UART: {e}')
            return False

class CarControlGUI:
    def __init__(self, master, publisher_node):
        self.master = master
        self.publisher_node = publisher_node
        self.master.title("Car Control GUI")

        # Create input fields
        tk.Label(master, text="Linear X:").grid(row=0, column=0)
        self.linear_x_entry = tk.Entry(master)
        self.linear_x_entry.grid(row=0, column=1)

        tk.Label(master, text="Linear Y:").grid(row=1, column=0)
        self.linear_y_entry = tk.Entry(master)
        self.linear_y_entry.grid(row=1, column=1)

        tk.Label(master, text="Angular Z:").grid(row=2, column=0)
        self.angular_z_entry = tk.Entry(master)
        self.angular_z_entry.grid(row=2, column=1)

        # Create button to publish
        self.publish_button = tk.Button(master, text="Publish Twist", command=self.publish_twist)
        self.publish_button.grid(row=3, columnspan=2)

       

    def publish_twist(self):
        try:
            linear_x = float(self.linear_x_entry.get())
            linear_y = float(self.linear_y_entry.get())
            angular_z = float(self.angular_z_entry.get())
            self.publisher_node.set_twist(linear_x, linear_y, angular_z)
            self.publisher_node.publish_twist()
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numbers.")

    def spin_ros(self):
        rclpy.spin_once(self.publisher_node)
        self.master.after(100, self.spin_ros)  # Call this method again after 100 ms

def main(args=None):
    rclpy.init(args=args)
    publisher_node = TwistPublisher()

    # Load the controller manager
    #controller_manager = Node('controller_manager')

    # Load the joint state controller
    #controller_manager.declare_parameter('controller_manager', {'update_rate': 100.0})
    #controller_manager.get_logger().info("Loading joint state controller...")
    #controller_manager.get_parameter('controller_manager').set_parameter({'controllers': [{'type': 'joint_state_controller/JointStateController', 'joints': ['wheel_front_left_joint', 'wheel_front_right_joint', 'wheel_rear_left_joint', 'wheel_rear_right_joint']}]}).get_parameter('controllers')

    # Load the mecanum drive controller
    #controller_manager.get_logger().info("Loading mecanum drive controller...")
    #controller_manager.get_parameter('controller_manager').set_parameter({'controllers': [{'type': 'mecanum_controller/MecanumController', 'front_left_wheel': 'wheel_front_left_joint', 'front_right_wheel': 'wheel_front_right_joint', 'rear_left_wheel': 'wheel_rear_left_joint', 'rear_right_wheel': 'wheel_rear_right_joint', 'publish_rate': 50.0}]}).get_parameter('controllers')

    # Start the controllers
    #controller_manager.get_logger().info("Starting controllers...")
    #controller_manager.start_controller('joint_state_controller')
    #controller_manager.start_controller('mecanum_drive_controller')

    root = tk.Tk()
    gui = CarControlGUI(root, publisher_node)

    root.mainloop()  # Start the Tkinter main loop

    # Clean up after the GUI is closed
    publisher_node.destroy_node()
    #controller_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()