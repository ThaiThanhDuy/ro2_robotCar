import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import tkinter as tk
from tkinter import messagebox
from threading import Thread
import serial.tools.list_ports  # Import for listing COM ports

class STM32Interface(Node):
    def __init__(self):
        super().__init__('stm32_interface')
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Initialize serial port
        self.serial_port = None

        # Define joint names and initial states
        self.joint_state = JointState()
        self.joint_state.name = ['front_left_wheel', 'front_right_wheel', 'back_left_wheel', 'back_right_wheel']
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0]

        # Start the GUI in a separate thread
        self.command_in_progress = False  # State variable
        self.start_gui_thread()

        # Automatically open the serial port
        self.open_port('/dev/ttyUSB0')

        # Schedule the command sending
        self.send_command_periodically()

    def start_gui_thread(self):
        gui_thread = Thread(target=self.start_gui)
        gui_thread.daemon = True  # Daemonize the thread
        gui_thread.start()

    def start_gui(self):
        # Create the Tkinter window
        self.root = tk.Tk()
        self.root.title("Mecanum Drive Control")

        # Input fields for linear x, y and angular z
        tk.Label(self.root, text="Linear X:").grid(row=1, column=0)
        self.linear_x_entry = tk.Entry(self.root)
        self.linear_x_entry.grid(row=1, column=1)
        self.linear_x_entry.bind("<Return>", self.update_command)  # Bind Enter key

        tk.Label(self.root, text="Linear Y:").grid(row=2, column=0)
        self.linear_y_entry = tk.Entry(self.root)
        self.linear_y_entry.grid(row=2, column=1)
        self.linear_y_entry.bind("<Return>", self.update_command)  # Bind Enter key

        tk.Label(self.root, text="Angular Z:").grid(row=3, column=0)
        self.angular_z_entry = tk.Entry(self.root)
        self.angular_z_entry.grid(row=3, column=1)
        self.angular_z_entry.bind("<Return>", self.update_command)  # Bind Enter key

        # Button to close the GUI
        close_button = tk.Button(self.root, text="Close", command=self.close_gui)
        close_button.grid(row=5, columnspan=3)

        # Start the Tkinter main loop
        self.root.mainloop()

    def open_port(self, port_name):
        """Open the specified COM port."""
        if self.serial_port is not None and self.serial_port.is_open:
            self.get_logger().warn("Port already open.")
            return

        try:
            self.serial_port = serial.Serial(port_name, 115200, timeout=1)
            self.get_logger().info(f"Opened port: {port_name}")
            messagebox.showinfo("Connection", f"Connected to {port_name} successfully!")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open port: {e}")
            messagebox.showerror("Connection Error", f"Failed to open port: {e}")

    def close_gui(self):
        """Close the GUI and perform cleanup."""
        self.get_logger().info("Closing GUI...")
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()  # Close the serial port
        self.root.quit()  # Exit the Tkinter main loop
        rclpy.shutdown()  # Shutdown ROS 2

    def send_command_periodically(self):
        """Send command periodically."""
        self.send_command()
        self.root.after(100, self.send_command_periodically)  # Schedule the next command sending

    def update_command(self, event):
        """Update the command when the user enters new values."""
        self.send_command()

    def send_command(self):
        if self.command_in_progress:
            self.get_logger().warn("Previous command still in progress. Please wait.")
            return

        # Check if the serial port is open
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn("Cannot send command: Port is not open.")
            messagebox.showwarning("Warning", "Cannot send command: Port is not open. Please open a port first.")
            return

        self.command_in_progress = True
        try:
            # Clear previous command by sending a stop command (if necessary)

            # Now send the new command
            wheel_radius = 0.1  # Adjust to your robot's wheel radius
            wheel_base = 0.5     # Distance between front and rear wheels

            v_x = float(self.linear_x_entry.get())
            v_y = float(self.linear_y_entry.get())
            v_theta = float(self.angular_z_entry.get())
            front_left_velocity =  (v_x - v_y - (wheel_base * v_theta)) / wheel_radius
            front_right_velocity = (v_x + v_y + (wheel_base * v_theta)) / wheel_radius
            rear_left_velocity = (v_x + v_y - (wheel_base * v_theta)) / wheel_radius
            rear_right_velocity = (v_x - v_y + (wheel_base * v_theta)) / wheel_radius

            # Normalize wheel speeds if necessary
            max_speed = max(abs(front_left_velocity), abs(front_right_velocity), abs(rear_left_velocity), abs(rear_right_velocity))
            if max_speed > 1.0:  # Assuming the maximum speed is 1.0
                front_left_velocity /= max_speed
                front_right_velocity /= max_speed
                rear_left_velocity /= max_speed
                rear_right_velocity /= max_speed

            # Round velocities to two decimal places
            front_left_velocity = round(front_left_velocity, 2)
            front_right_velocity = round(front_right_velocity, 2)
            rear_left_velocity = round(rear_left_velocity, 2)
            rear_right_velocity = round(rear_right_velocity, 2)
            # Send the new command to STM32
            command = f"c:{front_left_velocity},{front_right_velocity},{rear_left_velocity},{rear_right_velocity}\n"  # Added newline
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Command sent: {command.strip()}")

        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")
        finally:
            self.command_in_progress = False  # Reset the state variable

def main(args=None):
    rclpy.init(args=args)
    stm32_interface = STM32Interface()
    rclpy.spin(stm32_interface)
    stm32_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()