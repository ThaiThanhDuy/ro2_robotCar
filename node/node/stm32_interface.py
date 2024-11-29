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

        # Timer to read from UART
        self.create_timer(0.1, self.read_uart)

        # Start the GUI in a separate thread
        self.command_in_progress = False  # State variable
        self.start_gui_thread()

    def start_gui_thread(self):
        gui_thread = Thread(target=self.start_gui)
        gui_thread.daemon = True  # Daemonize the thread
        gui_thread.start()

    def start_gui(self):
        # Create the Tkinter window
        self.root = tk.Tk()
        self.root.title("Mecanum Drive Control")

        # Dropdown to select COM port
        tk.Label(self.root, text="Select COM Port:").grid(row=0, column=0)
        self.com_ports = self.list_com_ports()
        self.selected_port = tk.StringVar(self.root)
        self.selected_port.set(self.com_ports[0])  # Default to first port

        self.port_menu = tk.OptionMenu(self.root, self.selected_port, *self.com_ports)
        self.port_menu.grid(row=0, column=1)

        # Button to open the selected port
        open_port_button = tk.Button(self.root, text="Open Port", command=self.open_port)
        open_port_button.grid(row=0, column=2)

        # Input fields for linear x, y and angular z
        tk.Label(self.root, text="Linear X:").grid(row=1, column=0)
        self.linear_x_entry = tk.Entry(self.root)
        self.linear_x_entry.grid(row=1, column=1)

        tk.Label(self.root, text="Linear Y:").grid(row=2, column=0)
        self.linear_y_entry = tk.Entry(self.root)
        self.linear_y_entry.grid(row=2, column=1)

        tk.Label(self.root, text="Angular Z:").grid(row=3, column=0)
        self.angular_z_entry = tk.Entry(self.root)
        self.angular_z_entry.grid(row=3, column=1)

        # Button to send command
        send_button = tk.Button(self.root, text="Send Command", command=self.send_command)
        send_button.grid(row=4, columnspan=3)

        # Button to close the GUI
        close_button = tk.Button(self.root, text="Close", command=self.close_gui)
        close_button.grid(row=5, columnspan=3)

        # Start the Tkinter main loop
        self.root.mainloop()

    def list_com_ports(self):
        """List available COM ports."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def open_port(self):
        """Open the selected COM port."""
        if self.serial_port is not None and self.serial_port.is_open:
            self.get_logger().warn("Port already open.")
            return

        selected = self.selected_port.get()
        try:
            self.serial_port = serial.Serial(selected, 115200, timeout=1)
            self.get_logger().info(f"Opened port: {selected}")
            messagebox.showinfo("Connection", f"Connected to {selected} successfully!")
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

            # Send the new command to STM32
            command = f"{front_left_velocity},{front_right_velocity},{rear_left_velocity},{rear_right_velocity}\n"  # Added newline
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Command sent: {command.strip()}")

        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")
        finally:
            self.command_in_progress = False  # Reset the state variable

    def read_uart(self):
        # Read data from UART and publish joint states
        if self.serial_port and self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f"Received data: {data}")  # Log the received data

            # Example expected data format: "pos1:1.0 vel1:0.5 pos2:-1.0 vel2:0.3 pos3:0.0 vel3:0.0 pos4:0.0 vel4:0.0"
          
            try:
                # Split the data by spaces
                parts = data.split()
                pos = []
                vel = []

                # Extract position and velocity values
                for part in parts:
                    if part.startswith('pos'):
                        pos_value = float(part.split(':')[1])
                        pos.append(pos_value)
                    elif part.startswith('vel'):
                        vel_value = float(part.split(':')[1])
                        vel.append(vel_value)

                # Update joint states if we have exactly 4 positions and 4 velocities
                if len(pos) == 4 and len(vel) == 4:
                    self.joint_state.position = pos
                    self.joint_state.velocity = vel
                    self.joint_state.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
                    self.joint_state_pub.publish(self.joint_state)
                    #self.get_logger().info(f"Joint states published: {self.joint_state}")
                else:
                    self.get_logger().error("Received data does not contain exactly 4 positions and 4 velocities.")

            except ValueError as e:
                self.get_logger().error(f"Error parsing data: {e}")

def main(args=None):
    rclpy.init(args=args)
    stm32_interface = STM32Interface()
    rclpy.spin(stm32_interface)
    stm32_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()