import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import tkinter as tk
from threading import Thread

class STM32Interface(Node):
    def __init__(self):
        super().__init__('stm32_interface')
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Open UART connection
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("UART connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open UART: {e}")
            rclpy.shutdown()

        # Define joint names and initial states
        self.joint_state = JointState()
        self.joint_state.name = ['front_left_wheel', 'front_right_wheel', 'back_left_wheel', 'back_right_wheel']
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0]

        # Timer to read from UART
        self.create_timer(0.1, self.read_uart)

        # Start the GUI in a separate thread
        self.start_gui_thread()

    def start_gui_thread(self):
        gui_thread = Thread(target=self.start_gui)
        gui_thread.daemon = True  # Daemonize the thread
        gui_thread.start()

    def start_gui(self):
        # Create the Tkinter window
        self.root = tk.Tk()
        self.root.title("Mecanum Drive Control")

        # Input fields for linear x, y and angular z
        tk.Label(self.root, text="Linear X:").grid(row=0, column=0)
        self.linear_x_entry = tk.Entry(self.root)
        self.linear_x_entry.grid(row=0, column=1)

        tk.Label(self.root, text="Linear Y:").grid(row=1, column=0)
        self.linear_y_entry = tk.Entry(self.root)
        self.linear_y_entry.grid(row=1, column=1)

        tk.Label(self.root, text="Angular Z:").grid(row=2, column=0)
        self.angular_z_entry = tk.Entry(self.root)
        self.angular_z_entry.grid(row=2, column=1)

        # Button to send command
        send_button = tk.Button(self.root, text="Send Command", command=self.send_command)
        send_button.grid(row=3, columnspan=2)

        # Start the Tkinter main loop
        self.root.mainloop()

    def send_command(self):
        try:
            wheel_radius = 0.1  # Adjust to your robot's wheel radius
            wheel_base = 0.5     # Distance between front and rear wheels

            # Calculate wheel velocities for mecanum drive

            
            v_x = float(self.linear_x_entry.get())
            v_y = float(self.linear_y_entry.get())
            v_theta = float(self.angular_z_entry.get())
            front_left_velocity =  (v_x - v_y - (wheel_base * v_theta)) / wheel_radius
            front_right_velocity = (v_x + v_y + (wheel_base * v_theta)) / wheel_radius
            rear_left_velocity = (v_x + v_y - (wheel_base * v_theta)) / wheel_radius
            rear_right_velocity = (v_x - v_y + (wheel_base * v_theta)) / wheel_radius
            # Calculate wheel velocities
            v_FL = front_left_velocity  # Front Left
            v_FR = front_right_velocity # Front Right
            v_BL = rear_left_velocity  # Back Left
            v_BR = rear_right_velocity  # Back Right

            # Normalize wheel speeds if necessary
            max_speed = max(abs(v_FL), abs(v_FR), abs(v_BL), abs(v_BR))
            if max_speed > 1.0:  # Assuming the maximum speed is 1.0
                v_FL /= max_speed
                v_FR /= max_speed
                v_BL /= max_speed
                v_BR /= max_speed

            # Send the command to STM32
            command = f"cmd: FL:{v_FL} FR:{v_FR} BL:{v_BL} BR:{v_BR}\n"
            self.serial_port.write(command.encode('utf-8'))
            self.get_logger().info(f"Command sent: {command.strip()}")
        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")

    def read_uart(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                # Parse the line to extract positions and velocities
                data = line.split()
                pos1 = float(data[0].split(':')[1])
                vel1 = float(data[1].split(':')[1])
                pos2 = float(data[2].split(':')[1])
                vel2 = float(data[3].split(':')[1])
                pos3 = float(data[4].split(':')[1])
                vel3 = float(data[5].split(':')[1])
                pos4 = float(data[6].split(':')[1])
                vel4 = float(data[7].split(':')[1])

                # Update joint state
                self.joint_state.position = [pos1, pos2, pos3, pos4]
                self.joint_state.velocity = [vel1, vel2, vel3, vel4]

                # Publish joint states
                self.joint_state_pub.publish(self.joint_state)
        except Exception as e:
            self.get_logger().error(f"Error reading from UART: {e}")

    def destroy_node(self):
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("UART connection closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    stm32_interface = STM32Interface()
    try:
        rclpy.spin(stm32_interface)
    except KeyboardInterrupt:
        pass
   
    stm32_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()