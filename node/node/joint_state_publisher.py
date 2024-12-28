import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import serial  # Make sure to install pyserial if you haven't already
from collections import deque
import threading

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.transform_broadcaster = TransformBroadcaster(self)

        # Joint state initialization
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['left_wheel_joint_1', 'right_wheel_joint_1', 'left_wheel_joint_2', 'right_wheel_joint_2']
        self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]  # Initial positions
        self.joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0]  # Initial velocities

        self.timer = self.create_timer(0.05, self.publish_joint_state)  # Publish every 0.05 seconds

        # Initialize base_link position and orientation
        self.base_link_x = 0.0
        self.base_link_y = 0.0
        self.base_link_z = 0.0
        self.orientation = 0.0  # Orientation in radians
        
        # Initialize linear and angular velocities
        self.linear_x = 0.0
        self.linear_y = 0.0 
        self.angular_z = 0.0

        # Robot parameters
        self.wheel_base = 0.38  # Distance between left and right wheels (in meters)
        self.track_width = 0.3  # Distance between front and rear wheels (in meters)
        self.wheel_radius = 0.0485  # Radius of the wheels (in meters)

        # Initialize UART
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Adjust the port and baud rate as needed
        self.serial_yaw = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)      # For yaw
        # Buffer for incoming data
        self.data_buffer = deque(maxlen=10)  # Adjust the size as needed
        self.yaw_buffer = deque(maxlen=10)    # For yaw
        self.lock = threading.Lock()
        
        # Start a thread to read UART data
        self.reading_thread = threading.Thread(target=self.read_uart, daemon=True)
        self.reading_thread.start()

        # Smoothing parameters
        self.alpha = 0.5  # Smoothing factor (0 < alpha < 1)
        self.prev_position = [0.0, 0.0, 0.0, 0.0]
        self.prev_velocity = [0.0, 0.0, 0.0, 0.0]
        # Previous joint positions for comparison
        self.prev_joint_positions = [0.0, 0.0, 0.0, 0.0]

    def publish_joint_state(self):
        # Check if there's data to process
        with self.lock:
            if self.data_buffer:
                data = self.data_buffer.popleft()
                self.process_uart_data(data)
            if self.yaw_buffer:
                yaw_data = self.yaw_buffer.popleft()
                self.process_yaw_data(yaw_data)

        # Publish the joint state message
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state_msg)
        
        # Update base_link position and orientation only if there's a change
        if self.joint_state_msg.position != self.prev_joint_positions:
            # Update previous joint positions after the update
            self.prev_joint_positions = list(self.joint_state_msg.position)  # Convert to list

            # Update base_link position and orientation
            self.update_base_link_position(self.linear_x, self.linear_y, self.angular_z)

        # Create and publish the transform
        self.publish_transform()

        self.get_logger().info(f'Publishing: Position: {self.joint_state_msg.position}, Velocity: {self.joint_state_msg.velocity}, Base Link Position: ({self.base_link_x}, {self.base_link_y}, {self.base_link_z}), Orientation: {self.orientation}')
      #  self.get_logger().info(f 'Publishing: XYZ: {self.linear_x}, {self.linear_y}, {self.angular_z}')

    def read_uart(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received data: {data}")  # Log the received data
                if self.serial_yaw.in_waiting > 0:
                    yaw_data = self.serial_yaw.readline().decode('utf-8').strip()
                    self.get_logger().info(f"Received yaw data: {yaw_data}")  # Log the received yaw data

                    with self.lock:
                        self.data_buffer.append(data)
                        self.yaw_buffer.append(yaw_data)

    def process_yaw_data(self, yaw_data):
        try:
            yaw = float(yaw_data.split(':')[1])  # Assuming the format is 'yaw:<value>'
            yaw_radians = (yaw % 360) * (math.pi / 180)

            # Update orientation
            self.orientation = yaw_radians

        except ValueError as e:
            self.get_logger().error(f"Error parsing yaw data: {e}")

    def process_uart_data(self, data):
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
                # Apply exponential smoothing
                self.joint_state_msg.position = [
                    round(self.alpha * p + (1 - self.alpha) * pp, 2) for p, pp in zip(pos, self.prev_position)
                ]
                self.joint_state_msg.velocity = [
                    round(self.alpha * v + (1 - self.alpha) * pv, 2) for v, pv in zip(vel, self.prev_velocity)
                ]
                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp

                # Update previous values for the next iteration
                self.prev_position = self.joint_state_msg.position
                self.prev_velocity = self.joint_state_msg.velocity
                if(self.joint_state_msg.velocity[0]>=0 and self.joint_state_msg.velocity[0] <=0.02):
                   self.joint_state_msg.velocity[0]=0.0
                if(self.joint_state_msg.velocity[1]>=0 and self.joint_state_msg.velocity[1] <=0.02):
                   self.joint_state_msg.velocity[1]=0.0
                if(self.joint_state_msg.velocity[2]>=0 and self.joint_state_msg.velocity[2] <=0.02):
                   self.joint_state_msg.velocity[2]=0.0
                if(self.joint_state_msg.velocity[3]>=0 and self.joint_state_msg.velocity[3] <=0.02):
                   self.joint_state_msg.velocity[3]=0.0

                if(self.joint_state_msg.velocity[0]<=0 and self.joint_state_msg.velocity[0] >= -0.02):
                   self.joint_state_msg.velocity[0]=0.0
                if(self.joint_state_msg.velocity[1]<=0 and self.joint_state_msg.velocity[1] >=-0.02):
                   self.joint_state_msg.velocity[1]=0.0
                if(self.joint_state_msg.velocity[2]<=0 and self.joint_state_msg.velocity[2] >=-0.02):
                   self.joint_state_msg.velocity[2]=0.0
                if(self.joint_state_msg.velocity[3]<=0 and self.joint_state_msg.velocity[3] >=-0.02):
                   self.joint_state_msg.velocity[3]=0.0
                # Calculate linear and angular velocities
                self.linear_x = (self.joint_state_msg.velocity[0] + self.joint_state_msg.velocity[1] +
                                 self.joint_state_msg.velocity[2] + self.joint_state_msg.velocity[3]) / (4 * self.wheel_radius)
                self.linear_y = (-self.joint_state_msg.velocity[0] + self.joint_state_msg.velocity[1] +
                                 self.joint_state_msg.velocity[2] - self.joint_state_msg.velocity[3]) / (4 * self.wheel_radius)
                self.angular_z = (-self.joint_state_msg.velocity[0] + self.joint_state_msg.velocity[1] -
                                  self.joint_state_msg.velocity[2] + self.joint_state_msg.velocity[3]) / (4 * 0.1475)
                self.get_logger().info(f'Publishing: XYZ: {self.linear_x}, {self.linear_y}, {self.angular_z}')

            else:
                self.get_logger().error("Received data does not contain exactly 4 positions and 4 velocities.")

        except ValueError as e:
            self.get_logger().error(f"Error parsing data: {e}")

    def update_base_link_position(self, linear_x, linear_y, angular_z):
        # Update base_link position based on the desired linear and angular velocities
        dt = 0.001  # Time step
        
        self.base_link_x += (linear_x * math.cos(self.orientation) - linear_y * math.sin(self.orientation)) * dt
        self.base_link_y += (linear_x * math.sin(self.orientation) + linear_y * math.cos(self.orientation)) * dt
        # Assuming constant height for simplicity
        self.base_link_z = 0.0
      
    def publish_transform(self):
        # Create a TransformStamped message for odom to base_link
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
