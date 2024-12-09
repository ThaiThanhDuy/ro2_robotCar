import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import smbus
import math
import serial
from collections import deque
import threading

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.publish_joint_state)  # Publish every 0.05 seconds

        # Initialize I2C bus for MPU6050
        self.bus = smbus.SMBus(1)
        self.address = 0x68  # MPU6050 I2C address
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake up the MPU6050

        # Joint state initialization
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['left_wheel_joint_1', 'right_wheel_joint_1', 'left_wheel_joint_2', 'right_wheel_joint_2']
        self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]  # Initial positions
        self.joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0]  # Initial velocities

        # Initialize base_link position and orientation
        self.base_link_x = 0.0
        self.base_link_y = 0.0
        self.base_link_z = 0.0
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # Quaternion [qx, qy, qz, qw]

        # Initialize UART
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # Buffer for incoming data
        self.data_buffer = deque(maxlen=10)
        self.lock = threading.Lock()
        
        # Start a thread to read UART data
        self.reading_thread = threading.Thread(target=self.read_uart, daemon=True)
        self.reading_thread.start()

        # Previous joint positions for comparison
        self.prev_joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.prev_position = [0.0, 0.0, 0.0, 0.0]
        self.prev_velocity = [0.0, 0.0, 0.0, 0.0]
        self.alpha = 0.5  # Smoothing factor
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

    def read_imu_data(self):
        # Read accelerometer data
        accel_x = self.read_word_2c(0x3B)
        accel_y = self.read_word_2c(0x3D)
        accel_z = self.read_word_2c(0x3F)

        # Read gyroscope data
        gyro_x = self.read_word_2c(0x43)
        gyro_y = self.read_word_2c(0x45)
        gyro_z = self.read_word_2c(0x47)

        # Convert raw values to g and rad/s
        accel_x = accel_x / 16384.0  # Assuming the MPU6050 is set to ±2g
        accel_y = accel_y / 16384.0
        accel_z = accel_z / 16384.0

        gyro_x = gyro_x * (math.pi / 180)  # Convert degrees/sec to rad/sec
        gyro_y = gyro_y * (math.pi / 180)
        gyro_z = gyro_z * (math.pi / 180)

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    def read_word_2c(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    def publish_joint_state(self):
        # Check if there's data to process
        with self.lock:
            if self.data_buffer:
                data = self.data_buffer.popleft()
                self.process_uart_data(data)

        # Read IMU data
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_imu_data()

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'  # Frame ID for the IMU

        # Fill in the IMU message
        imu_msg.linear_acceleration.x = float(accel_x)
        imu_msg.linear_acceleration.y = float(accel_y)
        imu_msg.linear_acceleration.z = float(accel_z)
        imu_msg.angular_velocity.x = float(gyro_x)
        imu_msg.angular_velocity.y = float(gyro_y)
        imu_msg.angular_velocity.z = float(gyro_z)

        self.imu_publisher_.publish(imu_msg)

        # Calculate orientation from accelerometer data
        roll = math.atan2(accel_y, accel_z)
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        yaw = 0.0  # You can set this to a fixed value or integrate gyroscope data for yaw

        # Convert to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        # Update orientation
        self.orientation = [qx, qy, qz, qw]

        # Update base_link position and orientation only if there's a change
        if self.joint_state_msg.position != self.prev_joint_positions:
            # Update previous joint positions after the update
            self.prev_joint_positions = list(self.joint_state_msg.position)  # Convert to list

            # Update base_link position
            self.update_base_link_position(self.linear_x, self.linear_y, self.angular_z)

        # Create and publish the transform
        self.publish_transform()

        # Publish the joint state message
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state_msg)

        self.get_logger().info(f'Publishing: Position: {self.joint_state_msg.position}, Velocity: {self.joint_state_msg.velocity}, Base Link Position: ({self.base_link_x}, {self.base_link_y}, {self.base_link_z}), Orientation: {self.orientation}')

    def read_uart(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received data: {data}")  # Log the received data

                with self.lock:
                    self.data_buffer.append(data)

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

                # Calculate linear and angular velocities
                self.linear_x = (self.joint_state_msg.velocity[0] + self.joint_state_msg.velocity[1] +
                                 self.joint_state_msg.velocity[2] + self.joint_state_msg.velocity[3]) / (4 * self.wheel_radius)
                self.linear_y = (-self.joint_state_msg.velocity[0] + self.joint_state_msg.velocity[1] +
                                 self.joint_state_msg.velocity[2] - self.joint_state_msg.velocity[3]) / (4 * self.wheel_radius)
                self.angular_z = (-self.joint_state_msg.velocity[0] + self.joint_state_msg.velocity[1] -
                                  self.joint_state_msg.velocity[2] + self.joint_state_msg.velocity[3]) / (4 * self.wheel_base)

            else:
                self.get_logger().error("Received data does not contain exactly 4 positions and 4 velocities.")

        except ValueError as e:
            self.get_logger().error(f"Error parsing data: {e}")

    def update_base_link_position(self, linear_x, linear_y, angular_z):
        # Update base_link position based on the desired linear and angular velocities
        dt = 0.001  # Time step

        # Convert current quaternion to yaw
        current_yaw = self.quaternion_to_yaw(self.orientation)

        # Update yaw based on angular velocity
        current_yaw += angular_z * dt

        # Convert back to quaternion
        self.orientation = self.yaw_to_quaternion(current_yaw)

        # Update position based on the updated orientation
        self.base_link_x += (linear_x * math.cos(current_yaw) - linear_y * math.sin(current_yaw)) * dt
        self.base_link_y += (linear_x * math.sin(current_yaw) + linear_y * math.cos(current_yaw)) * dt
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
        
        # Set rotation based on the IMU orientation
        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]
        
        # Send the transform
        self.transform_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def quaternion_to_yaw(self, orientation):
        # Convert quaternion to yaw angle
        qx, qy, qz, qw = orientation
        # Yaw (Z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        # Convert yaw angle to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]  # Assuming roll and pitch are zero

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
