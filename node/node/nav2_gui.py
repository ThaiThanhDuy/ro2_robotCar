import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, QLineEdit, QPushButton, QGraphicsView, QGraphicsScene
from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter

class Nav2GoalSetter(Node):
    def __init__(self):
        super().__init__('nav2_goal_setter')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info("Nav2 Goal Setter Node Initialized")

        # Subscriber to get the current position of the robot
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_position = (0.0, 0.0)  # Initialize current position

    def set_goal(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = "map"  # Set the frame to map
        msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0  # No rotation
        self.publisher.publish(msg)
        self.get_logger().info(f"Goal set to: x={x}, y={y}")

    def odom_callback(self, msg):
        # Update the current position from the odometry message
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Nav2 Goal Setter")
        self.setGeometry(100, 100, 400, 300)

        self.node = Nav2GoalSetter()

        # Create a central widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # Create layout
        layout = QVBoxLayout(central_widget)

        # Create input fields for X and Y
        self.x_input = QLineEdit(self)
        self.x_input.setPlaceholderText("Enter X position")
        layout.addWidget(self.x_input)

        self.y_input = QLineEdit(self)
        self.y_input.setPlaceholderText("Enter Y position")
        layout.addWidget(self.y_input)

        # Create a button to set the goal
        self.set_goal_button = QPushButton("Set Goal", self)
        self.set_goal_button.clicked.connect(self.set_goal)
        layout.addWidget(self.set_goal_button)

        # Create a label to display the current position
        self.current_position_label = QLabel("Current Position: (0.0, 0.0)", self)
        layout.addWidget(self.current_position_label)

        # Create a timer to update the current position label
        self.timer = self.node.create_timer(0.5, self.update_current_position)

    def set_goal(self):
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            self.node.set_goal(x, y)
        except ValueError:
            self.node.get_logger().warn("Invalid input. Please enter valid numbers for X and Y.")

    def update_current_position(self):
        x, y = self.node.current_position
        self.current_position_label.setText(f"Current Position: ({x:.2f}, {y:.2f})")

def main():
    rclpy.init()
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()
    rclpy.shutdown()

if __name__ == '__main__':
    main()