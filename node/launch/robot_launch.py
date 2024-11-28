from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='controller_manager',
            name='controller_manager',
            output='screen',
            parameters=[{'use_sim_time': True}, 'config/controller_config.yaml'],
            remappings=[
                # Add any necessary topic remappings here
            ]
        ),
        # Add other nodes related to your robot here
    ])