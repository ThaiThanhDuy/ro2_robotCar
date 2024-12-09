from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch the Navigation 2 bringup with the specified map
        #Node(
           # package='nav2_bringup',
           # executable='bringup_launch.py',
           # name='nav2_bringup',
           # parameters=[{
           #     'use_sim_time': True,
          #      'map': '/home/duy/robot/map_1733563821.yaml'  # Specify the map path directly
         #   }],
        #    remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')]
       # ),
        # commnet cd $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch
        # ros2 launch nav2_bringup bringup_launch.py map:=/home/duy/robot/map_1733563821.yaml

           Node(
            package='node',  # Replace with your package name
            executable='motor_controller',  # Replace with your node executable name
            name='motor_controller',
            output='screen',
            parameters=[],
            remappings=[],
        ),

        # Launch the AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=['/home/duy/robot/src/main/config/amcl.yaml',],
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static','/scan', '/lidar_link')]
        ),
       #  Node(
        #    package='nav2_bringup',
        #    executable='nav2_bringup',
         #   name='nav2_bringup',
         #   output='screen',
          #  parameters=[
           #     'path/to/costmap.yaml',
            #    'path/to/local_planner.yaml',
             #   'path/to/nav2_params.yaml'
            #],
            #remappings=[
             #   ('/scan', '/your_lidar_topic','/ultrasonic_distance', '/ultrasonic_distance')  # Change this to your LIDAR topic
            #]
        #),
        Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    output='screen',
    parameters=['src/main/config/nav2_params.yaml'],
)
    ])
