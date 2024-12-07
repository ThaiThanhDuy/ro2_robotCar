import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        
       

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['src/main/launch/launch.py']),
        ),
      
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['src/robot_model/launch/rsp.launch.py']),
        ),
     
    ])