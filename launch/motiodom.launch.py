import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
     package_dir = get_package_share_directory('motiodom')
     return launch.LaunchDescription([
          Node(package='motiodom', executable='motiodom', name='motiodom', parameters=[os.path.join(package_dir, 'config', 'params.yaml')], output='screen')
     ])