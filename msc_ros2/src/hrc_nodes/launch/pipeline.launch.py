from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    default=os.path.join(get_package_share_directory('hrc_nodes'),'config','pipeline.yaml')
    return LaunchDescription([DeclareLaunchArgument('config',default_value=default)]+[
        Node(package='hrc_nodes',executable=n,output='screen',parameters=[LaunchConfiguration('config')])
        for n in ['stvl53l7cx_driver','human_detection','sensor_health','speed_scaling','decision_making','led_lighting']])
