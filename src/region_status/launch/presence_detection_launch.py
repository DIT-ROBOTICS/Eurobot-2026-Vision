import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():

    package_name = 'region_status'
    
    config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'presence_detection_params.yaml'
    )

    presence_node = Node(
        package=package_name,
        executable='presence_detection', 
        name='presence_detection_node',
        output='screen',
        parameters=[config_path] 
    )

    return LaunchDescription([
        presence_node
    ])