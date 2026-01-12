from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('yolo'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='yolo',
            executable='yolo_node.py',
            name='yolo_node',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])