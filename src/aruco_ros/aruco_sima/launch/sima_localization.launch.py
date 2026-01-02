from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    nodes_list = []
    sima_ids = [1, 5, 7, 12, 14]
    for sima_id in sima_ids:
        
        sima_params = {
            'marker_length': LaunchConfiguration('marker_length'),
            'sima_id': sima_id, 
        }

        node = Node(
            package='aruco_sima',
            executable='sima_detector',
            name=f'sima_detector_{sima_id}',
            parameters=[sima_params],
            remappings=[('/sima_pose', f'/sima_{sima_id}/global/pose')]
        )
        
        nodes_list.append(node)

    return nodes_list


def generate_launch_description():

    marker_length_arg = DeclareLaunchArgument(
        'marker_length', default_value='0.08',
        description='Marker size in m. '
    )

    ld = LaunchDescription()

    ld.add_action(marker_length_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld