import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 取得 Package 的路徑
    pkg_robot = get_package_share_directory('aruco_robot')
    pkg_realsense_ros = get_package_share_directory('realsense2_camera')

    # 2. 定義設定檔路徑
    config_path = os.path.join(pkg_robot, 'config', 'robot_param.yaml')
    realsense_path = os.path.join(pkg_realsense_ros, 'launch', 'rs_launch.py')

    # 3. 宣告啟動參數：透過 mode:=cal 或 mode:=robot 切換
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='robot',
    )

    # 4. RealSense Launch 檔
    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(realsense_path)
    #     # 如果需要傳參數給 RealSense (例如降低解析度或 FPS)，可以在這裡加
    #     # launch_arguments={'depth_module.profile': '640x480x30'}.items(),
    # )

    # 5. 包裝多個相機成一個group
    camera_names = ['cam_left', 'cam_mid', 'cam_right']
    robot_actions = []
    for cam in camera_names:
        robot_actions.append(
            Node(
                package='aruco_robot',
                executable='Aruco_detector_node',
                name=f'Aruco_detector_{cam}',
                parameters=[config_path, {'cam_name': cam}],
                output='screen'
            )
        )
        robot_actions.append(
            Node(
                package='aruco_robot',
                executable='Robot_detector_node',
                name=f'Robot_detector_{cam}',
                parameters=[config_path, {'cam_name': cam}],
                output='screen'
            )
        )
    robot_actions.append(
        Node(
            package='aruco_robot',
            executable='Robot_localizer_node',
            name='Robot_localizer_node',
            output='screen'
        )
    )

    # --- 模式一： 校正模式 (mode:=cal) ---
    calib_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'cal'),
        actions=[
            Node(
                package='aruco_robot',
                executable='Camera_calibrator_node',
                name='Camera_calibrator_node',
                output='screen'
            ),
        ]
    )

    # --- 模式二： Test1 模式 (mode:=test1) ---
    test1_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'test1'),
        actions=[
            Node(
                package='aruco_robot',
                executable='Aruco_detector_node',
                name='Aruco_detector_node',
                parameters=[config_path, {'cam_name': camera_cb}],
                output='screen'
            ),
            Node(
                package='aruco_robot',
                executable='Robot_detector_node',
                name='Robot_detector_node',
                parameters=[config_path, {'cam_name': camera_cb}],
                output='screen'
            ),
            Node(
                package='aruco_robot',
                executable='Robot_localizer_node',
                name='Robot_localizer_node',
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen'
            ),
        ]
    )

    # --- 模式三： Test2 模式 (mode:=test2) ---
    test2_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'test2'),
        actions=[
            Node(
                package='aruco_robot',
                executable='Relative_detector_node',
                name='Relative_detector_node',
                parameters=[config_path, {'cam_name': camera_cb}],
                output='screen'
            ),
            Node(
                package='aruco_robot',
                executable='Robot_localizer_node',
                name='Robot_localizer_node',
                output='screen'
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen'
            ),
        ]
    )

    # --- 模式四： Robot 模式 (mode:=robot) ---
    robot_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'robot'),
        actions=robot_actions
    )

    return LaunchDescription([
        mode_arg,
        # realsense_launch,
        calib_group,
        test1_group,
        test2_group,
        robot_group,
    ])