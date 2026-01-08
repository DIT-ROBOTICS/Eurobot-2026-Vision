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
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_path)
        # 如果需要傳參數給 RealSense (例如降低解析度或 FPS)，可以在這裡加
        # launch_arguments={'depth_module.profile': '640x480x30'}.items(),
    )

    # --- 模式一：校正模式 (mode:=cal) ---
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

    # --- 模式二： Test 模式 (mode:=test) ---
    test_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'test'),
        actions=[
            Node(
                package='aruco_robot',
                executable='Aruco_detector_node',
                name='Aruco_detector_node',
                output='screen'
            ),
            Node(
                package='aruco_robot',
                executable='Robot_detector_node',
                name='Robot_detector_node',
                parameters=[config_path],
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
                # 如果你有預設的 .rviz 設定檔，可以取消下面這行的註解
                # arguments=['-d', os.path.join(pkg_aruco_ros, 'rviz', 'aruco_config.rviz')]
            ),
        ]
    )

    # --- 模式三： Robot 模式 (mode:=robot) ---
    robot_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'robot'),
        actions=[
            Node(
                package='aruco_robot',
                executable='Aruco_detector_node',
                name='Aruco_detector_node',
                output='screen'
            ),
            Node(
                package='aruco_robot',
                executable='Robot_detector_node',
                name='Robot_detector_node',
                parameters=[config_path],
                output='screen'
            ),
            Node(
                package='aruco_robot',
                executable='Robot_localizer_node',
                name='Robot_localizer_node',
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        mode_arg,
        realsense_launch,
        calib_group,
        test_group,
        robot_group,
    ])