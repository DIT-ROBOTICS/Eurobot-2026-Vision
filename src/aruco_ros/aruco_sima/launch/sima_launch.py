import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 取得 Package 的路徑
    pkg_sima = get_package_share_directory('aruco_sima')
    pkg_realsense_ros = get_package_share_directory('realsense2_camera')

    # 2. 定義設定檔路徑
    config_path = os.path.join(pkg_sima, 'config', 'sima_param.yaml')
    realsense_path = os.path.join(pkg_realsense_ros, 'launch', 'rs_launch.py')

    # 3. 宣告啟動參數：透過 mode:=cal 或 mode:=robot 或 mode:=sima 切換
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sima',
    )

    # 4. RealSense Launch 檔
    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(realsense_path)
    #     # 如果需要傳參數給 RealSense (例如降低解析度或 FPS)，可以在這裡加
    #     # launch_arguments={'depth_module.profile': '640x480x30'}.items(),
    # )

    # 定義你要追蹤的 SIMA ID 清單
    sima_ids = [1, 5, 7, 12, 14]
    # 動態生成 SIMA 偵測節點清單
    sima_nodes = []
    for sid in sima_ids:
        sima_nodes.append(
            Node(
                package='aruco_sima',
                executable='Sima_detector_node',
                name=f'sima_detector_{sid}', 
                parameters=[config_path, {'sima_id': sid}],
                output='screen'
            )
        )

    # --- 模式一：校正模式 (mode:=cal) ---
    calib_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'cal'),
        actions=[
            Node(
                package='aruco_sima',
                executable='Camera_calibrator_node',
                name='Camera_calibrator_node',
                output='screen'
            ),
        ]
    )

    # --- 模式二：Test模式 (mode:test) ---
    test_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'test'),
        actions=[
            Node(
                package='aruco_sima',
                executable='Aruco_detector_node',
                name='Aruco_detector_node',
                parameters=[config_path],
                output='screen'
            ),
            Node(
                package='aruco_sima',
                executable='Sima_detector_node',
                name='Sima_detector_node',
                parameters=[config_path],
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

    # --- 模式三：Sima模式 (mode:=sima) ---
    sima_group = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'sima'),
        actions=[
            Node(
                package='aruco_sima',
                executable='Aruco_detector_node',
                name='Aruco_detector_node',
                parameters=[config_path],
                output='screen'
            ),
            *sima_nodes,
        ]
    )

    return LaunchDescription([
        mode_arg,
        # realsense_launch,
        calib_group,
        test_group,
        sima_group
    ])