cmake_minimum_required(VERSION 3.8)
project(Robot_localization)

# 1. 尋找所有需要的 Package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. 定義共通的 ROS 依賴項變數，避免重複打字
set(ROS_DEPS
  rclcpp
  sensor_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# 3. 設定標頭檔路徑
include_directories(include)

# 4. 定義所有 Executables
add_executable(Camera_calibrator_node src/Camera_calibrator_node.cpp)
ament_target_dependencies(Camera_calibrator_node ${ROS_DEPS})
target_link_libraries(Camera_calibrator_node ${OpenCV_LIBRARIES})

add_executable(Robot_detector_node src/Robot_detector_node.cpp)
ament_target_dependencies(Robot_detector_node ${ROS_DEPS})
target_link_libraries(Robot_detector_node ${OpenCV_LIBRARIES})

add_executable(Robot_localizer_node src/Robot_localizer_node.cpp)
ament_target_dependencies(Robot_localizer_node ${ROS_DEPS})
target_link_libraries(Robot_localizer_node ${OpenCV_LIBRARIES})

add_executable(aruco_detector_node src/aruco_detector_node.cpp)
ament_target_dependencies(aruco_detector_node ${ROS_DEPS})
target_link_libraries(aruco_detector_node ${OpenCV_LIBRARIES})

add_executable(robot_detector_node src/robot_detector_node.cpp)
ament_target_dependencies(robot_detector_node ${ROS_DEPS})
target_link_libraries(robot_detector_node ${OpenCV_LIBRARIES})

# 5. 安裝與打包
install(TARGETS
  Camera_calibrator_node
  Robot_detector_node
  Robot_localizer_node
  aruco_detector_node
  robot_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${ROS_DEPS})
ament_package()