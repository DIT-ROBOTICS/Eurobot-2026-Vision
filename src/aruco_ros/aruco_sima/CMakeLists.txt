cmake_minimum_required(VERSION 3.8)
project(aruco_sima)

# 1. 尋找所有需要的 Package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 2. 定義共通的 ROS 依賴項變數
set(ROS_DEPS
  rclcpp
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# 3. 設定標頭檔路徑
include_directories(
  ${apriltag_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 4. 定義所有 Executables
add_executable(Aruco_detector_node src/Aruco_detector_node.cpp)
ament_target_dependencies(Aruco_detector_node ${ROS_DEPS})
target_link_libraries(Aruco_detector_node ${OpenCV_LIBS})

add_executable(Camera_calibrator_node src/Camera_calibrator_node.cpp)
ament_target_dependencies(Camera_calibrator_node ${ROS_DEPS})
target_link_libraries(Camera_calibrator_node ${OpenCV_LIBS})

add_executable(Sima_detector_node src/Sima_detector_node.cpp)
ament_target_dependencies(Sima_detector_node ${ROS_DEPS})
target_link_libraries(Sima_detector_node apriltag ${OpenCV_LIBS})

# 5. 安裝與打包
install(TARGETS
  Aruco_detector_node
  Camera_calibrator_node
  Sima_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${ROS_DEPS})
ament_package()