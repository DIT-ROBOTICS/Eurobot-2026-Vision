cmake_minimum_required(VERSION 3.8)
project(aruco_robot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ROS_DEPS
  rclcpp
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(robot_detector src/robot_detector.cpp)
add_executable(Aruco_detector_node src/Aruco_detector_node.cpp)
ament_target_dependencies(Aruco_detector_node ${ROS_DEPS})
target_link_libraries(Aruco_detector_node ${OpenCV_LIBS})

ament_target_dependencies(robot_detector
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
)

target_link_libraries(robot_detector
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  robot_detector
  Aruco_detector_node
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${ROS_DEPS})
ament_package()
