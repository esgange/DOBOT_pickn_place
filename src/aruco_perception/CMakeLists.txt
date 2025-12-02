cmake_minimum_required(VERSION 3.8)
project(aruco_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)

add_executable(aruco_detector_node src/aruco_detector_node.cpp)
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  ament_index_cpp
  OpenCV
  cv_bridge
)
target_link_libraries(aruco_detector_node ${OpenCV_LIBS})

add_executable(perception_calibration src/perception_calibration.cpp)
ament_target_dependencies(perception_calibration
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)
target_link_libraries(perception_calibration yaml-cpp)

install(TARGETS aruco_detector_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS perception_calibration
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
