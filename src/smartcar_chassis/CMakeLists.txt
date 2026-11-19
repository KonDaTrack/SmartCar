cmake_minimum_required(VERSION 3.8)
project(smartcar_chassis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smartcar_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(chassis_node src/chassis_node.cpp)
target_include_directories(chassis_node PRIVATE include)
ament_target_dependencies(chassis_node rclcpp smartcar_msgs)

add_executable(can_odometry_node src/can_odometry_node.cpp)
target_include_directories(can_odometry_node PRIVATE include)
ament_target_dependencies(
  can_odometry_node rclcpp smartcar_msgs nav_msgs geometry_msgs tf2 tf2_ros)

install(
  TARGETS chassis_node can_odometry_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
