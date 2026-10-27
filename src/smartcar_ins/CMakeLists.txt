cmake_minimum_required(VERSION 3.8)
project(smartcar_ins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(smartcar_msgs REQUIRED)

add_executable(vx1000_node src/vx1000_node.cpp)
target_include_directories(vx1000_node PRIVATE include)
ament_target_dependencies(vx1000_node rclcpp sensor_msgs smartcar_msgs)

install(TARGETS vx1000_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
