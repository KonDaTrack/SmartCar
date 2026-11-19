cmake_minimum_required(VERSION 3.8)
project(smartcar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleStatus.msg"
  "msg/Vx1000Ins.msg"
  "msg/Vx1000Imu.msg"
  "msg/Vx1000Gnss.msg"
  "msg/Vx1000Stat.msg"
  "msg/Vx1000Cov.msg"
  "msg/Vx1000Odom.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
