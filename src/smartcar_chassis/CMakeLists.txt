cmake_minimum_required(VERSION 3.8)
project(smartcar_chassis)

find_package(ament_cmake REQUIRED)

# TODO: implement the ROS 2 node for the chassis gateway:
#   gateway: 192.168.100.110
#   CAN1 UDP 4001, CAN2 UDP 4002, local UDP 8002
#   publish /vehicle_status (speed, steer, wheel speeds, total odometer)
#   subscribe to control messages (later)
# add_executable(...)
# ament_target_dependencies(...)

ament_package()
