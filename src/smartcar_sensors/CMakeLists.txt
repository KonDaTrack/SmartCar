cmake_minimum_required(VERSION 3.8)
project(smartcar_sensors)

find_package(ament_cmake REQUIRED)

# TODO: add wrappers/launch/config for:
#   - LSLIDAR C16  (192.168.100.201, UDP 2368/2369)
#   - LakiBeam left/right (192.168.100.200/.210, UDP 2468/2568)
#   - VX1000 GNSS/IMU (/dev/ttyUSB0, 460800)

ament_package()
