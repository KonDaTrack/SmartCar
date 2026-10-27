cmake_minimum_required(VERSION 3.8)
project(smartcar_slam)

find_package(ament_cmake REQUIRED)

# TODO: choose and wire the SLAM stack:
#   FAST-LIO2 / Point-LIO / NDT localization / other.
#   Subscribe to /lslidar_point_cloud, /VX1000_imu, /tf, /vehicle_status.
#   Keep algorithm parameters in config/vehicle.yaml / config/dev.yaml.

ament_package()
