# smartcar_chassis

Placeholder package for the `.110` CAN-to-Ethernet chassis gateway.

Reference from the legacy ROS 1 stack:

- Gateway IP: `192.168.100.110`
- CAN1 remote UDP port: `4001`
- CAN2 remote UDP port: `4002`
- Local UDP port: `8002`
- UDP payload is a 13-byte raw CAN frame: `[dlc][id:4][data:8]` for TX;
  the gateway returns raw CAN frames decoded by the legacy `can_module_usb2can`.

Planned ROS 2 interface:

- Publish `/vehicle_status` (`smartcar_msgs/VehicleStatus`).
- Optionally publish wheel odometry (`nav_msgs/Odometry`) if SLAM needs it.
- Subscribe to `/ecu` / control commands later.

The chassis is currently started inside the `sensors` container; it can be
split into its own `chassis` service later without changing the image.
