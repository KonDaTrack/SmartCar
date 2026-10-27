// CAN-over-UDP protocol used by the 192.168.100.110 chassis gateway.
//
// Wire format (13 bytes):
//   [0]    DLC
//   [1..4] CAN ID, big endian
//   [5..12] 8 data bytes, zero padded
//
// Ported from the vehicle's ROS 1 can_module_usb2can driver.
#ifndef SMARTCAR_CHASSIS_CAN_PROTOCOL_HPP
#define SMARTCAR_CHASSIS_CAN_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace smartcar_chassis
{

constexpr size_t kCanUdpFrameLen = 13;

constexpr uint32_t kCanIdVehicleStatus = 0x51;
constexpr uint32_t kCanIdWheelLeft = 0x178;
constexpr uint32_t kCanIdWheelRight = 0x188;
constexpr uint32_t kCanIdBatteryLegacy = 0x2AA;
constexpr uint32_t kCanIdBattery = 0x17904001;

struct CanFrame
{
  uint32_t id{0};
  uint8_t dlc{0};
  uint8_t data[8]{};
};

inline bool parseCanUdp(const uint8_t * buf, size_t len, CanFrame & frame)
{
  if (len < kCanUdpFrameLen) {
    return false;
  }
  frame.dlc = static_cast<uint8_t>(buf[0] & 0x0f);
  frame.id = (static_cast<uint32_t>(buf[1]) << 24) |
             (static_cast<uint32_t>(buf[2]) << 16) |
             (static_cast<uint32_t>(buf[3]) << 8) |
             static_cast<uint32_t>(buf[4]);
  std::memcpy(frame.data, buf + 5, sizeof(frame.data));
  return true;
}

inline int readBit(const uint8_t * data, int pos)
{
  return (data[pos >> 3] >> (pos & 0x07)) & 1;
}

// LSB-first bit field reader, matching the vendor's readAsInt().
inline uint32_t readAsUInt(const uint8_t * data, int offset, int length)
{
  uint32_t value = 0;
  for (int i = length - 1; i >= 0; --i) {
    value = (value << 1) | static_cast<uint32_t>(readBit(data, offset + i));
  }
  return value;
}

inline int16_t readAsInt16(const uint8_t * data, int offset)
{
  return static_cast<int16_t>(readAsUInt(data, offset, 16));
}

// 0x51 vehicle status, offsets from the ROS 1 MsgBase.h.
struct VehicleStatusData
{
  uint8_t shift_level{0};       // 0 unknown, 1 D, 2 N, 3 R
  bool is_brake{false};
  uint8_t ignition_status{0};
  bool wheel_direction_right{false};
  double cur_steer{0.0};        // front wheel angle [deg]
  double cur_speed{0.0};        // [m/s]
  uint8_t drive_mode{0};
  uint8_t acc_level{0};
  uint8_t brake_level{0};
  double total_odometer{0.0};   // [km]
};

inline VehicleStatusData decodeVehicleStatus(
  const CanFrame & frame, double steer_raw_scale, double steer_scale)
{
  const uint8_t * d = frame.data;

  VehicleStatusData out;
  out.shift_level = static_cast<uint8_t>(readAsUInt(d, 0, 2));
  out.is_brake = readBit(d, 2) != 0;
  out.ignition_status = static_cast<uint8_t>(readAsUInt(d, 3, 2));
  out.wheel_direction_right = readBit(d, 7) != 0;

  const double raw_angle = readAsUInt(d, 8, 12) / steer_raw_scale;
  out.cur_steer = (out.wheel_direction_right ? raw_angle : -raw_angle) * steer_scale;

  out.cur_speed = readAsUInt(d, 20, 9) / 10.0;
  out.drive_mode = static_cast<uint8_t>(readAsUInt(d, 29, 3));
  out.acc_level = static_cast<uint8_t>(readAsUInt(d, 32, 2));
  out.brake_level = static_cast<uint8_t>(readAsUInt(d, 34, 2));
  out.total_odometer = readAsUInt(d, 36, 20);
  return out;
}

// 0x178/0x188 wheel drive status.
struct WheelStatusData
{
  uint16_t torque{0};
  int16_t rpm{0};
  uint16_t current{0};
};

inline WheelStatusData decodeWheelStatus(const CanFrame & frame)
{
  const uint8_t * d = frame.data;
  WheelStatusData out;
  out.torque = static_cast<uint16_t>(readAsUInt(d, 0, 16));
  out.rpm = readAsInt16(d, 16);
  out.current = static_cast<uint16_t>(readAsUInt(d, 32, 16));
  return out;
}

}  // namespace smartcar_chassis

#endif  // SMARTCAR_CHASSIS_CAN_PROTOCOL_HPP
