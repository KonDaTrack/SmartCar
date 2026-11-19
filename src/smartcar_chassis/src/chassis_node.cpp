// Chassis gateway receiver.
//
// The 192.168.100.110 gateway forwards CAN frames to this host over UDP
// (local port 8002 in the ROS 1 deployment). This node publishes the decoded
// 0x51 vehicle status on /vehicle_status. Control (0x121) is intentionally
// not ported yet.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <smartcar_msgs/msg/vehicle_status.hpp>

#include "smartcar_chassis/can_protocol.hpp"

namespace smartcar_chassis
{

class ChassisNode : public rclcpp::Node
{
public:
  ChassisNode()
  : Node("chassis_node")
  {
    bind_address_ = declare_parameter<std::string>("bind_address", "0.0.0.0");
    local_port_ = declare_parameter<int>("local_port", 8002);
    gateway_ip_ = declare_parameter<std::string>("gateway_ip", "192.168.100.110");
    car_type_ = declare_parameter<std::string>("car_type", "JD01");
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0);
    steer_raw_scale_ = declare_parameter<double>("steer_raw_scale", 31.48);
    steer_scale_ = declare_parameter<double>("steer_scale", 1.0);
    debug_frames_ = declare_parameter<bool>("debug_frames", false);

    if (car_type_ != "JD01") {
      RCLCPP_WARN(
        get_logger(), "car_type=%s; this port decodes the JD01 CAN layout",
        car_type_.c_str());
    }

    status_pub_ = create_publisher<smartcar_msgs::msg::VehicleStatus>(
      "vehicle_status", rclcpp::QoS(20));

    openSocket();

    RCLCPP_INFO(
      get_logger(), "chassis gateway %s -> %s:%d (car_type=%s)",
      gateway_ip_.c_str(), bind_address_.c_str(), local_port_, car_type_.c_str());
  }

  ~ChassisNode() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  void run()
  {
    uint8_t buf[64];

    while (rclcpp::ok()) {
      pollfd pfd{};
      pfd.fd = fd_;
      pfd.events = POLLIN;

      const int rc = ::poll(&pfd, 1, 200);
      if (rc < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "poll() failed: %s", std::strerror(errno));
        break;
      }
      if (rc == 0) {
        logStats();
        continue;
      }

      sockaddr_in peer{};
      socklen_t peer_len = sizeof(peer);
      const ssize_t n = ::recvfrom(
        fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&peer), &peer_len);
      if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "recvfrom() failed: %s", std::strerror(errno));
        break;
      }
      if (n < static_cast<ssize_t>(kCanUdpFrameLen)) {
        ++short_frames_;
        continue;
      }

      CanFrame frame;
      if (!parseCanUdp(buf, static_cast<size_t>(n), frame)) {
        ++short_frames_;
        continue;
      }

      peer_ip_ = inet_ntoa(peer.sin_addr);
      handleFrame(frame);
      logStats();
    }
  }

private:
  void openSocket()
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));
    }

    // Deliberately no SO_REUSEADDR: if another driver (for example the ROS 1
    // can_module node) already owns 8002, fail loudly instead of splitting
    // frames between two subscribers.
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(local_port_));
    if (bind_address_ == "0.0.0.0" || bind_address_.empty()) {
      addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, bind_address_.c_str(), &addr.sin_addr) != 1) {
      throw std::runtime_error("invalid bind_address: " + bind_address_);
    }

    if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      throw std::runtime_error(
              "bind(" + bind_address_ + ":" + std::to_string(local_port_) +
              ") failed: " + std::strerror(errno));
    }
  }

  void handleFrame(const CanFrame & frame)
  {
    ++frames_;

    switch (frame.id) {
      case kCanIdVehicleStatus: {
        const auto data = decodeVehicleStatus(frame, steer_raw_scale_, steer_scale_);

        smartcar_msgs::msg::VehicleStatus msg;
        msg.header.stamp = now();
        msg.header.frame_id = "base_link";
        msg.speed = static_cast<float>(data.cur_speed);
        msg.steer = static_cast<float>(data.cur_steer);
        msg.gear = data.shift_level;
        msg.brake = data.is_brake;
        msg.left_wheel_speed = static_cast<float>(left_wheel_speed_);
        msg.right_wheel_speed = static_cast<float>(right_wheel_speed_);
        msg.total_odometer = static_cast<float>(data.total_odometer);
        status_pub_->publish(msg);

        ++status_frames_;
        last_speed_ = data.cur_speed;
        last_steer_ = data.cur_steer;

        if (debug_frames_) {
          RCLCPP_INFO(
            get_logger(),
            "0x51 speed=%.2f m/s steer=%.2f deg gear=%u brake=%d odometer=%.1f km",
            data.cur_speed, data.cur_steer, data.shift_level, data.is_brake,
            data.total_odometer);
        }
        break;
      }

      case kCanIdWheelLeft:
      case kCanIdWheelRight: {
        const auto wheel = decodeWheelStatus(frame);
        const double speed = wheel_radius_ > 0.0
          ? wheel.rpm * 2.0 * M_PI * wheel_radius_ / 60.0
          : 0.0;
        if (frame.id == kCanIdWheelLeft) {
          left_wheel_speed_ = speed;
          ++left_wheel_frames_;
        } else {
          right_wheel_speed_ = speed;
          ++right_wheel_frames_;
        }
        break;
      }

      case kCanIdBattery:
      case kCanIdBatteryLegacy:
        ++battery_frames_;
        break;

      default:
        ++unknown_frames_;
        break;
    }
  }

  void logStats()
  {
    const auto now_tp = std::chrono::steady_clock::now();
    if (now_tp - last_stats_time_ < std::chrono::seconds(5)) {
      return;
    }
    last_stats_time_ = now_tp;

    RCLCPP_INFO(
      get_logger(),
      "peer=%s frames=%llu status=%llu wheelL=%llu wheelR=%llu battery=%llu "
      "unknown=%llu short=%llu speed=%.2f steer=%.2f",
      peer_ip_.c_str(),
      static_cast<unsigned long long>(frames_),
      static_cast<unsigned long long>(status_frames_),
      static_cast<unsigned long long>(left_wheel_frames_),
      static_cast<unsigned long long>(right_wheel_frames_),
      static_cast<unsigned long long>(battery_frames_),
      static_cast<unsigned long long>(unknown_frames_),
      static_cast<unsigned long long>(short_frames_),
      last_speed_, last_steer_);
  }

  std::string bind_address_;
  int local_port_{8002};
  std::string gateway_ip_;
  std::string car_type_;
  double wheel_radius_{0.0};
  double steer_raw_scale_{31.48};
  double steer_scale_{1.0};
  bool debug_frames_{false};

  int fd_{-1};
  std::string peer_ip_{"none"};

  double left_wheel_speed_{0.0};
  double right_wheel_speed_{0.0};
  double last_speed_{0.0};
  double last_steer_{0.0};

  uint64_t frames_{0};
  uint64_t status_frames_{0};
  uint64_t left_wheel_frames_{0};
  uint64_t right_wheel_frames_{0};
  uint64_t battery_frames_{0};
  uint64_t unknown_frames_{0};
  uint64_t short_frames_{0};
  std::chrono::steady_clock::time_point last_stats_time_{std::chrono::steady_clock::now()};

  rclcpp::Publisher<smartcar_msgs::msg::VehicleStatus>::SharedPtr status_pub_;
};

}  // namespace smartcar_chassis

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<smartcar_chassis::ChassisNode>();
    node->run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("chassis_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
