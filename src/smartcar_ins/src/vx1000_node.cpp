// VX1000 integrated navigation unit (INS/IMU/GNSS) serial driver.
//
// Ported from the vehicle's ROS 1 vx1000_drv. The wire protocol and scaling
// factors are kept identical: 68-byte frames, header 0xAA 0x55, type at
// byte 2, 64-byte payload at byte 3, checksum at byte 67 (8-bit sum of the
// first 67 bytes).
//
// ROS 2 additions: std_msgs/Header on every custom topic, plus standard
// sensor_msgs/Imu and sensor_msgs/NavSatFix topics. No time sync, extrinsic
// calibration or filtering is applied yet.

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <smartcar_msgs/msg/vx1000_cov.hpp>
#include <smartcar_msgs/msg/vx1000_gnss.hpp>
#include <smartcar_msgs/msg/vx1000_imu.hpp>
#include <smartcar_msgs/msg/vx1000_ins.hpp>
#include <smartcar_msgs/msg/vx1000_odom.hpp>
#include <smartcar_msgs/msg/vx1000_stat.hpp>

#include "smartcar_ins/vx1000_protocol.hpp"

namespace smartcar_ins
{

static_assert(sizeof(InternalCan_InsFrame_TypeDef) == 64, "INS frame must be 64 bytes");
static_assert(sizeof(InternalCan_ImuFrame_TypeDef) == 64, "IMU frame must be 64 bytes");
static_assert(sizeof(InternalCan_GnssFrame_TypeDef) == 64, "GNSS frame must be 64 bytes");
static_assert(sizeof(InternalCan_StateFrame_TypeDef) == 64, "STAT frame must be 64 bytes");
static_assert(sizeof(InternalCan_CovFrame_TypeDef) == 64, "COV frame must be 64 bytes");
static_assert(sizeof(InternalCan_OdomFrame_TypeDef) == 64, "ODOM frame must be 64 bytes");

namespace
{

speed_t baudToSpeed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default:
      throw std::invalid_argument("unsupported VX1000 baud rate: " + std::to_string(baud));
  }
}

}  // namespace

class Vx1000Node : public rclcpp::Node
{
public:
  Vx1000Node()
  : Node("vx1000_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 460800);
    frame_id_ = declare_parameter<std::string>("frame_id", "vx1000_link");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "vx1000_imu");
    gnss_frame_id_ = declare_parameter<std::string>("gnss_frame_id", "vx1000_gnss");
    publish_standard_ = declare_parameter<bool>("publish_standard_topics", true);
    // The ROS 1 driver exposes the raw gyro field in deg/s (offset 500).
    gyro_in_deg_ = declare_parameter<bool>("gyro_in_deg", true);

    ins_pub_ = create_publisher<smartcar_msgs::msg::Vx1000Ins>("VX1000_ins", rclcpp::QoS(100));
    imu_pub_ = create_publisher<smartcar_msgs::msg::Vx1000Imu>("VX1000_imu", rclcpp::QoS(100));
    gnss_pub_ = create_publisher<smartcar_msgs::msg::Vx1000Gnss>("VX1000_gnss", rclcpp::QoS(100));
    stat_pub_ = create_publisher<smartcar_msgs::msg::Vx1000Stat>("VX1000_stat", rclcpp::QoS(100));
    cov_pub_ = create_publisher<smartcar_msgs::msg::Vx1000Cov>("VX1000_cov", rclcpp::QoS(100));
    odom_pub_ = create_publisher<smartcar_msgs::msg::Vx1000Odom>("VX1000_odom", rclcpp::QoS(100));

    if (publish_standard_) {
      standard_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(100));
      standard_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", rclcpp::QoS(100));
    }

    openSerial();

    RCLCPP_INFO(
      get_logger(), "VX1000 opened %s @ %d, frame_id=%s",
      port_.c_str(), baudrate_, frame_id_.c_str());
  }

  ~Vx1000Node() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  void run()
  {
    std::array<uint8_t, 4096> chunk{};

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
      if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        RCLCPP_ERROR(get_logger(), "serial device hung up (revents=0x%x)", pfd.revents);
        break;
      }

      const ssize_t n = ::read(fd_, chunk.data(), chunk.size());
      if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "read() failed: %s", std::strerror(errno));
        break;
      }
      if (n == 0) {
        continue;
      }

      buffer_.insert(buffer_.end(), chunk.data(), chunk.data() + n);
      if (buffer_.size() > kMaxBufferBytes) {
        dropped_bytes_ += buffer_.size() - Vx1000_PROTO_LEN;
        buffer_.erase(buffer_.begin(), buffer_.end() - Vx1000_PROTO_LEN);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "VX1000 receive buffer overflow, dropping unparsed bytes");
      }

      processBuffer();
      logStats();
    }
  }

private:
  static constexpr size_t kMaxBufferBytes = 1024 * 1024;

  void openSerial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error(
              "failed to open " + port_ + ": " + std::strerror(errno));
    }

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      throw std::runtime_error(
              "tcgetattr(" + port_ + ") failed: " + std::strerror(errno));
    }

    ::cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    const speed_t speed = baudToSpeed(baudrate_);
    if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0) {
      throw std::runtime_error("failed to set baud rate on " + port_);
    }
    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error(
              "tcsetattr(" + port_ + ") failed: " + std::strerror(errno));
    }
    ::tcflush(fd_, TCIFLUSH);
  }

  void processBuffer()
  {
    while (buffer_.size() >= 2) {
      if (buffer_[0] != VX1000_FRAME_HEAD_0) {
        const auto it = std::find(buffer_.begin(), buffer_.end(), VX1000_FRAME_HEAD_0);
        dropped_bytes_ += static_cast<uint64_t>(std::distance(buffer_.begin(), it));
        buffer_.erase(buffer_.begin(), it);
        if (buffer_.size() < 2) {
          return;
        }
      }

      if (buffer_[1] != VX1000_FRAME_HEAD_1) {
        buffer_.erase(buffer_.begin());
        ++dropped_bytes_;
        continue;
      }

      if (buffer_.size() < Vx1000_PROTO_LEN) {
        return;
      }

      uint8_t checksum = 0;
      for (size_t i = 0; i < Vx1000_PROTO_LEN - 1; ++i) {
        checksum = static_cast<uint8_t>(checksum + buffer_[i]);
      }
      if (checksum != buffer_[Vx1000_PROTO_LEN - 1]) {
        ++checksum_errors_;
        buffer_.erase(buffer_.begin());
        continue;
      }

      handleFrame(buffer_.data());
      ++frames_;
      buffer_.erase(buffer_.begin(), buffer_.begin() + Vx1000_PROTO_LEN);
    }
  }

  void handleFrame(const uint8_t * frame)
  {
    const uint8_t type = frame[2];
    if (type < VX1000_TYPE_1 || type > VX1000_TYPE_8) {
      return;
    }
    ++type_counts_[type - VX1000_TYPE_1];

    switch (type) {
      case VX1000_TYPE_1: handleIns(frame); break;
      case VX1000_TYPE_2: handleImu(frame); break;
      case VX1000_TYPE_3: handleGnss(frame); break;
      case VX1000_TYPE_4: handleStat(frame); break;
      case VX1000_TYPE_5: handleCov(frame); break;
      case VX1000_TYPE_6: handleOdom(frame); break;
      default: break;
    }
  }

  void handleIns(const uint8_t * frame)
  {
    InternalCan_InsFrame_TypeDef f{};
    std::memcpy(&f.Data, frame + 3, sizeof(f.Data));

    smartcar_msgs::msg::Vx1000Ins msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.gps_week = f.bit.gps_week;
    msg.gps_week_second = static_cast<float>(f.bit.gps_week_second * 1e-3);
    msg.sys_tick = f.bit.sys_tick;
    msg.navi_tick = static_cast<int8_t>(f.bit.ins_nav_stat - 1);
    msg.ins_lat = f.bit.ins_lat * 1e-10 - 90.0;
    msg.ins_lon = f.bit.ins_lon * 1e-10 - 180.0;
    msg.ins_alt = static_cast<float>(f.bit.ins_alt * 1e-3 - 1000.0);
    msg.ins_vel_n = static_cast<float>(f.bit.ins_vel_n * 1e-3 - 100.0);
    msg.ins_vel_e = static_cast<float>(f.bit.ins_vel_e * 1e-3 - 100.0);
    msg.ins_vel_d = static_cast<float>(f.bit.ins_vel_d * 1e-3 - 100.0);
    msg.ins_roll = static_cast<float>(f.bit.ins_roll * 1e-4 - 180.0);
    msg.ins_pitch = static_cast<float>(f.bit.ins_pitch * 1e-4 - 90.0);
    msg.ins_heading = static_cast<float>(f.bit.ins_heading * 1e-4);
    msg.cycle_cnt_ins = f.bit.cycle_cnt;

    ins_pub_->publish(msg);
  }

  void handleImu(const uint8_t * frame)
  {
    InternalCan_ImuFrame_TypeDef f{};
    std::memcpy(&f.Data, frame + 3, sizeof(f.Data));

    smartcar_msgs::msg::Vx1000Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.gps_week = f.bit.gps_week;
    msg.gps_week_second = static_cast<float>(f.bit.gps_week_second * 1e-3);
    msg.gyr_x = f.bit.gyr_x * 1e-5 - 500.0;
    msg.gyr_y = f.bit.gyr_y * 1e-5 - 500.0;
    msg.gyr_z = f.bit.gyr_z * 1e-5 - 500.0;
    msg.acc_x = f.bit.acc_x * 1e-5 - 400.0;
    msg.acc_y = f.bit.acc_y * 1e-5 - 400.0;
    msg.acc_z = f.bit.acc_z * 1e-5 - 400.0;
    msg.gyr_temp = static_cast<float>(f.bit.gyr_temp * 1e-2 - 128.0);
    msg.acc_temp = static_cast<float>(f.bit.acc_temp * 1e-2 - 128.0);
    msg.gyr_self_test_flag = f.bit.gyr_self_test_flg;
    msg.gyr_config_flag = f.bit.gyr_cfg_flg;
    msg.gyr_run_stat = f.bit.gyr_run_stat;
    msg.gyr_comu_err = f.bit.gyr_comu_err;
    msg.gyr_crc_err = f.bit.gyr_crc_err;
    msg.gyr_obt_err = f.bit.gyr_obt_err;
    msg.gyr_over_range = f.bit.gyr_over_range;
    msg.gyr_temp_stat = f.bit.gyr_temp_stat;
    msg.acc_self_test_flag = f.bit.acc_self_test_flg;
    msg.acc_config_flag = f.bit.acc_cfg_flg;
    msg.acc_run_stat = f.bit.acc_run_stat;
    msg.acc_comu_err = f.bit.acc_comu_err;
    msg.acc_crc_err = f.bit.acc_crc_err;
    msg.acc_obt_err = f.bit.acc_obt_err;
    msg.acc_over_range = f.bit.acc_over_range;
    msg.acc_temp_stat = f.bit.acc_temp_stat;
    msg.cycle_cnt_imu = f.bit.cycle_cnt;

    imu_pub_->publish(msg);

    if (standard_imu_pub_) {
      sensor_msgs::msg::Imu out;
      out.header.stamp = msg.header.stamp;
      out.header.frame_id = imu_frame_id_;
      const double gyro_scale = gyro_in_deg_ ? M_PI / 180.0 : 1.0;
      out.angular_velocity.x = msg.gyr_x * gyro_scale;
      out.angular_velocity.y = msg.gyr_y * gyro_scale;
      out.angular_velocity.z = msg.gyr_z * gyro_scale;
      out.linear_acceleration.x = msg.acc_x;
      out.linear_acceleration.y = msg.acc_y;
      out.linear_acceleration.z = msg.acc_z;
      out.orientation.w = 1.0;
      // The VX1000 IMU frame carries no orientation; mark it unavailable.
      out.orientation_covariance[0] = -1.0;
      standard_imu_pub_->publish(out);
    }
  }

  void handleGnss(const uint8_t * frame)
  {
    InternalCan_GnssFrame_TypeDef f{};
    std::memcpy(&f.Data, frame + 3, sizeof(f.Data));

    smartcar_msgs::msg::Vx1000Gnss msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.gps_week = f.bit.gps_week;
    msg.gps_week_second = static_cast<float>(f.bit.gps_week_second * 1e-3);
    // The wire field is years since 2000 (e.g. 26), not the full year.
    msg.utc_year = static_cast<uint16_t>(
      f.bit.utc_year < 100 ? f.bit.utc_year + 2000 : f.bit.utc_year);
    msg.utc_month = f.bit.utc_month;
    msg.utc_day = f.bit.utc_day;
    msg.utc_hour = f.bit.utc_hour;
    msg.utc_minute = f.bit.utc_min;
    msg.utc_second = f.bit.utc_sec;
    msg.utc_millisecond = f.bit.utc_ms;
    msg.gnss_fix_mode = f.bit.gnss_fix_mode;
    msg.gnss_head_mode = f.bit.gnss_head_mode;
    msg.gnss_diff_age = f.bit.gnss_diff_age;
    msg.gnss_sat_cnt = f.bit.gnss_sat_cnt;
    msg.gnss_sat_cnt_slove = f.bit.gnss_sat_cnt_solve;
    msg.gnss_lat = f.bit.gnss_lat * 1e-10 - 90.0;
    msg.gnss_lon = f.bit.gnss_lon * 1e-10 - 180.0;
    msg.gnss_alt = static_cast<float>(f.bit.gnss_alt * 1e-3 - 1000.0);
    msg.gnss_trace_angle = static_cast<float>(f.bit.gnss_trace_angle * 1e-2);
    msg.gnss_heading = static_cast<float>(f.bit.gnss_heading * 1e-2);
    // Same -100 offset as the velocity components (observed 100.01 at rest
    // with the ROS 1 formula, which omitted it).
    msg.gnss_speed = static_cast<float>(f.bit.gnss_speed * 1e-2 - 100.0);
    msg.gnss_vel_n = static_cast<float>(f.bit.gnss_vel_n * 1e-2 - 100.0);
    msg.gnss_vel_e = static_cast<float>(f.bit.gnss_vel_e * 1e-2 - 100.0);
    msg.gnss_vel_d = static_cast<float>(f.bit.gnss_vel_d * 1e-2 - 100.0);
    msg.gnss_pdop = static_cast<float>(f.bit.gnss_pdop * 1e-2);
    msg.gnss_hdop = static_cast<float>(f.bit.gnss_hdop * 1e-2);
    msg.gnss_vdop = static_cast<float>(f.bit.gnss_vdop * 1e-2);
    msg.gnss_lat_std = static_cast<float>(f.bit.gnss_lat_std * 1e-2);
    msg.gnss_lon_std = static_cast<float>(f.bit.gnss_lon_std * 1e-2);
    msg.gnss_head_std = static_cast<float>(f.bit.gnss_head_std * 1e-2);
    msg.gnss_self_test_flag = f.bit.gnss_self_test_flg;
    msg.gnss_config_flag = f.bit.gnss_cfg_flg;
    msg.gnss_run_stat = f.bit.gnss_run_stat;
    msg.gnss_comu_stat = f.bit.gnss_comu_stat;
    msg.gnss_pos_stat = f.bit.gnss_pos_stat;
    msg.gnss_head_stat = f.bit.gnss_head_stat;
    msg.gnss_rtcm_stat = f.bit.gnss_rtcm_stat;
    msg.gnss_ant1_stat = f.bit.gnss_ant1_stat;
    msg.gnss_ant2_stat = f.bit.gnss_ant2_stat;
    msg.pps_stat = f.bit.pps_stat;
    msg.gps_time_stat = f.bit.GpsTimeStat;
    msg.cycle_cnt_gnss = f.bit.cycle_cnt;

    gnss_pub_->publish(msg);

    if (standard_fix_pub_) {
      sensor_msgs::msg::NavSatFix out;
      out.header.stamp = msg.header.stamp;
      out.header.frame_id = gnss_frame_id_;
      out.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

      // Observed on the vehicle: mode 1 with a valid position and 23 solved
      // satellites is a single-point fix, so only mode 0 is "no fix".
      // Mode 6 is RTK fixed, reported as GBAS_FIX (highest status available).
      if (msg.gnss_fix_mode == 0) {
        out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      } else if (msg.gnss_fix_mode == 6) {
        out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      } else {
        out.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      }

      out.latitude = msg.gnss_lat;
      out.longitude = msg.gnss_lon;
      out.altitude = msg.gnss_alt;
      out.position_covariance[0] = msg.gnss_lat_std * msg.gnss_lat_std;
      out.position_covariance[4] = msg.gnss_lon_std * msg.gnss_lon_std;
      // No vertical standard deviation in the vendor frame; use the
      // horizontal average so the value is not reported as perfect.
      out.position_covariance[8] =
        (msg.gnss_lat_std * msg.gnss_lat_std +
         msg.gnss_lon_std * msg.gnss_lon_std) * 0.5;
      out.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      standard_fix_pub_->publish(out);
    }
  }

  void handleStat(const uint8_t * frame)
  {
    InternalCan_StateFrame_TypeDef f{};
    std::memcpy(&f.Data, frame + 3, sizeof(f.Data));

    smartcar_msgs::msg::Vx1000Stat msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.gps_week = f.bit.gps_week;
    msg.gps_week_second = static_cast<float>(f.bit.gps_week_second * 1e-3);
    msg.sys_tick = f.bit.sys_tick;
    msg.vol_sup = static_cast<float>(f.bit.vol_sup * 1e-1 - 40.0);
    msg.vol_sys = static_cast<float>(f.bit.vol_sys * 1e-1 - 40.0);
    msg.vol_gnss = static_cast<float>(f.bit.vol_gnss * 1e-1 - 40.0);
    msg.vol_ant = static_cast<float>(f.bit.vol_ant * 1e-1 - 40.0);
    msg.sys_temp = static_cast<float>(f.bit.sys_temp * 1e-2 - 128.0);
    msg.sys_work_stat = f.bit.sys_work_stat;
    msg.core_start_up = f.bit.core_start_up;
    msg.mon_start_up = f.bit.mon_start_up;
    msg.para_read_flag = f.bit.para_read_flag;
    msg.vol_sup_stat = f.bit.vol_sup_stat;
    msg.vol_sys_stat = f.bit.vol_sys_stat;
    msg.vol_gnss_stat = f.bit.vol_gnss_stat;
    msg.vol_ant_stat = f.bit.vol_ant_stat;
    msg.vol_mcu_stat = f.bit.vol_mcu_stat;
    msg.vol_imu_stat = f.bit.vol_imu_stat;
    msg.vol_can_stat = f.bit.vol_can_stat;
    msg.eth_init_flag = f.bit.eth_init_flg;
    msg.eth_dhcp_stat = f.bit.eth_dhcp_stat;
    msg.eth_link_stat = f.bit.eth_link_stat;
    msg.ins_nav_stat = static_cast<int8_t>(f.bit.ins_nav_stat - 1);
    msg.gyr_stat = f.bit.gyr_stat;
    msg.acc_stat = f.bit.acc_stat;
    msg.gnss_stat = f.bit.gnss_stat;
    msg.odom_stat = f.bit.odom_stat;
    msg.cycle_cnt_state = f.bit.cycle_cnt;

    stat_pub_->publish(msg);
  }

  void handleCov(const uint8_t * frame)
  {
    InternalCan_CovFrame_TypeDef f{};
    std::memcpy(&f.Data, frame + 3, sizeof(f.Data));

    smartcar_msgs::msg::Vx1000Cov msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.gps_week = f.bit.gps_week;
    msg.gps_week_second = static_cast<float>(f.bit.gps_week_second * 1e-3);
    msg.ins_lat_err = static_cast<float>(f.bit.ins_lat_err * 1e-3);
    msg.ins_lon_err = static_cast<float>(f.bit.ins_lon_err * 1e-3);
    msg.ins_alt_err = static_cast<float>(f.bit.ins_alt_err * 1e-3);
    msg.ins_lat_conf = f.bit.ins_lat_conf;
    msg.ins_lon_conf = f.bit.ins_lon_conf;
    msg.ins_alt_conf = f.bit.ins_alt_conf;
    msg.ins_heading_err = static_cast<float>(f.bit.ins_heading_err * 1e-3);
    msg.ins_pitch_err = static_cast<float>(f.bit.ins_pitch_err * 1e-3);
    msg.ins_roll_err = static_cast<float>(f.bit.ins_roll_err * 1e-3);
    msg.ins_heading_conf = f.bit.ins_heading_conf;
    msg.ins_pitch_conf = f.bit.ins_pitch_conf;
    msg.ins_roll_conf = f.bit.ins_roll_conf;
    msg.ins_vel_n_err = static_cast<float>(f.bit.ins_vel_n_err * 1e-3);
    msg.ins_vel_e_err = static_cast<float>(f.bit.ins_vel_e_err * 1e-3);
    msg.ins_vel_d_err = static_cast<float>(f.bit.ins_vel_d_err * 1e-3);
    msg.ins_vel_n_conf = f.bit.ins_vel_n_conf;
    msg.ins_vel_e_conf = f.bit.ins_vel_e_conf;
    msg.ins_vel_d_conf = f.bit.ins_vel_d_conf;
    msg.ins_cali_prog = f.bit.ins_cali_prog;
    msg.cycle_cnt_cov = f.bit.cycle_cnt;

    cov_pub_->publish(msg);
  }

  void handleOdom(const uint8_t * frame)
  {
    InternalCan_OdomFrame_TypeDef f{};
    std::memcpy(&f.Data, frame + 3, sizeof(f.Data));

    smartcar_msgs::msg::Vx1000Odom msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.gps_week = f.bit.gps_week;
    msg.gps_week_second = static_cast<float>(f.bit.gps_week_second * 1e-3);
    msg.odom_gear = f.bit.odom_gear;
    msg.wheel_spd_fl = static_cast<float>(f.bit.wheel_spd_fl * 1e-3 - 100.0);
    msg.wheel_spd_fr = static_cast<float>(f.bit.wheel_spd_fr * 1e-3 - 100.0);
    msg.wheel_spd_rl = static_cast<float>(f.bit.wheel_spd_rl * 1e-3 - 100.0);
    msg.wheel_spd_rr = static_cast<float>(f.bit.wheel_spd_rr * 1e-3 - 100.0);
    msg.vehicle_spd = static_cast<float>(f.bit.vehicle_spd * 1e-3 - 100.0);
    msg.wheel_pulse_fl = f.bit.wheel_pulse_fl;
    msg.wheel_pulse_fr = f.bit.wheel_pulse_fr;
    msg.wheel_pulse_rl = f.bit.wheel_pulse_rl;
    msg.wheel_pulse_rr = f.bit.wheel_pulse_rr;
    msg.steer_wheel_angle = static_cast<float>(f.bit.steer_wheel_agl * 1e-1 - 720.0);
    msg.steer_wheel_speed = static_cast<uint16_t>(f.bit.steer_wheel_spd * 2);
    msg.wheel_scale_l = static_cast<float>(f.bit.wheel_scale_l * 1e-5 - 5.0);
    msg.wheel_scale_r = static_cast<float>(f.bit.wheel_scale_r * 1e-5 - 5.0);
    msg.odom_run_stat = f.bit.odom_run_stat;
    msg.gear_err = f.bit.gear_err;
    msg.wheel_spd_err = f.bit.wheel_spd_err;
    msg.vehicle_spd_err = f.bit.veh_spd_err;
    msg.wheel_pulse_err = f.bit.wheel_pulse_err;
    msg.steer_wheel_err = f.bit.steer_wheel_err;
    msg.cycle_cnt_odom = f.bit.cycle_cnt;

    odom_pub_->publish(msg);
  }

  void logStats()
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_stats_time_ < std::chrono::seconds(5)) {
      return;
    }
    last_stats_time_ = now;

    RCLCPP_INFO(
      get_logger(),
      "frames=%llu ins=%llu imu=%llu gnss=%llu stat=%llu cov=%llu odom=%llu "
      "checksum_errors=%llu dropped_bytes=%llu",
      static_cast<unsigned long long>(frames_),
      static_cast<unsigned long long>(type_counts_[0]),
      static_cast<unsigned long long>(type_counts_[1]),
      static_cast<unsigned long long>(type_counts_[2]),
      static_cast<unsigned long long>(type_counts_[3]),
      static_cast<unsigned long long>(type_counts_[4]),
      static_cast<unsigned long long>(type_counts_[5]),
      static_cast<unsigned long long>(checksum_errors_),
      static_cast<unsigned long long>(dropped_bytes_));
  }

  std::string port_;
  int baudrate_{460800};
  std::string frame_id_;
  std::string imu_frame_id_;
  std::string gnss_frame_id_;
  bool publish_standard_{true};
  bool gyro_in_deg_{true};

  int fd_{-1};
  std::vector<uint8_t> buffer_;

  uint64_t frames_{0};
  uint64_t checksum_errors_{0};
  uint64_t dropped_bytes_{0};
  std::array<uint64_t, 8> type_counts_{};
  std::chrono::steady_clock::time_point last_stats_time_{std::chrono::steady_clock::now()};

  rclcpp::Publisher<smartcar_msgs::msg::Vx1000Ins>::SharedPtr ins_pub_;
  rclcpp::Publisher<smartcar_msgs::msg::Vx1000Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<smartcar_msgs::msg::Vx1000Gnss>::SharedPtr gnss_pub_;
  rclcpp::Publisher<smartcar_msgs::msg::Vx1000Stat>::SharedPtr stat_pub_;
  rclcpp::Publisher<smartcar_msgs::msg::Vx1000Cov>::SharedPtr cov_pub_;
  rclcpp::Publisher<smartcar_msgs::msg::Vx1000Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr standard_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr standard_fix_pub_;
};

}  // namespace smartcar_ins

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<smartcar_ins::Vx1000Node>();
    node->run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("vx1000_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
