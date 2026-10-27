// Wheel/steering odometry from /vehicle_status.
//
// Ported from the vehicle's ROS 1 can_odometry package. Parameters follow the
// yuhuo JD01 launch: wheel_base=1.2, is_akm=true. The steering angle reported
// by the chassis is already divided by 31.48 in chassis_node; steer_angle_scale
// is the single calibration knob for the heading (negative flips the sign).

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smartcar_msgs/msg/vehicle_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace smartcar_chassis
{

const std::array<double, 36> kOdomPoseCovariance = {
  1e-3, 0, 0, 0, 0, 0,
  0, 1e-3, 0, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e3};

const std::array<double, 36> kOdomPoseCovariance2 = {
  1e-9, 0, 0, 0, 0, 0,
  0, 1e-3, 1e-9, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e-9};

const std::array<double, 36> kOdomTwistCovariance = {
  1e-3, 0, 0, 0, 0, 0,
  0, 1e-3, 0, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e3};

const std::array<double, 36> kOdomTwistCovariance2 = {
  1e-9, 0, 0, 0, 0, 0,
  0, 1e-3, 1e-9, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e-9};

class CanOdometryNode : public rclcpp::Node
{
public:
  CanOdometryNode()
  : Node("can_odometry_node")
  {
    wheel_base_ = declare_parameter<double>("wheel_base", 1.2);
    is_akm_ = declare_parameter<bool>("is_akm", true);
    steer_angle_scale_ = declare_parameter<double>("steer_angle_scale", 1.0);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    max_dt_ = declare_parameter<double>("max_dt", 0.5);

    if (wheel_base_ <= 0.0) {
      throw std::runtime_error("wheel_base must be > 0");
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    status_sub_ = create_subscription<smartcar_msgs::msg::VehicleStatus>(
      "vehicle_status", rclcpp::QoS(10),
      std::bind(&CanOdometryNode::onVehicleStatus, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "odometry: wheel_base=%.3f is_akm=%d steer_angle_scale=%.3f frames=%s->%s",
      wheel_base_, is_akm_, steer_angle_scale_, odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void onVehicleStatus(const smartcar_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    double vx = msg->speed;
    const double angle_rad = -deg2rad(msg->steer * steer_angle_scale_);
    double vth = 0.0;

    if (is_akm_) {
      if (msg->gear == 1) {           // D
        // vx stays positive
      } else if (msg->gear == 3) {    // R
        vx = -vx;
      } else {
        vx = 0.0;
      }
      vth = std::tan(angle_rad) * vx / wheel_base_;
    } else {
      vth = (msg->right_wheel_speed - msg->left_wheel_speed) / wheel_base_;
      vx = (msg->left_wheel_speed + msg->right_wheel_speed) * 0.5;
    }

    const double stamp_sec =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    if (has_last_stamp_) {
      const double dt = stamp_sec - last_stamp_sec_;
      if (dt > 0.0 && dt <= max_dt_) {
        x_ += vx * std::cos(th_) * dt;
        y_ += vx * std::sin(th_) * dt;
        th_ += vth * dt;
        th_ = std::atan2(std::sin(th_), std::cos(th_));
      }
    }
    last_stamp_sec_ = stamp_sec;
    has_last_stamp_ = true;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, th_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;

    const bool stationary =
      std::fabs(msg->speed) <= 0.05 && std::fabs(msg->steer) < 0.05;
    const auto & pose_cov = stationary ? kOdomPoseCovariance2 : kOdomPoseCovariance;
    const auto & twist_cov = stationary ? kOdomTwistCovariance2 : kOdomTwistCovariance;
    std::copy(pose_cov.begin(), pose_cov.end(), odom.pose.covariance.begin());
    std::copy(twist_cov.begin(), twist_cov.end(), odom.twist.covariance.begin());

    odom_pub_->publish(odom);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }
  }

  static double deg2rad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  double wheel_base_{1.2};
  bool is_akm_{true};
  double steer_angle_scale_{1.0};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  bool publish_tf_{true};
  double max_dt_{0.5};

  double x_{0.0};
  double y_{0.0};
  double th_{0.0};
  double last_stamp_sec_{0.0};
  bool has_last_stamp_{false};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<smartcar_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace smartcar_chassis

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<smartcar_chassis::CanOdometryNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("can_odometry_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
