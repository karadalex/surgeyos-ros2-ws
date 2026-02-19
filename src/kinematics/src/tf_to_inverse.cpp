#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"

class TfToInverseNode : public rclcpp::Node
{
public:
  TfToInverseNode()
  : Node("tf_to_inverse")
  {
    this->declare_parameter<std::string>("base_frame", "world");
    this->declare_parameter<std::string>("target_frame", "target");
    this->declare_parameter<bool>("elbow_up", false);

    base_frame_ = this->get_parameter("base_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    elbow_up_ = this->get_parameter("elbow_up").as_bool();

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SensorDataQoS(),
      std::bind(&TfToInverseNode::onTf, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Listening on /tf for transform %s -> %s and publishing /joint_states",
      base_frame_.c_str(), target_frame_.c_str());
  }

private:
  static std::string stripSlash(const std::string & frame)
  {
    if (!frame.empty() && frame.front() == '/') {
      return frame.substr(1);
    }
    return frame;
  }

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  bool solveIk(
    double x, double y, double z, double target_pitch,
    std::array<double, 5> & q_out) const
  {
    constexpr double kShoulderRadius = 0.041051;  // L1_to_L2 origin y
    constexpr double kShoulderHeight = 0.062 + 0.0534;  // base_to_L1 + L1_to_L2 origin z
    constexpr double kUpperArm = std::sqrt(0.119401 * 0.119401 + 0.119401 * 0.119401);  // L2_to_L3
    constexpr double kForearm = std::sqrt(0.090485 * 0.090485 + 0.090455 * 0.090455);  // L3_to_L4

    // Joint limits from roarm/urdf/roarm.urdf
    constexpr double q1_min = -3.14;
    constexpr double q1_max = 3.14;
    constexpr double q2_min = -1.0467;
    constexpr double q2_max = 1.0467;
    constexpr double q3_min = -2.7;
    constexpr double q3_max = 2.7;
    constexpr double q4_min = -2.1;
    constexpr double q4_max = 2.1;
    constexpr double q5_min = -1.57;
    constexpr double q5_max = 0.0;

    const double q1 = std::atan2(y, x);
    const double radial = std::hypot(x, y) - kShoulderRadius;
    const double vertical = z - kShoulderHeight;

    const double dist = std::hypot(radial, vertical);
    if (dist < 1e-6) {
      return false;
    }

    double c3 = (dist * dist - kUpperArm * kUpperArm - kForearm * kForearm) /
      (2.0 * kUpperArm * kForearm);
    c3 = clamp(c3, -1.0, 1.0);
    double q3 = std::acos(c3);
    if (elbow_up_) {
      q3 = -q3;
    }

    const double k1 = kUpperArm + kForearm * std::cos(q3);
    const double k2 = kForearm * std::sin(q3);
    const double q2 = std::atan2(vertical, radial) - std::atan2(k2, k1);

    // Use target pitch if provided by TF orientation; otherwise compensates links.
    const double q4 = target_pitch - (q2 + q3);
    const double q5 = -0.5;  // static gripper pose

    q_out = {
      clamp(q1, q1_min, q1_max),
      clamp(q2, q2_min, q2_max),
      clamp(q3, q3_min, q3_max),
      clamp(q4, q4_min, q4_max),
      clamp(q5, q5_min, q5_max)
    };
    return true;
  }

  void onTf(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    const auto expected_parent = stripSlash(base_frame_);
    const auto expected_child = stripSlash(target_frame_);

    for (const auto & tf : msg->transforms) {
      const std::string parent = stripSlash(tf.header.frame_id);
      const std::string child = stripSlash(tf.child_frame_id);
      if (parent != expected_parent || child != expected_child) {
        continue;
      }

      const double x = tf.transform.translation.x;
      const double y = tf.transform.translation.y;
      const double z = tf.transform.translation.z;

      tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      m.getRPY(roll, pitch, yaw);

      (void)roll;
      (void)yaw;

      std::array<double, 5> joints{};
      if (!solveIk(x, y, z, pitch, joints)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "IK failed for transform %s -> %s", parent.c_str(), child.c_str());
        return;
      }

      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.name = {
        "base_to_L1",
        "L1_to_L2",
        "L2_to_L3",
        "L3_to_L4",
        "L4_to_L5_1_A"
      };
      js.position.assign(joints.begin(), joints.end());
      joint_pub_->publish(js);
      return;
    }
  }

  std::string base_frame_;
  std::string target_frame_;
  bool elbow_up_{false};

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfToInverseNode>());
  rclcpp::shutdown();
  return 0;
}
