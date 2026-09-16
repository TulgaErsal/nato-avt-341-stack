#include "avt_341_msgs/msg/nav_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341_nav/node/node_types.h"
#include "avt_341_nav/core/ros_msg_utils.hpp"
#include "avt_341_nav/atat_speed_control_params_service.hpp"

#include <algorithm>
#include <cmath>
#include <deque>

using avt_341_nav::core::NavStackState;

namespace avt_341_atat {

/// Simple moving-average filter to smooth noisy inputs; port of ATAT's MovingAverageFilter.
class MovingAverageFilter {
public:
  explicit MovingAverageFilter(std::size_t n) : n_(std::max<std::size_t>(1, n)) {}

  double Step(double ref) {
    buffer_.push_front(ref);
    while (buffer_.size() > n_) {
      buffer_.pop_back();
    }
    double sum = 0.0;
    for (double v : buffer_) {
      sum += v;
    }
    return sum / static_cast<double>(buffer_.size());
  }

  void Reset() { buffer_.clear(); }

private:
  std::size_t n_;
  std::deque<double> buffer_;
};

/// Port of ATAT's KPIDController: PID plus a feed-forward polynomial in the reference.
class KpidController {
public:
  KpidController(double k0, double k1, double k2, double kp, double ki, double kd,
                 double dt, double integral_abs_max)
    : k0_(k0), k1_(k1), k2_(k2), kp_(kp), ki_(ki), kd_(kd),
      dt_(dt), integral_abs_max_(integral_abs_max) {}

  double Step(double ref, double meas) {
    const double error = ref - meas;

    // integral with anti-windup clamping
    integral_ += error * dt_;
    integral_ = std::clamp(integral_, -integral_abs_max_, integral_abs_max_);

    // first sample has no prior error to differentiate against
    const double derivative = have_last_error_ ? (error - last_error_) / dt_ : 0.0;
    last_error_ = error;
    have_last_error_ = true;

    double output = 0.0;
    output += k0_ * Sign(ref);
    output += k1_ * ref;
    output += k2_ * ref * ref;
    output += kp_ * error;
    output += ki_ * integral_;
    output += kd_ * derivative;
    return output;
  }

  void Reset() {
    integral_ = 0.0;
    have_last_error_ = false;
  }

private:
  static double Sign(double x) {
    return x == 0.0 ? 0.0 : std::copysign(1.0, x);
  }

  double k0_, k1_, k2_, kp_, ki_, kd_, dt_, integral_abs_max_;
  double integral_ = 0.0;
  double last_error_ = 0.0;
  bool have_last_error_ = false;
};

}  // namespace avt_341_atat

rclcpp::Node::SharedPtr g_node;

nav_msgs::msg::Odometry g_odom;
bool g_have_odom = false;
rclcpp::Time g_last_odom_time;

double g_desired_speed = 0.0;
bool g_have_speed = false;
rclcpp::Time g_last_speed_time;

double g_desired_speed_factor = 1.0;

double g_desired_steer_rad = 0.0;
bool g_have_steer = false;
rclcpp::Time g_last_steer_time;

int g_run_state = NavStackState::NotInit;
bool g_reset_called = false;

void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
  g_odom = *msg;
  g_have_odom = true;
  g_last_odom_time = g_node->now();
}

void DesiredSpeedCallback(std_msgs::msg::Float64::SharedPtr msg) {
  g_desired_speed = msg->data;
  g_have_speed = true;
  g_last_speed_time = g_node->now();
}

void DesiredSpeedFactorCallback(std_msgs::msg::Float64::SharedPtr msg) {
  g_desired_speed_factor = msg->data;
}

void DesiredSteerCallback(std_msgs::msg::Float64::SharedPtr msg) {
  g_desired_steer_rad = msg->data;
  g_have_steer = true;
  g_last_steer_time = g_node->now();
}

void StateCallback(avt_341_msgs::msg::NavState::SharedPtr msg) {
  g_run_state = msg->run_state;
}

void ResetCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (msg->data.find(avt_341_nav::node::NodeType::Control) != std::string::npos) {
    g_reset_called = true;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("avt_341_atat_speed_control_node");

  avt_341_nav::params::atat_speed_control::ParamsListener param_listener(g_node);
  const auto params = param_listener.get_params();

  RCLCPP_INFO(g_node->get_logger(),
      "ATAT KPID Values:\n  k0=%.3f k1=%.3f k2=%.3f\n  kp=%.3f ki=%.3f kd=%.3f\n  "
      "integral_abs_max=%.3f\n  output=[%.3f, %.3f]  brake_drag=%.3f",
      params.feed_forward_k0, params.feed_forward_k1, params.feed_forward_k2,
      params.throttle_kp, params.throttle_ki, params.throttle_kd,
      params.integral_abs_max, params.throttle_output_min, params.throttle_output_max,
      params.brake_drag);

  auto cmd_vel_pub = g_node->create_publisher<geometry_msgs::msg::Twist>("avt_341/cmd_vel", 1);
  auto reset_ack_pub = g_node->create_publisher<std_msgs::msg::String>("avt_341/reset_ack", 1);

  auto odom_sub = g_node->create_subscription<nav_msgs::msg::Odometry>(
      "avt_341/odometry", 1, OdometryCallback);
  auto state_sub = g_node->create_subscription<avt_341_msgs::msg::NavState>(
      "avt_341/state", 1, StateCallback);
  auto speed_sub = g_node->create_subscription<std_msgs::msg::Float64>(
      "avt_341/desired_speed", 1, DesiredSpeedCallback);
  auto speed_factor_sub = g_node->create_subscription<std_msgs::msg::Float64>(
      "avt_341/desired_speed_factor", 1, DesiredSpeedFactorCallback);
  auto steer_sub = g_node->create_subscription<std_msgs::msg::Float64>(
      "avt_341/cmd_steer", 1, DesiredSteerCallback);
  auto reset_sub = g_node->create_subscription<std_msgs::msg::String>(
      "avt_341/reset", 10, ResetCallback);

  avt_341_atat::MovingAverageFilter meas_filter(static_cast<std::size_t>(std::max<int64_t>(1, params.measurement_buffer_size)));
  avt_341_atat::MovingAverageFilter ref_filter(static_cast<std::size_t>(std::max<int64_t>(1, params.reference_buffer_size)));
  avt_341_atat::KpidController controller(
      params.feed_forward_k0, params.feed_forward_k1, params.feed_forward_k2,
      params.throttle_kp, params.throttle_ki, params.throttle_kd,
      1.0 / params.control_rate, params.integral_abs_max);

  rclcpp::Rate r(params.control_rate);
  while (rclcpp::ok()) {
    if (g_reset_called) {
      controller.Reset();
      ref_filter.Reset();
      g_run_state = NavStackState::NotInit;
      std_msgs::msg::String ack;
      ack.data = avt_341_nav::node::NodeType::Control;
      reset_ack_pub->publish(ack);
      g_reset_called = false;
    }

    const auto now = g_node->now();
    const bool odom_stale = !g_have_odom || (now - g_last_odom_time).seconds() > params.odom_timeout;
    const bool speed_stale = !g_have_speed || (now - g_last_speed_time).seconds() > params.control_timeout;
    const bool steer_stale = !g_have_steer || (now - g_last_steer_time).seconds() > params.control_timeout;

    if (odom_stale) {
      RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 1000,
          "odometry timeout detected; halting motion");
    }
    if (speed_stale || steer_stale) {
      RCLCPP_WARN_THROTTLE(g_node->get_logger(), *g_node->get_clock(), 1000,
          "control input timeout detected; halting motion");
    }

    // measurement filter only advances on real odometry, matching ATAT's odom_cb
    const double measured_vel = g_have_odom ? meas_filter.Step(g_odom.twist.twist.linear.x) : 0.0;

    const bool active = !odom_stale && !speed_stale && !steer_stale &&
        g_run_state == static_cast<int>(NavStackState::Active);

    double throttle = 0.0;
    double brake = 0.0;
    double steering_cmd = 0.0;

    if (!active) {
      controller.Reset();
      ref_filter.Reset();
      brake = params.brake_drag;
    } else {
      const double ref_raw = std::clamp(
          g_desired_speed_factor * g_desired_speed,
          params.throttle_input_min, params.throttle_input_max);

      if (std::abs(ref_raw) < 1.0e-6) {
        controller.Reset();
        ref_filter.Reset();
        brake = params.brake_drag;
      } else {
        const double ref = ref_filter.Step(ref_raw);
        double effort = controller.Step(ref, measured_vel);
        effort = std::clamp(effort, params.throttle_output_min, params.throttle_output_max);
        throttle = effort;
      }

      const double steer = std::clamp(
          g_desired_steer_rad, -params.steering_delta_abs_max, params.steering_delta_abs_max);
      steering_cmd = params.output_steering_percent ? steer / params.steering_delta_abs_max : steer;
    }

    geometry_msgs::msg::Twist dc;
    dc.linear.x = throttle;
    dc.linear.y = brake;
    dc.angular.z = steering_cmd;
    cmd_vel_pub->publish(dc);

    rclcpp::spin_some(g_node);
    r.sleep();
  }

  return 0;
}
