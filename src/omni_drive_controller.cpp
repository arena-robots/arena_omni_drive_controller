// Copyright 2026 dev@voshch.dev (Volodymyr Shcherbyna)
// SPDX-License-Identifier: Apache-2.0

#include "arena_omni_drive_controller/omni_drive_controller.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
double wrap_to_pi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

namespace arena_omni_drive_controller
{

controller_interface::CallbackReturn OmniDriveController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "param init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
OmniDriveController::command_interface_configuration() const
{
  std::vector<std::string> names;
  names.reserve(params_.wheel_joint_names.size());
  for (const auto & j : params_.wheel_joint_names) {
    names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::InterfaceConfiguration
OmniDriveController::state_interface_configuration() const
{
  std::vector<std::string> names;
  names.reserve(params_.wheel_joint_names.size());
  for (const auto & j : params_.wheel_joint_names) {
    names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn
OmniDriveController::on_configure(const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();
  limiter_x_ = SpeedLimiter(
    params_.linear.x.has_velocity_limits, params_.linear.x.min_velocity,
    params_.linear.x.max_velocity, params_.linear.x.has_acceleration_limits,
    params_.linear.x.min_acceleration, params_.linear.x.max_acceleration);
  limiter_y_ = SpeedLimiter(
    params_.linear.y.has_velocity_limits, params_.linear.y.min_velocity,
    params_.linear.y.max_velocity, params_.linear.y.has_acceleration_limits,
    params_.linear.y.min_acceleration, params_.linear.y.max_acceleration);
  limiter_wz_ = SpeedLimiter(
    params_.angular.z.has_velocity_limits, params_.angular.z.min_velocity,
    params_.angular.z.max_velocity, params_.angular.z.has_acceleration_limits,
    params_.angular.z.min_acceleration, params_.angular.z.max_acceleration);
  wheel_count_ = params_.wheel_joint_names.size();

  // Cross-field length validation that generate_parameter_library can't express.
  if (params_.wheel_positions_x.size() != wheel_count_ ||
    params_.wheel_positions_y.size() != wheel_count_ ||
    params_.wheel_angles.size() != wheel_count_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "wheel_joint_names (%zu), wheel_positions_x (%zu), wheel_positions_y (%zu), "
      "wheel_angles (%zu) must all have the same length",
      wheel_count_, params_.wheel_positions_x.size(),
      params_.wheel_positions_y.size(), params_.wheel_angles.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto cmd_qos = rclcpp::QoS(10).reliable();
  if (params_.use_stamped_vel) {
    cmd_vel_stamped_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_vel", cmd_qos,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        cmd_vel_buffer_.writeFromNonRT(msg);
      });
  } else {
    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/cmd_vel", cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        auto stamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
        stamped->header.stamp = get_node()->now();
        stamped->twist = *msg;
        cmd_vel_buffer_.writeFromNonRT(stamped);
      });
  }

  if (params_.publish_odom) {
    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "~/odometry", rclcpp::SystemDefaultsQoS());
    rt_odom_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);
    rt_odom_pub_->msg_.header.frame_id = params_.odom_frame_id;
    rt_odom_pub_->msg_.child_frame_id = params_.base_frame_id;
    for (std::size_t i = 0; i < 6; ++i) {
      rt_odom_pub_->msg_.pose.covariance[i * 6 + i] = params_.pose_covariance_diagonal[i];
      rt_odom_pub_->msg_.twist.covariance[i * 6 + i] = params_.twist_covariance_diagonal[i];
    }
  }
  if (params_.publish_tf) {
    tf_pub_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
    rt_tf_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_pub_);
    rt_tf_pub_->msg_.transforms.resize(1);
    rt_tf_pub_->msg_.transforms[0].header.frame_id = params_.odom_frame_id;
    rt_tf_pub_->msg_.transforms[0].child_frame_id = params_.base_frame_id;
  }

  // Cache the IK matrix M and its QR decomposition. Static under fixed-theta wheels.
  double sign;
  if (params_.direction == "ccw") {
    sign = 1.0;
  } else if (params_.direction == "cw") {
    sign = -1.0;
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "direction must be 'ccw' or 'cw', got '%s'", params_.direction.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  ik_M_.resize(static_cast<Eigen::Index>(wheel_count_), 3);
  const double inv_r = sign / params_.wheel_radius;
  for (std::size_t i = 0; i < wheel_count_; ++i) {
    const double x = params_.wheel_positions_x[i];
    const double y = params_.wheel_positions_y[i];
    const double th = params_.wheel_angles[i];
    const double c = std::cos(th);
    const double s = std::sin(th);
    ik_M_(static_cast<Eigen::Index>(i), 0) = c * inv_r;
    ik_M_(static_cast<Eigen::Index>(i), 1) = s * inv_r;
    ik_M_(static_cast<Eigen::Index>(i), 2) = (x * s - y * c) * inv_r;
  }
  ik_qr_.compute(ik_M_);

  // Diff-drive style geometries (e.g. 2 wheels with theta=0) yield rank < 3: some
  // Cartesian DoF is unobservable in odom and infeasible in IK.
  if (ik_qr_.rank() < 3) {
    // Probe which Cartesian axis (vx, vy, wz) projects to zero on M, to name the lost DoF.
    constexpr const char * axis_names[3] = {"vx", "vy", "wz"};
    const double tol = 1e-9;
    std::string lost;
    for (int a = 0; a < 3; ++a) {
      Eigen::Vector3d e = Eigen::Vector3d::Zero();
      e(a) = 1.0;
      if ((ik_M_ * e).norm() < tol) {
        if (!lost.empty()) {lost += ", ";}
        lost += axis_names[a];
      }
    }
    if (lost.empty()) {lost = "(undetermined)";}
    RCLCPP_WARN(
      get_node()->get_logger(),
      "wheel geometry yields a rank-%d IK matrix (N=%zu, expected rank 3); "
      "DoF unobservable in odom / infeasible in IK: %s",
      static_cast<int>(ik_qr_.rank()), wheel_count_, lost.c_str());
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OmniDriveController::on_activate(const rclcpp_lifecycle::State &)
{
  auto find_cmd = [&](const std::string & full_name, std::size_t & out) {
      auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
        [&](const auto & h) {return h.get_name() == full_name;});
      if (it == command_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "missing command interface: %s", full_name.c_str());
        return false;
      }
      out = static_cast<std::size_t>(std::distance(command_interfaces_.begin(), it));
      return true;
    };
  auto find_state = [&](const std::string & full_name, std::size_t & out) {
      auto it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
        [&](const auto & h) {return h.get_name() == full_name;});
      if (it == state_interfaces_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "missing state interface: %s", full_name.c_str());
        return false;
      }
      out = static_cast<std::size_t>(std::distance(state_interfaces_.begin(), it));
      return true;
    };

  wheel_cmd_idx_.assign(wheel_count_, 0);
  wheel_state_idx_.assign(wheel_count_, 0);

  for (std::size_t i = 0; i < wheel_count_; ++i) {
    if (!find_cmd(
        params_.wheel_joint_names[i] + "/" + hardware_interface::HW_IF_VELOCITY,
        wheel_cmd_idx_[i])) {return controller_interface::CallbackReturn::ERROR;}
    if (!find_state(
        params_.wheel_joint_names[i] + "/" + hardware_interface::HW_IF_VELOCITY,
        wheel_state_idx_[i])) {return controller_interface::CallbackReturn::ERROR;}
  }

  auto zero = std::make_shared<geometry_msgs::msg::TwistStamped>();
  zero->header.stamp = get_node()->now();
  cmd_vel_buffer_.writeFromNonRT(zero);

  last_vx_ = 0.0;
  last_vy_ = 0.0;
  last_wz_ = 0.0;

  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OmniDriveController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto i : wheel_cmd_idx_) {
    (void)command_interfaces_[i].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
OmniDriveController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto cmd_ptr = *cmd_vel_buffer_.readFromRT();
  double vx = 0.0, vy = 0.0, wz = 0.0;
  bool fresh = false;
  if (cmd_ptr) {
    const double age = (time - cmd_ptr->header.stamp).seconds();
    if (age < params_.cmd_vel_timeout) {
      vx = cmd_ptr->twist.linear.x;
      vy = cmd_ptr->twist.linear.y;
      wz = cmd_ptr->twist.angular.z;
      fresh = true;
    }
  }
  if (fresh) {
    const double dt_cmd = period.seconds();
    vx = limiter_x_.limit(vx, last_vx_, dt_cmd);
    vy = limiter_y_.limit(vy, last_vy_, dt_cmd);
    wz = limiter_wz_.limit(wz, last_wz_, dt_cmd);
  }
  last_vx_ = vx;
  last_vy_ = vy;
  last_wz_ = wz;

  // Inverse kinematics.
  const Eigen::Vector3d v(vx, vy, wz);
  const Eigen::VectorXd omegas = ik_M_ * v;
  for (std::size_t i = 0; i < wheel_count_; ++i) {
    (void)command_interfaces_[wheel_cmd_idx_[i]].set_value(omegas(static_cast<Eigen::Index>(i)));
  }

  // Forward kinematics for odometry. Cached QR, only b is rebuilt per tick.
  Eigen::VectorXd b(static_cast<Eigen::Index>(wheel_count_));
  for (std::size_t i = 0; i < wheel_count_; ++i) {
    b(static_cast<Eigen::Index>(i)) =
      state_interfaces_[wheel_state_idx_[i]].get_optional().value_or(0.0);
  }
  const Eigen::Vector3d twist_est = ik_qr_.solve(b);
  const double vx_est = twist_est(0);
  const double vy_est = twist_est(1);
  const double wz_est = twist_est(2);

  const double dt = period.seconds();
  const double cy = std::cos(odom_yaw_);
  const double sy = std::sin(odom_yaw_);
  odom_x_ += (vx_est * cy - vy_est * sy) * dt;
  odom_y_ += (vx_est * sy + vy_est * cy) * dt;
  odom_yaw_ = wrap_to_pi(odom_yaw_ + wz_est * dt);

  if (rt_odom_pub_ && rt_odom_pub_->trylock()) {
    auto & msg = rt_odom_pub_->msg_;
    msg.header.stamp = time;
    msg.pose.pose.position.x = odom_x_;
    msg.pose.pose.position.y = odom_y_;
    msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw_);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.linear.x = vx_est;
    msg.twist.twist.linear.y = vy_est;
    msg.twist.twist.angular.z = wz_est;
    rt_odom_pub_->unlockAndPublish();
  }
  if (rt_tf_pub_ && rt_tf_pub_->trylock()) {
    auto & tf = rt_tf_pub_->msg_.transforms[0];
    tf.header.stamp = time;
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_yaw_);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    rt_tf_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace arena_omni_drive_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arena_omni_drive_controller::OmniDriveController, controller_interface::ControllerInterface)
