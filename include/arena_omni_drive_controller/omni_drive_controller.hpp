// Copyright 2026 dev@voshch.dev (Volodymyr Shcherbyna)
// SPDX-License-Identifier: Apache-2.0

#ifndef ARENA_OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_
#define ARENA_OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_

#include <Eigen/Dense>

#include <memory>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <arena_omni_drive_controller/arena_omni_drive_controller_parameters.hpp>

namespace arena_omni_drive_controller
{

class OmniDriveController : public controller_interface::ControllerInterface
{
public:
  OmniDriveController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Wheel count N derived from params_.wheel_joint_names.size() in on_configure.
  std::size_t wheel_count_{0};

  // Indices into command_interfaces_ / state_interfaces_, parallel to params_.wheel_joint_names.
  std::vector<std::size_t> wheel_cmd_idx_;
  std::vector<std::size_t> wheel_state_idx_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::TwistStamped>> cmd_vel_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tf_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> rt_tf_pub_;

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};

  // IK matrix M: row i = [cos theta_i, sin theta_i, x_i sin theta_i - y_i cos theta_i] / R.
  // Built once in on_configure from static params, reused every tick. QR decomposition cached
  // for the least-squares FK solve.
  Eigen::MatrixXd ik_M_;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> ik_qr_;
};

}  // namespace arena_omni_drive_controller

#endif  // ARENA_OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_
