// Copyright 2026 dev@voshch.dev (Volodymyr Shcherbyna)
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time.hpp"

#include "arena_omni_drive_controller/omni_drive_controller.hpp"

namespace
{
constexpr double kTolerance = 1e-4;

struct Geometry
{
  std::vector<std::string> joints;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> theta;
  double radius;
};

// 3-omni "kiwi" platform: 3 wheels at 120 deg spacing, rolling tangent to radial.
Geometry kiwi_3omni()
{
  return Geometry{
    {"w1", "w2", "w3"},
    {0.05, -0.025, -0.025},
    {0.0, 0.0433, -0.0433},
    // perpendicular to each wheel's radial direction
    {M_PI_2, M_PI_2 + 2.0 * M_PI / 3.0, M_PI_2 + 4.0 * M_PI / 3.0},
    0.03,
  };
}

// 4-omni cardinal: wheels at front/left/rear/right, rolling tangent.
Geometry cardinal_4omni()
{
  return Geometry{
    {"front", "left", "rear", "right"},
    {0.15, 0.0, -0.15, 0.0},
    {0.0, 0.15, 0.0, -0.15},
    // rolling direction = body-angle + pi/2
    {M_PI_2, M_PI, -M_PI_2, 0.0},
    0.05,
  };
}

// 2-wheel diff-drive geometry. Theta=0 means both wheels roll along +x; vy is unobservable
// in FK and infeasible in IK. M will have rank 2.
Geometry diff_drive()
{
  return Geometry{
    {"left", "right"},
    {0.0, 0.0},
    {0.2, -0.2},
    {0.0, 0.0},
    0.1,
  };
}

class OmniControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void Build(const Geometry & g)
  {
    geom_ = g;
    const std::size_t n = geom_.joints.size();
    wheel_cmd_vals_.assign(n, 0.0);
    wheel_state_vals_.assign(n, 0.0);
    wheel_cmd_handles_.clear();
    wheel_state_handles_.clear();
    for (std::size_t i = 0; i < n; ++i) {
      wheel_cmd_handles_.emplace_back(
        geom_.joints[i], hardware_interface::HW_IF_VELOCITY, &wheel_cmd_vals_[i]);
      wheel_state_handles_.emplace_back(
        geom_.joints[i], hardware_interface::HW_IF_VELOCITY, &wheel_state_vals_[i]);
    }
  }

  controller_interface::return_type Init(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    controller_ = std::make_shared<arena_omni_drive_controller::OmniDriveController>();

    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("wheel_joint_names", geom_.joints),
      rclcpp::Parameter("wheel_positions_x", geom_.x),
      rclcpp::Parameter("wheel_positions_y", geom_.y),
      rclcpp::Parameter("wheel_angles", geom_.theta),
      rclcpp::Parameter("wheel_radius", geom_.radius),
      rclcpp::Parameter("cmd_vel_timeout", 0.5),
      rclcpp::Parameter("use_stamped_vel", false),
      rclcpp::Parameter("publish_odom", false),
      rclcpp::Parameter("publish_tf", false),
      rclcpp::Parameter("odom_frame_id", std::string("odom")),
      rclcpp::Parameter("base_frame_id", std::string("base_link")),
      rclcpp::Parameter(
        "pose_covariance_diagonal",
        std::vector<double>{0.001, 0.001, 1.0e+9, 1.0e+9, 1.0e+9, 0.01}),
      rclcpp::Parameter(
        "twist_covariance_diagonal",
        std::vector<double>{0.001, 0.001, 1.0e+9, 1.0e+9, 1.0e+9, 0.01}),
    };
    for (const auto & p : overrides) {
      auto it = std::find_if(
        params.begin(), params.end(),
        [&](const auto & x) {return x.get_name() == p.get_name();});
      if (it != params.end()) {*it = p;} else {params.push_back(p);}
    }

    rclcpp::NodeOptions opts;
    opts.parameter_overrides(params);

    return controller_->init("test_omni_controller", "", 100, "", opts);
  }

  void spin_for(rclcpp::Node::SharedPtr extra, std::chrono::milliseconds budget)
  {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
      if (extra) {rclcpp::spin_some(extra);}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void ConfigureAndActivate()
  {
    auto node = controller_->get_node();
    ASSERT_EQ(
      node->configure().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    std::vector<hardware_interface::LoanedCommandInterface> cmd_ifaces;
    for (auto & h : wheel_cmd_handles_) {cmd_ifaces.emplace_back(h);}
    std::vector<hardware_interface::LoanedStateInterface> state_ifaces;
    for (auto & h : wheel_state_handles_) {state_ifaces.emplace_back(h);}
    controller_->assign_interfaces(std::move(cmd_ifaces), std::move(state_ifaces));

    ASSERT_EQ(
      node->activate().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  // Publish a Twist and call update() once, returning the commanded wheel speeds.
  std::vector<double> CommandTwist(double vx, double vy, double wz)
  {
    auto pub_node = rclcpp::Node::make_shared("test_pub");
    auto cmd_pub = pub_node->create_publisher<geometry_msgs::msg::Twist>(
      "/test_omni_controller/cmd_vel", rclcpp::QoS(10).reliable());

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;
    cmd_pub->publish(cmd);
    spin_for(nullptr, std::chrono::milliseconds(200));

    const rclcpp::Time t0 = controller_->get_node()->now();
    const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
    EXPECT_EQ(controller_->update(t0, dt), controller_interface::return_type::OK);
    return wheel_cmd_vals_;
  }

  Geometry geom_;
  std::shared_ptr<arena_omni_drive_controller::OmniDriveController> controller_;
  std::vector<double> wheel_cmd_vals_;
  std::vector<double> wheel_state_vals_;
  std::vector<hardware_interface::CommandInterface> wheel_cmd_handles_;
  std::vector<hardware_interface::StateInterface> wheel_state_handles_;
};

// 3-omni IK: per-wheel speed = (cos theta_i*vx + sin theta_i*vy + (x sin - y cos)*wz) / R.
TEST_F(OmniControllerTest, ThreeOmniForwardIK)
{
  Build(kiwi_3omni());
  ASSERT_EQ(Init(), controller_interface::return_type::OK);
  ConfigureAndActivate();

  const auto cmds = CommandTwist(0.1, 0.0, 0.0);
  for (std::size_t i = 0; i < cmds.size(); ++i) {
    const double expected =
      (std::cos(geom_.theta[i]) * 0.1 +
      (geom_.x[i] * std::sin(geom_.theta[i]) - geom_.y[i] * std::cos(geom_.theta[i])) * 0.0) /
      geom_.radius;
    EXPECT_NEAR(cmds[i], expected, kTolerance) << "wheel[" << i << "]";
  }
}

// 3-omni roundtrip: apply IK in software, push as state, FK should recover (vx, vy, wz).
TEST_F(OmniControllerTest, ThreeOmniRoundtripOdom)
{
  Build(kiwi_3omni());
  ASSERT_EQ(
    Init({rclcpp::Parameter("publish_odom", true)}),
    controller_interface::return_type::OK);
  ConfigureAndActivate();

  // Synthesize a non-trivial body twist and the wheel speeds it implies, then push them as state.
  const double vx = 0.05, vy = 0.07, wz = 0.3;
  for (std::size_t i = 0; i < geom_.joints.size(); ++i) {
    const double th = geom_.theta[i];
    wheel_state_vals_[i] =
      (std::cos(th) * vx + std::sin(th) * vy +
      (geom_.x[i] * std::sin(th) - geom_.y[i] * std::cos(th)) * wz) /
      geom_.radius;
  }

  auto sub_node = rclcpp::Node::make_shared("test_sub");
  nav_msgs::msg::Odometry::SharedPtr received;
  auto sub = sub_node->create_subscription<nav_msgs::msg::Odometry>(
    "/test_omni_controller/odometry", rclcpp::SystemDefaultsQoS(),
    [&](nav_msgs::msg::Odometry::SharedPtr msg) {received = msg;});
  spin_for(sub_node, std::chrono::milliseconds(200));

  const rclcpp::Time t0 = controller_->get_node()->now();
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
  EXPECT_EQ(controller_->update(t0, dt), controller_interface::return_type::OK);

  for (int i = 0; i < 50 && !received; ++i) {
    rclcpp::spin_some(sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(received, nullptr);
  EXPECT_NEAR(received->twist.twist.linear.x, vx, kTolerance);
  EXPECT_NEAR(received->twist.twist.linear.y, vy, kTolerance);
  EXPECT_NEAR(received->twist.twist.angular.z, wz, kTolerance);
}

// 4-omni cardinal: a pure +vy command should engage the left and right wheels symmetrically
// and leave the front/rear wheels untouched (their rolling axes are along Y).
TEST_F(OmniControllerTest, FourOmniLateralOnlyDrivesLeftRight)
{
  Build(cardinal_4omni());
  ASSERT_EQ(Init(), controller_interface::return_type::OK);
  ConfigureAndActivate();

  const auto cmds = CommandTwist(0.0, 0.2, 0.0);
  // Joints: [front, left, rear, right] with theta = [pi/2, pi, -pi/2, 0].
  // sin(pi/2)*0.2 / R for front, sin(pi)*0.2 / R for left, etc.
  EXPECT_NEAR(cmds[0], 0.2 / geom_.radius, kTolerance);   // front
  EXPECT_NEAR(cmds[1], 0.0, kTolerance);                  // left (sin pi = 0)
  EXPECT_NEAR(cmds[2], -0.2 / geom_.radius, kTolerance);  // rear (sin -pi/2 = -1)
  EXPECT_NEAR(cmds[3], 0.0, kTolerance);                  // right (sin 0 = 0)
}

// Diff-drive geometry: theta=0, wheels on the y axis. Pure vx gives equal positive wheel
// speeds, pure vy projects to zero (rank-2 matrix), pure yaw splits left vs right.
TEST_F(OmniControllerTest, DiffDriveProjectsLateralToZero)
{
  Build(diff_drive());
  ASSERT_EQ(Init(), controller_interface::return_type::OK);
  ConfigureAndActivate();

  const auto fwd = CommandTwist(0.5, 0.0, 0.0);
  EXPECT_NEAR(fwd[0], 0.5 / geom_.radius, kTolerance);
  EXPECT_NEAR(fwd[1], 0.5 / geom_.radius, kTolerance);

  const auto sideways = CommandTwist(0.0, 0.5, 0.0);
  EXPECT_NEAR(sideways[0], 0.0, kTolerance);
  EXPECT_NEAR(sideways[1], 0.0, kTolerance);

  // Pure yaw: wheel separation 0.4 m, theta=0, x=0 -> wheel speed = (-y) * wz / R = -+0.2*wz/R.
  const auto yaw = CommandTwist(0.0, 0.0, 1.0);
  EXPECT_NEAR(yaw[0], -0.2 * 1.0 / geom_.radius, kTolerance);
  EXPECT_NEAR(yaw[1], 0.2 * 1.0 / geom_.radius, kTolerance);
}

// No cmd_vel ever sent: the zero stamp from on_activate ages past cmd_vel_timeout and update
// emits a zero twist. Wheels stay at zero.
TEST_F(OmniControllerTest, ZeroAfterTimeout)
{
  Build(kiwi_3omni());
  ASSERT_EQ(Init(), controller_interface::return_type::OK);
  ConfigureAndActivate();

  const rclcpp::Time start = controller_->get_node()->now();
  const rclcpp::Time late = start + rclcpp::Duration::from_seconds(10.0);
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.02);
  EXPECT_EQ(controller_->update(late, dt), controller_interface::return_type::OK);
  for (auto v : wheel_cmd_vals_) {
    EXPECT_NEAR(v, 0.0, kTolerance);
                                                                  }
}

// Cross-field length mismatch must fail configure cleanly.
TEST_F(OmniControllerTest, LengthMismatchFailsConfigure)
{
  Build(kiwi_3omni());
  // Override wheel_positions_x with the wrong length (2 entries for a 3-wheel geometry).
  ASSERT_EQ(
    Init({rclcpp::Parameter("wheel_positions_x", std::vector<double>{0.0, 0.0})}),
    controller_interface::return_type::OK);

  auto node = controller_->get_node();
  EXPECT_NE(
    node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
