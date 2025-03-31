#pragma once

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "realtime_tools/realtime_buffer.hpp"
#include "torque_msgs/msg/commands.hpp"
#include "rclcpp/subscription.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_torque_controller {

using torqueCmd = torque_msgs::msg::Commands;

class FrankaTorqueController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
 private:
  std::string _arm_id;
  const int _nDof = 7;
  Vector7d _q, _qdot;
  realtime_tools::RealtimeBuffer<std::shared_ptr<torqueCmd>> _rt_torque_command_ptr;
  rclcpp::Subscription<torqueCmd>::SharedPtr _torque_commands_subs;
  double _alpha = 0.99;

  void updateJointStates();
};
}  // namespace franka_torque_controller
