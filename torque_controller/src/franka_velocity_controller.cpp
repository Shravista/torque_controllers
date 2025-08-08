#include <torque_controller/franka_velocity_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>

#include <eigen3/Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/robot_utils.hpp>

namespace franka_velocity_controller {


controller_interface::InterfaceConfiguration
FrankaVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= _nDof; ++i) {
    config.names.push_back(_arm_id + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
FrankaVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= _nDof; ++i) {
    config.names.push_back(_arm_id + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(_arm_id + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type FrankaVelocityController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  auto joint_commands = _rt_torque_command_ptr.readFromRT();
  // RCLCPP_INFO_STREAM(get_node()->get_logger(), "Time: " << time.nanoseconds() << " duration: " << duration.seconds());

  /**
   * if no control is received hold the position using the inverse dynamics control
   * implementing this method depend on the how well your model parameters is known.
   * if no command is received hold the position using inverse dynamics cont.
   * else external agent is respnsible to send the sensible torque so that system is stable
   */
  auto sz = command_interfaces_.size();
  // WARN("update", _torque_commands_subs->get_publisher_count())
  if (!joint_commands || !(*joint_commands) || (_torque_commands_subs->get_publisher_count() == 0)){
      RCLCPP_DEBUG(get_node()->get_logger(), "No command received, holding position");
      for (auto index = 0ul; index < sz; ++index)
          command_interfaces_[index].set_value(0.0);
  } else{
      RCLCPP_DEBUG(get_node()->get_logger(), "Command received, executing");
      if ((*joint_commands)->commands.size() != sz){
          RCLCPP_ERROR_THROTTLE(
              get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                "command size(%zu) does not match number of interfaces (%zu)",
                (*joint_commands)->commands.size(), sz);
          return controller_interface::return_type::ERROR;
      }
      for (auto index = 0ul; index < sz; ++index)
          command_interfaces_[index].set_value((*joint_commands)->commands[index]);
  }
  return controller_interface::return_type::OK;
}

CallbackReturn FrankaVelocityController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  _arm_id = "fr3";
  is_gazebo = true;//get_node()->get_parameter("gazebo").as_bool();
  _q = Eigen::VectorXd::Zero(_nDof);
  _qdot = Eigen::VectorXd::Zero(_nDof);
  // subscriber
  _torque_commands_subs = get_node()->create_subscription<velocityCmd>(
    "~/velocity", rclcpp::SystemDefaultsQoS(),
     [this](const velocityCmd::SharedPtr msg) {
        _rt_torque_command_ptr.writeFromNonRT(msg);}
  );
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  _arm_id = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
  RCLCPP_WARN_STREAM(get_node()->get_logger(), "is gazebo " << std::boolalpha << is_gazebo);
  if (!is_gazebo) {
    auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");
    auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

    auto future_result = client->async_send_request(request);
    future_result.wait_for(robot_utils::time_out);

    auto success = future_result.get();
    if (!success) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaVelocityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  _rt_torque_command_ptr = realtime_tools::RealtimeBuffer<std::shared_ptr<velocityCmd>>(nullptr);
  // for (auto it: command_interfaces_)

  RCLCPP_INFO(get_node()->get_logger(), "Activate Successful");
  return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn FrankaVelocityController::on_deactivate(const rclcpp_lifecycle ::State& /*previous_state*/){

  // reset the command buffer
  _rt_torque_command_ptr = realtime_tools::RealtimeBuffer<std::shared_ptr<velocityCmd>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

void FrankaVelocityController::updateJointStates() {
  for (auto i = 0; i < _nDof; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    _q(i) = position_interface.get_value();
    _qdot(i) = (1-_alpha)*_qdot(i) + _alpha*velocity_interface.get_value();
  }
}
}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_velocity_controller::FrankaVelocityController,
                       controller_interface::ControllerInterface)
