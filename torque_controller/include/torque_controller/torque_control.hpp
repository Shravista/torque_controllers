#ifndef _TORQUE_CONTROLLER_HPP_
#define _TORQUE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "torque_controller/visibility_controller.h"
#include "torque_msgs/msg/commands.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace torque_controller{
using torqueCmd = torque_msgs::msg::Commands;

class TorqueController : public controller_interface::ControllerInterface{
    public:
        TORQUE_CONTROLLER_PUBLIC TorqueController();
        TORQUE_CONTROLLER_PUBLIC ~TorqueController() = default;

        TORQUE_CONTROLLER_PUBLIC 
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        TORQUE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        TORQUE_CONTROLLER_PUBLIC
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    
    protected:
        virtual void declare_parameters() = 0;
        virtual controller_interface::CallbackReturn read_parameters() = 0;

        std::vector<std::string> _joint_names;
        std::string _interface_name;

        std::vector<std::string> _command_interface_types;
        realtime_tools::RealtimeBuffer<std::shared_ptr<torqueCmd>> _rt_torque_command_ptr;
        rclcpp::Subscription<torqueCmd>::SharedPtr _torque_commands_subs;
};
} // torque_controller

#endif //_TORQUE_CONTROLLER_HPP_