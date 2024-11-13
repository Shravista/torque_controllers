#ifndef _TORQUE_CONTROLLER_HPP_
#define _TORQUE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "forward_command_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace torque_controller{
// using torqueCmd ;

class TorqueController : public controller_interface::ControllerInterface{

};
}

#endif //_TORQUE_CONTROLLER_HPP_