#include "torque_controller/torque_controller.hpp"
#include "rclcpp/logging.hpp"

namespace torque_controller{

TorqueController::TorqueController() : controller_interface::ControllerInterface(), 
                    _rt_torque_command_ptr(nullptr),
                    _torque_commands_subs(nullptr){

}

controller_interface::CallbackReturn TorqueController::on_init(){
    try{
        declare_parameters();
    } catch (const std::exception& e){
        RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown during initial stage with :\n" <<
                                                        e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueController::on_configure(const rclcpp_lifecycle::State& previous_state){
    auto ret = this->read_parameters();
    if (ret != controller_interface::CallbackReturn::SUCCESS)
        return ret;

    _torque_commands_subs = get_node()->create_subscription<torqueCmd>(
        "~/torque", rclcpp::SystemDefaultsQoS(),
         [this](const torqueCmd::SharedPtr msg) {_rt_torque_command_ptr.writeFromNonRT(msg);}
    );
    RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

void TorqueController::declare_parameters(){
    _param_listener = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn TorqueController::read_parameters(){
    if (_param_listener){
        RCLCPP_ERROR(get_node()->get_logger(), "[read_parameters] Error encountered during init ");
        return controller_interface::CallbackReturn::ERROR;
    }
    _params = _param_listener->get_params();
    if(_params.joints.empty()){
        RCLCPP_ERROR(get_node()->get_logger(), "[read_parameters] 'joints' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }
    if(_params.interface_name.empty()){
        RCLCPP_ERROR(get_node()->get_logger(), "[read_parameters] 'interface_name' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    for (const auto &joint: _params.joints)
        _command_interface_types.push_back(joint+"/"+_params.interface_name);
    
    return controller_interface::CallbackReturn::SUCCESS;
}
} // torque_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(torque_controller::TorqueController, controller_interface::ControllerInterface)
