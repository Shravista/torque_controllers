#include "torque_controller/torque_controller.hpp"
#include "rclcpp/logging.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/qos.hpp"

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

controller_interface::CallbackReturn TorqueController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/){
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

controller_interface::InterfaceConfiguration TorqueController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration cmd_interfaces_cfg;
    cmd_interfaces_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cmd_interfaces_cfg.names =  _command_interface_types;
    return cmd_interfaces_cfg;
}

controller_interface::InterfaceConfiguration TorqueController::state_interface_configuration() const{
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn TorqueController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
    /** This is critical as of now and will be changed in future
     * subjected to it capability of real time and deterministic
     * cycles for practical applications
     */
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
    if(!controller_interface::get_ordered_interfaces(
        command_interfaces_, _command_interface_types, std::string(""), ordered_interfaces) ||
        _command_interface_types.size() !=ordered_interfaces.size()){
            RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu command interfaces, got %zu", 
            _command_interface_types.size(), ordered_interfaces.size());
            return controller_interface::CallbackReturn::ERROR;
        }
    _rt_torque_command_ptr = realtime_tools::RealtimeBuffer<std::shared_ptr<torqueCmd>>(nullptr);
    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueController::on_deactivate(const rclcpp_lifecycle ::State& /*previous_state*/){
    // reset the command buffer
    _rt_torque_command_ptr = realtime_tools::RealtimeBuffer<std::shared_ptr<torqueCmd>>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TorqueController::update(const rclcpp::Time& time, const rclcpp::Duration& duration){
    return controller_interface::return_type::OK;
}
} // torque_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(torque_controller::TorqueController, controller_interface::ControllerInterface)
