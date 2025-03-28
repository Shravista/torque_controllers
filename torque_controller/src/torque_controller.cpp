#include "torque_controller/torque_controller.hpp"
#include "rclcpp/logging.hpp"
#include "controller_interface/helpers.hpp"
#include "rclcpp/qos.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#define WARN(method, var) RCLCPP_WARN_STREAM(get_node()->get_logger(), "[" << method << "] " << #var " = " << var);
#define DEBUG(method, var) RCLCPP_DEBUG_STREAM(get_node()->get_logger(), "[" << method << "] " << #var " = " << var);


namespace torque_controller{

TorqueController::TorqueController() : controller_interface::ControllerInterface(), 
                    _rt_torque_command_ptr(nullptr),
                    _torque_commands_subs(nullptr){
    _interface_name = hardware_interface::HW_IF_EFFORT;
    
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

    // robot description
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();

    // pinocchio
    try{
        auto robot_description = result[0].value_to_string();
        pinocchio::urdf::buildModelFromXML(robot_description, _model);
        _data = pinocchio::Data(_model);
    } catch (std::exception& e){
        RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown during initial stage with :\n" <<
                                                        e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    // initialize states dimensions
    _q = Eigen::VectorXd::Zero(_params.joints.size());
    _qdot = Eigen::VectorXd::Zero(_params.joints.size());

    // subscriber
    _torque_commands_subs = get_node()->create_subscription<torqueCmd>(
        "~/torque", rclcpp::SystemDefaultsQoS(),
         [this](const torqueCmd::SharedPtr msg) {
            _rt_torque_command_ptr.writeFromNonRT(msg);}
    );
    RCLCPP_INFO(get_node()->get_logger(), "Configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

void TorqueController::declare_parameters(){
    _param_listener = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn TorqueController::read_parameters(){
    if (!_param_listener){
        RCLCPP_ERROR(get_node()->get_logger(), "[read_parameters] Error encountered during init ");
        return controller_interface::CallbackReturn::ERROR;
    }
    _params = _param_listener->get_params();
    if(_params.joints.empty()){
        RCLCPP_ERROR(get_node()->get_logger(), "[read_parameters] 'joints' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    if (_params.K.size() != _params.joints.size()){
        RCLCPP_ERROR(get_node()->get_logger(), "[read parameters] 'K' parameter size (%zu) is not same as size of 'joint's' size (%zu)", _params.K.size(), _params.joints.size());
        return controller_interface::CallbackReturn::ERROR;
    }
    _K = (Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>
            (_params.K.data(), _params.K.size())).asDiagonal();
    for (const auto &joint: _params.joints)
        _command_interface_types.push_back(joint+"/"+_interface_name);
    for (const auto &joint: _params.joints)
        _state_interface_types.push_back(joint+"/position");
    for (const auto &joint: _params.joints)
        _state_interface_types.push_back(joint+"/velocity");
    
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TorqueController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration cmd_interfaces_cfg;
    cmd_interfaces_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cmd_interfaces_cfg.names =  _command_interface_types;
    
    return cmd_interfaces_cfg;
}

controller_interface::InterfaceConfiguration TorqueController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration state_interfaces_cfg;
    state_interfaces_cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_cfg.names = _state_interface_types;
    return state_interfaces_cfg;
}

controller_interface::CallbackReturn TorqueController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){
    
    /** This is critical as of now and will be changed in future
     * subjected to it capability of real time and deterministic
     * cycles for practical applications
     */
    
    if(!controller_interface::get_ordered_interfaces(
        command_interfaces_, _command_interface_types, std::string(""), _cmd_interfaces) ||
        _command_interface_types.size() !=_cmd_interfaces.size()){
            RCLCPP_ERROR(get_node()->get_logger(), "[on_activate] Expected %zu command interfaces, got %zu", 
            _command_interface_types.size(), _cmd_interfaces.size());
            return controller_interface::CallbackReturn::ERROR;
    }
    
    
    if(!controller_interface::get_ordered_interfaces(
        state_interfaces_, _state_interface_types, std::string(""), _state_interfaces) ||
        _state_interface_types.size() != _state_interfaces.size()){
            RCLCPP_ERROR(get_node()->get_logger(), " [on_activate] Expected %zu state interfaces, got %zu", 
            _state_interface_types.size(), _state_interfaces.size());
            return controller_interface::CallbackReturn::ERROR;
    }
    _rt_torque_command_ptr = realtime_tools::RealtimeBuffer<std::shared_ptr<torqueCmd>>(nullptr);
    RCLCPP_INFO(get_node()->get_logger(), "Activate Successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueController::on_deactivate(const rclcpp_lifecycle ::State& /*previous_state*/){
    this->sendStaticInput();

    // reset the command buffer
    _rt_torque_command_ptr = realtime_tools::RealtimeBuffer<std::shared_ptr<torqueCmd>>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TorqueController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /* duration*/){
    auto joint_commands = _rt_torque_command_ptr.readFromRT();
    this->read_parameters();
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
        this->sendStaticInput();
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

void TorqueController::sendStaticInput(){
    auto sz = command_interfaces_.size();
    // get states
    for (auto index= 0ul; index < sz; index++)
        _q[index] = state_interfaces_[index].get_value();
    for (auto index= 0ul; index < sz; index++)
        _qdot[index] = state_interfaces_[sz+index].get_value();

    // compute the system matrices
    pinocchio::crba(_model, _data, _q);
    pinocchio::computeCoriolisMatrix(_model, _data, _q, _qdot);
    pinocchio::computeGeneralizedGravity(_model, _data, _q);
    _data.M.triangularView<Eigen::StrictlyLower>() = 
                    _data.M.transpose().triangularView<Eigen::StrictlyLower>();
    
    // compute control inputs based on inverse dynamics with current state (position) as target and zero velocity
    _u_static = -_data.M*_K*_qdot + _data.C*_qdot + _data.g;
    for (auto index = 0ul; index < sz; ++index)
        command_interfaces_[index].set_value(_u_static[index]);
}
} // torque_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(torque_controller::TorqueController, controller_interface::ControllerInterface)
