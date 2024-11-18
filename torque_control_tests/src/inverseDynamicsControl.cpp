#include "torque_control_tests/inverseDynamicsControl.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

InverseDynamicsControl::InverseDynamicsControl(std::string robot_name) 
    : Node("inverseDynamicsController"), _robot_name(robot_name){
    declareParams();

    _pub = this->create_publisher<std_msgs::msg::Float64MultiArray>
           (this->get_parameter(_robot_name+".controller_name").as_string()+"/commands",10);
    _subs = this->create_subscription<sensor_msgs::msg::JointState>
            ("/joint_states", 10, std::bind(&InverseDynamicsControl::callback, this, _1));
    _fileName = ament_index_cpp::get_package_share_directory
                (this->get_parameter(_robot_name+".package_name").as_string()) + "/" +
                 this->get_parameter(_robot_name+".relative_path").as_string() + "/" +
                 this->get_parameter(_robot_name+".description").as_string();
    RCLCPP_WARN_STREAM(this->get_logger(), "fileName = " << _fileName);
    // pinocchio
    pinocchio::urdf::buildModel(_fileName, _model);
    _data = pinocchio::Data(_model);
    
    _q = pinocchio::randomConfiguration(_model);
    _qdot = Eigen::VectorXd::Zero(_q.size());
    _qDes = Eigen::VectorXd::Zero(_q.size());
    
    getGains();

    // needed for all callbacks have been called
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(5ms);

    // need for dynamics callback for parameters
    auto timer_callback = [this](){
        this->getGains();
    };
    _timer = this->create_wall_timer(1ms, timer_callback);
    RCLCPP_INFO_STREAM(this->get_logger(), "Node " << this->get_name() << " is initialized succesfully");
}

InverseDynamicsControl::InverseDynamicsControl(std::string robot_name, std::string version) 
    : Node("inverseDynamicsController"+version), _robot_name(robot_name){
    declareParams();

    _commander = this->create_publisher<torque_msgs::msg::Commands>
           (this->get_parameter(_robot_name+".controller_name").as_string()+"/torque",10);
    _subs = this->create_subscription<sensor_msgs::msg::JointState>
            ("/joint_states", 10, std::bind(&InverseDynamicsControl::callback, this, _1));
    _fileName = ament_index_cpp::get_package_share_directory
                (this->get_parameter(_robot_name+".package_name").as_string()) + "/" +
                 this->get_parameter(_robot_name+".relative_path").as_string() + "/" +
                 this->get_parameter(_robot_name+".description").as_string();
    RCLCPP_WARN_STREAM(this->get_logger(), "fileName = " << _fileName);
    // pinocchio
    pinocchio::urdf::buildModel(_fileName, _model);
    _data = pinocchio::Data(_model);
    
    _q = pinocchio::randomConfiguration(_model);
    _qdot = Eigen::VectorXd::Zero(_q.size());
    _qDes = Eigen::VectorXd::Zero(_q.size());
    
    getGains();

    // needed for all callbacks have been called
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(5ms);

    // need for dynamics callback for parameters
    auto timer_callback = [this](){
        this->getGains();
    };
    _timer = this->create_wall_timer(1ms, timer_callback);
    RCLCPP_INFO_STREAM(this->get_logger(), "Node " <<  this->get_name() << " is initialized succesfully with version " << version);
}


void InverseDynamicsControl::declareParams(){
    std::vector<double> val;
    this->declare_parameter(_robot_name+".package_name", "example_robots");
    this->declare_parameter(_robot_name+".relative_path", "test");
    this->declare_parameter(_robot_name+".description", "iiwa14.urdf");
    this->declare_parameter(_robot_name+".controller_name", "test_controller");
    this->declare_parameter(_robot_name+".kp", val);
    this->declare_parameter(_robot_name+".kd", val);
}

void InverseDynamicsControl::getGains(){
    auto Kp = this->get_parameter(_robot_name+".kp").as_double_array();
    auto Kd = this->get_parameter(_robot_name+".kd").as_double_array();
    _Kp = (Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(Kp.data(), Kp.size())).asDiagonal();
    _Kd = (Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(Kd.data(), Kd.size())).asDiagonal();

}  

void InverseDynamicsControl::callback(sensor_msgs::msg::JointState::SharedPtr msg){
    _state = msg;
    _q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_state->position.data(), _state->position.size());
    _qdot = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_state->velocity.data(), _state->velocity.size());
}

void InverseDynamicsControl::run(Eigen::VectorXd target){
    _qDes = target;
    Eigen::VectorXd val(_q.size()), u(_q.size());
    _msg.data.resize(_q.size());
    while (rclcpp::ok()){
        rclcpp::spin_some(this->get_node_base_interface());

        RCLCPP_INFO(this->get_logger(), "Running");
        // compute the system matrices
        pinocchio::crba(_model, _data, _q);
        pinocchio::computeCoriolisMatrix(_model, _data, _q, _qdot);
        pinocchio::computeGeneralizedGravity(_model, _data, _q);

        _data.M.triangularView<Eigen::StrictlyLower>() = 
                        _data.M.transpose().triangularView<Eigen::StrictlyLower>();

        // compute the control input
        val = _Kp*(_qDes -_q) + _Kd*(-_qdot);

        u = _data.M*val + _data.C*_qdot + _data.g;

        Eigen::Map<Eigen::VectorXd>(_msg.data.data(), _msg.data.size()) = u;                

        _pub->publish(_msg);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Message Checking " << sensor_msgs::msg::to_yaml(*_state));
        rclcpp::sleep_for(1ms);
    }
}

void InverseDynamicsControl::pdGravityControl(Eigen::VectorXd target){
    _qDes = target;
    Eigen::VectorXd val(_q.size()), u(_q.size());
    _msg.data.resize(_q.size());
    while (rclcpp::ok()){
        rclcpp::spin_some(this->get_node_base_interface());

        RCLCPP_INFO(this->get_logger(), "Running");
        pinocchio::computeGeneralizedGravity(_model, _data, _q);

        // compute the control input
        u = _Kp*(_qDes -_q) + _Kd*(-_qdot) + _data.g;

        Eigen::Map<Eigen::VectorXd>(_msg.data.data(), _msg.data.size()) = u;                

        _pub->publish(_msg);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Message Checking " << sensor_msgs::msg::to_yaml(*_state));
        rclcpp::sleep_for(1ms);
    }
}

void InverseDynamicsControl::gravityCompensation(){
    Eigen::VectorXd val(_q.size()), u(_q.size());
    _msg.data.resize(_q.size());
    while (rclcpp::ok()){
        rclcpp::spin_some(this->get_node_base_interface());

        RCLCPP_INFO(this->get_logger(), "Running");
        pinocchio::computeGeneralizedGravity(_model, _data, _q);

        // compute the control input
        u = _data.g;

        Eigen::Map<Eigen::VectorXd>(_msg.data.data(), _msg.data.size()) = u;                

        _pub->publish(_msg);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Message Checking " << sensor_msgs::msg::to_yaml(*_state));
        rclcpp::sleep_for(1ms);
    }
}

void InverseDynamicsControl::run2(Eigen::VectorXd target){
    _qDes = target;
    Eigen::VectorXd val(_q.size()), u(_q.size());
    _tau.commands.resize(_q.size());
    while (rclcpp::ok()){
        rclcpp::spin_some(this->get_node_base_interface());

        RCLCPP_INFO(this->get_logger(), "Running");
        // compute the system matrices
        pinocchio::crba(_model, _data, _q);
        pinocchio::computeCoriolisMatrix(_model, _data, _q, _qdot);
        pinocchio::computeGeneralizedGravity(_model, _data, _q);

        _data.M.triangularView<Eigen::StrictlyLower>() = 
                        _data.M.transpose().triangularView<Eigen::StrictlyLower>();

        // compute the control input
        val = _Kp*(_qDes -_q) + _Kd*(-_qdot);

        u = _data.M*val + _data.C*_qdot + _data.g;

        Eigen::Map<Eigen::VectorXd>(_tau.commands.data(), _tau.commands.size()) = u;                

        _commander->publish(_tau);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Message Checking " << sensor_msgs::msg::to_yaml(*_state));
        rclcpp::sleep_for(1ms);
    }
}

void InverseDynamicsControl::pdGravityControl2(Eigen::VectorXd target){
    _qDes = target;
    Eigen::VectorXd val(_q.size()), u(_q.size());
    _tau.commands.resize(_q.size());
    while (rclcpp::ok()){
        rclcpp::spin_some(this->get_node_base_interface());

        RCLCPP_INFO(this->get_logger(), "Running");
        pinocchio::computeGeneralizedGravity(_model, _data, _q);

        // compute the control input
        u = _Kp*(_qDes -_q) + _Kd*(-_qdot) + _data.g;

        Eigen::Map<Eigen::VectorXd>(_tau.commands.data(), _tau.commands.size()) = u;                

        _commander->publish(_tau);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Message Checking " << sensor_msgs::msg::to_yaml(*_state));
        rclcpp::sleep_for(1ms);
    }
}

void InverseDynamicsControl::gravityCompensation2(){
    Eigen::VectorXd val(_q.size()), u(_q.size());
    _tau.commands.resize(_q.size());
    while (rclcpp::ok()){
        rclcpp::spin_some(this->get_node_base_interface());

        RCLCPP_INFO(this->get_logger(), "Running");
        pinocchio::computeGeneralizedGravity(_model, _data, _q);

        // compute the control input
        u = _data.g;

        Eigen::Map<Eigen::VectorXd>(_tau.commands.data(), _tau.commands.size()) = u;                

        _commander->publish(_tau);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Message Checking " << sensor_msgs::msg::to_yaml(*_state));
        rclcpp::sleep_for(1ms);
    }
}
