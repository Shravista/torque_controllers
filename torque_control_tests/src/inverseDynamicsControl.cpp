#include "torque_control_tests/inverseDynamicsControl.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"
#define PRINT(var) std::cout << #var " = " << var << std::endl;

InverseDynamicsControl::InverseDynamicsControl(std::string name, std::vector<std::string> joint_names) 
    : Node(name), _joint_names(joint_names){
        
    _q      = Eigen::VectorXd::Zero(7);
    _qdot   = Eigen::VectorXd::Zero(7);
    _qDes   = Eigen::VectorXd::Zero(7);
    _qdDes  = Eigen::VectorXd::Zero(7);
    _qddDes = Eigen::VectorXd::Zero(7);
    declareParams();

    _commander = this->create_publisher<torque_msgs::msg::Commands>
           (this->get_parameter("controller_name").as_string()+"/torque",10);
    _subs = this->create_subscription<sensor_msgs::msg::JointState>
            ("/joint_states", 10, std::bind(&InverseDynamicsControl::callback, this, _1));
    
    // robot description
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client = 
    this->create_client<rcl_interfaces::srv::GetParameters>("/robot_state_publisher/get_parameters");
    while(!client->wait_for_service(1s)){
        if (!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "Service Interrupted");
            break;
        } 
        RCLCPP_INFO(this->get_logger(), "service not available and waited for 1 s");
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("robot_description");
    
    auto result = client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "msg received success\n");
    } else{
        RCLCPP_ERROR(this->get_logger(), "Not retrieved urdf") ;
    }
    auto val  = result.get()->values;
    auto urdf = val[0].string_value;

    // pinocchio
    pinocchio::Model tmp;
    pinocchio::urdf::buildModelFromXML(urdf, tmp);
    auto q = pinocchio::neutral(tmp);
    // joints to lock
    std::vector<std::string> lock_joints{"fr3_finger_joint1", "fr3_finger_joint2"};
    std::vector<pinocchio::JointIndex> lock_joint_ids;
    for (auto it = lock_joints.begin(); it!= lock_joints.end(); it++){
        if (tmp.existJointName(*it))
            lock_joint_ids.push_back(tmp.getJointId(*it));
    }
    _model = pinocchio::buildReducedModel(tmp, lock_joint_ids, q);
    _data = pinocchio::Data(_model);

    getGains();

    // needed for all callbacks have been called
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(5ms);

    // needed for dynamics callback for parameters
    auto timer_callback = [this](){
        this->getGains();
    };
    _timer = this->create_wall_timer(1ms, timer_callback);
    RCLCPP_INFO_STREAM(this->get_logger(), "Node " <<  this->get_name() << " is initialized succesfully");
}


void InverseDynamicsControl::declareParams(){
    std::vector<double> val;
    this->declare_parameter("controller_name", "test_controller");
    this->declare_parameter("kp", val);
    this->declare_parameter("kd", val);
    this->declare_parameter("amplitude", 1.0);
}

void InverseDynamicsControl::getGains(){
    auto Kp = this->get_parameter("kp").as_double_array();
    auto Kd = this->get_parameter("kd").as_double_array();
    _Kp = (Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(Kp.data(), Kp.size())).asDiagonal();
    _Kd = (Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(Kd.data(), Kd.size())).asDiagonal();
    // RCLCPP_INFO(this->get_logger(), "Update params called");
}  

void InverseDynamicsControl::callback(sensor_msgs::msg::JointState::SharedPtr msg){
    _state = msg;
    for (size_t ii = 0; ii < 7; ii++){
        /**
         * In ROS 2 humble and Gazebo Ignition there is a bug in the joint_state publisher I guess,
         * which lead to  have disordered array of joints in the /joint_states topic.
         * Thus, the reason one requires the following code.
         */
        auto idx = findID(_state->name, _joint_names.at(ii));
        // std::cout << "in call back _q.size() = " << _q.size() ;
        // std::cout << " ii = " << ii << " idx = " << idx << " val = " << _state->position.at(idx) << std::endl;
        _q(ii) = _state->position.at(idx);
        _qdot(ii) = (1-alpha)*_qdot(ii) + alpha*_state->velocity.at(idx);
    }
}

void InverseDynamicsControl::run(Eigen::VectorXd target){
    _qDes = target;
    Eigen::VectorXd val(_q.size()), u(_q.size());
    _tau.commands.resize(_q.size());
    while (rclcpp::ok()){
        rclcpp::spin_some(this->get_node_base_interface());
        getGains();

        // compute the system matrices
        auto tmp = pinocchio::crba(_model, _data, _q);
        pinocchio::computeCoriolisMatrix(_model, _data, _q, _qdot);
        // pinocchio::computeGeneralizedGravity(_model, _data, _q);
        _data.M.triangularView<Eigen::StrictlyLower>() = 
                    _data.M.transpose().triangularView<Eigen::StrictlyLower>();
        
        // compute the control input
        val = _Kp*(_qDes -_q) + _Kd*(-_qdot);
        u = _data.M*val + _data.C*_qdot;// + _data.g;

        Eigen::Map<Eigen::VectorXd>(_tau.commands.data(), _tau.commands.size()) = u;
        RCLCPP_INFO_STREAM(this->get_logger(), "e = [" << (_qDes -_q).transpose() << "]");
        _commander->publish(_tau);
        RCLCPP_INFO_STREAM(this->get_logger(), "u = " << u.transpose());
        rclcpp::sleep_for(1ms);
    }
}

void InverseDynamicsControl::run(Eigen::VectorXd qf, Eigen::VectorXd offset, double duration, double dt){
    int nDof = _q.size();
    Eigen::VectorXd val(nDof), u(nDof);
    _tau.commands.resize(nDof);
    Eigen::VectorXd qDes(nDof), qdDes(nDof), qddDes(nDof);
    int numSteps = duration/dt + 1;
    /**
     * Following code is for the quintic polynoimal trajectory generation
     */
    double t = 0.0, tf = duration;
    Eigen::MatrixXd A(6,6);
    A << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
         0, 1, 2*t, 3*pow(t, 2), 4*pow(t, 3), 5*pow(t, 4),
         0, 0, 2, 6*t, 12*pow(t, 2), 20*pow(t, 3),
         1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5),
         0, 1, 2*tf, 3*pow(tf, 2), 4*pow(tf, 3), 5*pow(tf, 4),
         0, 0, 2, 6*tf, 12*pow(tf, 2), 20*pow(tf, 3);
    
    Eigen::MatrixXd B(6,nDof);
    B.setZero();
    auto q0 = _q + offset;
    B.block(0, 0, 1, nDof) = q0.transpose();
    B.block(3, 0, 1, nDof) = qf.transpose();
    Eigen::  MatrixXd coeff = (A.inverse()*B).transpose();

    auto pars = [](double t){
        Eigen::MatrixXd times(6,3);
        times << 1, 0, 0,
                t, 1, 0,
                pow(t, 2), 2*t, 1,
                pow(t, 3), 3*pow(t, 2), 3*t,
                pow(t, 4), 4*pow(t, 3), 6*pow(t, 2),
                pow(t, 5), 5*pow(t, 4), 10*pow(t, 3);
        return times;
    };
    RCLCPP_INFO_STREAM(this->get_logger(), "Running the inverser Dynamics Control for trajectory tracking");
    for (int iter = 0; iter < numSteps+1; iter++){
        rclcpp::spin_some(this->get_node_base_interface());

        t = iter*dt;
        Eigen::MatrixXd traj = coeff*pars(t);
        // extract data
        qDes = traj.block<7,1>(0,0);
        qdDes = traj.block<7,1>(0,1);
        qddDes = traj.block<7,1>(0,2);

        // compute the system matrices
        pinocchio::crba(_model, _data, _q);
        pinocchio::computeCoriolisMatrix(_model, _data, _q, _qdot);
        pinocchio::computeGeneralizedGravity(_model, _data, _q);

        _data.M.triangularView<Eigen::StrictlyLower>() = 
                        _data.M.transpose().triangularView<Eigen::StrictlyLower>();

        // compute the control input
        val = qddDes + _Kp*(qDes -_q) + _Kd*(qdDes-_qdot);

        u = _data.M*val + _data.C*_qdot + _data.g;

        Eigen::Map<Eigen::VectorXd>(_tau.commands.data(), _tau.commands.size()) = u;                
        RCLCPP_INFO_STREAM(this->get_logger(), "e = [" << (_qDes -_q).transpose() << "]");
        _commander->publish(_tau);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Message Checking " << sensor_msgs::msg::to_yaml(*_state));
        rclcpp::sleep_for(std::chrono::milliseconds((int) dt*1000));
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Completed the executing trajectory tracking");
}

void InverseDynamicsControl::sampleTest(int jointIndex){
    double period = 10.0, dt = 0.001, t = 0.0;
    double amplitude = this->get_parameter("amplitude").as_double();
    int nDof = _q.size();
    _tau.commands.resize(nDof);
    while (rclcpp::ok()){
        auto val = amplitude*sin(2*M_PI/period*t);
        t+=dt;
        for (int index = 0; index < nDof; index++){
            _tau.commands.at(index) = 0.0;
            if (index == jointIndex)
                _tau.commands.at(index) = val;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "tau = " << torque_msgs::msg::to_yaml(_tau));
        _commander->publish(_tau);

        rclcpp::sleep_for(std::chrono::milliseconds((int) dt*1000));
    }
}

void InverseDynamicsControl::pdControl(Eigen::VectorXd qDes){
    _qDes = qDes;
    Eigen::VectorXd val(_q.size()), u(_q.size());
    _tau.commands.resize(_q.size());
    _motion_generator = std::make_unique<MotionGenerator>(0.2, _q, _qDes);
    _start_time = this->now();
    while (rclcpp::ok()){
        rclcpp::spin_some(this->get_node_base_interface());
        getGains();
        auto trajectory_time = this->now() - _start_time;
        auto motion_generator_output = _motion_generator->getDesiredJointPositions(trajectory_time);
        Eigen::Matrix<double, 7, 1> q_i = motion_generator_output.first;
        bool finished = motion_generator_output.second;

        if (!finished){
            // compute the control input
            u = _Kp*(q_i -_q) + _Kd*(-_qdot);

            Eigen::Map<Eigen::VectorXd>(_tau.commands.data(), _tau.commands.size()) = u;
            RCLCPP_INFO_STREAM(this->get_logger(), "e = [" << (q_i -_q).transpose() << "]");
            _commander->publish(_tau);
            RCLCPP_INFO_STREAM(this->get_logger(), "u = " << u.transpose());
        } 
        rclcpp::sleep_for(1ms);
    }
}

void InverseDynamicsControl::collectSamples(std::string fileName){
    std::ofstream file(fileName);
    _start_time = this->now();
    auto trajectory_time = this->now() - _start_time;
    double val = 0.0;
    rclcpp::spin_some(this->get_node_base_interface());
    rclcpp::sleep_for(1ms);
    while (val <=  10.0){
        rclcpp::spin_some(this->get_node_base_interface());
        trajectory_time = this->now() - _start_time;
        val = trajectory_time.nanoseconds()*1e-09;
        file << val << "," << _q(0) << "," << _q(1) << "," << _q(2) << "," << _q(3) 
            << "," << _q(4) << "," << _q(5) << "," << _q(6) << ","
             << _qdot(0) << "," << _qdot(1) << "," << _qdot(2) << "," 
             << _qdot(3) << "," << _qdot(4) << "," << _qdot(5) << "," << _qdot(6) <<"\n";
        RCLCPP_INFO_STREAM(this->get_logger(), "time = " << val );
        // rclcpp::sleep_for(1ms);
    }

}
