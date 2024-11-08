#ifndef _INVERSE_DYNAMICS_CONTROL_HPP_
#define _INVERSE_DYNAMICS_CONTROL_HPP_

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp" 
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp" 
#include "pinocchio/serialization/model.hpp"
#include "pinocchio/algorithm/crba.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <filesystem>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class InverseDynamicsControl: public rclcpp::Node{
    protected:
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _pub;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _subs;
        sensor_msgs::msg::JointState::SharedPtr _state;
        std_msgs::msg::Float64MultiArray _msg;
        
        // control parameters
        Eigen::MatrixXd _Kp = Eigen::Matrix2d::Zero();
        Eigen::MatrixXd _Kd = Eigen::Matrix2d::Zero();
        long _iter=0;

        // states
        Eigen::VectorXd _q, _qdot, _qDes, _qdDes, _qddDes;

        // pinocchio
        std::string _fileName;
        pinocchio::Model _model;
        pinocchio::Data _data;
        const std::string _robot_name;

        // functions
        void declareParams();
        void getGains();
        void callback(sensor_msgs::msg::JointState::SharedPtr msg);
    
    public:
        InverseDynamicsControl(std::string robot_name);
        ~InverseDynamicsControl(){};

        void run(Eigen::VectorXd target);

};
#endif //_INVERSE_DYNAMICS_CONTROL_HPP_