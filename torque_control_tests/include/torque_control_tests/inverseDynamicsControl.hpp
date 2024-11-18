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
#include "torque_msgs/msg/commands.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;

class InverseDynamicsControl: public rclcpp::Node{
    /**
     * The class for the demonstating the effect of Inverse Dynamics Control strategies for
     * the class of Euler-Lagrangian system in specific to manipulators.
     */
    protected:
        // ros
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _pub;
        rclcpp::Publisher<torque_msgs::msg::Commands>::SharedPtr _commander;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _subs;
        sensor_msgs::msg::JointState::SharedPtr _state;
        std_msgs::msg::Float64MultiArray _msg;
        torque_msgs::msg::Commands _tau;
        rclcpp::TimerBase::SharedPtr _timer;
        
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
        /**
         * This method declare the parameter as required by the node.
         *  The following parameters format are being used in this project
         *  ros__parameters:
         *      <robot_name>:
         *          package_name: 
         *          relative_path: <comments: directory that folder contains the urdf description>
         *          description: robot description file
         *          controller_name: controller name
         *          kp: [0.0, 0.0, ..., 0.0] position control gain
         *          kd: [0.0, 0.0, ..., 0.0] velocity control gain
         */
        void declareParams();

        /**
         * This method assigns the control gains (@see declareParams) to appropriate variable
         */
        void getGains();

        /**
         * The method is a callback method for the joint states subscriber
         * @inputs:
         *          msg = (sensor_msgs::msg::JointState::SharedPtr) as provided by the joint state publisher
         * @outputs:
         *          None
         */
        void callback(sensor_msgs::msg::JointState::SharedPtr msg);
    
    public:
        /**
         * The following constructor to be used when the topic for commands provied by the default
         * effort_controllers/JointGroupEffortControllers and the topic provided by this controller is of
         * <controller name>/commands. 
         * @arg:
         *      robot_name = (string) robot name as defined in the params file and urdf
         */
        InverseDynamicsControl(std::string robot_name);

        /**
         * The following constructor to be used when the topic for commands provied by the 
         * torque_controller/TorqueController and the topic provided by this controller is of
         * <controller name>/torque. 
         * @arg:
         *      robot_name = (string) robot name as defined in the params file and urdf
         *      version    = (string) just to distinguish the different controller types
         */
        InverseDynamicsControl(std::string robot_name, std::string version);
        ~InverseDynamicsControl(){};

        /**
         * The following methods (version 1 or 2) are implemented with the same idea of sending the control inputs as
         * torques to the robot manipulator. These methods are self documented.
         */
        // version 1
        void run(Eigen::VectorXd target);
        void pdGravityControl(Eigen::VectorXd target);
        void gravityCompensation();

        // version 2
        void run2(Eigen::VectorXd target);
        void pdGravityControl2(Eigen::VectorXd target);
        void gravityCompensation2();


};
#endif //_INVERSE_DYNAMICS_CONTROL_HPP_