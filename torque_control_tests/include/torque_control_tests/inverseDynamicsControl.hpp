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

        // joint names
        const std::vector<std::string> _joint_names;

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
         *      package_name: 
         *      controller_name: controller name
         *      kp: [0.0, 0.0, ..., 0.0] position control gain
         *      kd: [0.0, 0.0, ..., 0.0] velocity control gain
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

        /**
         * The function finds the index of the element enquired in the given vector
         * @param x = (std::vector<T>) vector of type T is reference
         * @param val = (T) element of type T that has to be searched in the given array
         */
        template<typename T> 
        size_t findID(std::vector<T>x, T val) {
            auto it = std::find(x.begin(), x.end(), val); 
            return (it - x.begin());
        }
    
    public:

        /**
         * The following constructor to be used when the topic for commands provied by the 
         * torque_controller/TorqueController and the topic provided by this controller is of
         * <controller name>/torque. 
         * @param names = (std::vector<std::string>) vector of joint names
         */
        InverseDynamicsControl(std::vector<std::string> names);
        ~InverseDynamicsControl(){};

        /**
         * \brief The program run is for  set point tracking using inverse dynamics control
         * @param target = (Eigen::VectorXd) target joint angles
         */
        void run(Eigen::VectorXd target);

        /**
         * \brief The program run2 is  overloaded function for trajectory tracking using inverse dynamics control
         * @param q0 = (Eigen::VectorXd) initial joint angles
         * @param qf = (Eigen::VectorXd) final joint angles
         * @param duration = (double) duration of the trajectory
         * @param dt = (double) time step
         */
        void run(Eigen::VectorXd q0, Eigen::VectorXd qf, double duration, double dt);

};

/**
 * The class method below is obtained from the stackexchange at the following web-address
 * https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
*/

class InputParser{
    public:
        InputParser (int &argc, char **argv){
            for (int i=1; i < argc; ++i)
                this->tokens.push_back(std::string(argv[i]));
        }
        /// @author iain
        const std::string& getCmdOption(const std::string &option) const{
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end()){
                return *itr;
            }
            static const std::string empty_string("");
            return empty_string;
        }
        /// @author iain
        bool cmdOptionExists(const std::string &option) const{
            return std::find(this->tokens.begin(), this->tokens.end(), option)
                   != this->tokens.end();
        }
    private:
        std::vector<std::string> tokens;
};
#endif //_INVERSE_DYNAMICS_CONTROL_HPP_