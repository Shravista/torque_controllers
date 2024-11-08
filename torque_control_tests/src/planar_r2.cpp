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

class PlanarR1Controller : public rclcpp::Node {
    public:
        PlanarR1Controller(Eigen::Vector2d target, std::string fileName) : Node("planar_r2"), _fileName(fileName){
            
            _pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("r2_effort_controller/commands",10);
            _subs = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, 
                                                                            std::bind(&PlanarR1Controller::callback, this, _1));
            
            // pinocchio
            pinocchio::urdf::buildModel(_fileName, _model);
            _data = pinocchio::Data(_model);

            _q = pinocchio::randomConfiguration(_model);
            _qdot = Eigen::VectorXd::Zero(_q.size());
            _qDes = Eigen::VectorXd::Zero(_q.size());
            _qDes = target;

            
            // gains
            _Kp(0,0) = 16.0;
            _Kp(1,1) = 16.0;
            _Kd(0,0) = 8.0;
            _Kd(1,1) = 8.0;

            // needed for all callbacks have been called
            rclcpp::spin_some(this->get_node_base_interface());
            rclcpp::sleep_for(5ms);
        }

        void callback(sensor_msgs::msg::JointState::SharedPtr msg){
            _state = msg;
            _q = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_state->position.data(), _state->position.size());
            _qdot = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_state->velocity.data(), _state->velocity.size());
         }

        void run(){
            Eigen::VectorXd val(1), u(1);
            // rclcpp::spin
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

                Eigen::Map<Eigen::Vector2d>(_msg.data.data(), _msg.data.size()) = u;                

                _pub->publish(_msg);
                // RCLCPP_INFO_STREAM(this->get_logger(), "Message Checking " << sensor_msgs::msg::to_yaml(*_state));
                rclcpp::sleep_for(1ms);
            }
        }
    protected:
        // ros
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _pub;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _subs;
        sensor_msgs::msg::JointState::SharedPtr _state;
        std_msgs::msg::Float64MultiArray _msg;
        
        // control parameters
        Eigen::Matrix2d _Kp = Eigen::Matrix2d::Zero();
        Eigen::Matrix2d _Kd = Eigen::Matrix2d::Zero();
        long _iter=0;

        // states
        Eigen::VectorXd _q, _qdot, _qDes, _qdDes, _qddDes;

        // pinocchio
        const std::string _fileName;
        pinocchio::Model _model;
        pinocchio::Data _data;
        
};
int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);

    std::string pathToURDF  = "/home/shravista/muse_ws/src/torque_controllers/example_robots/planar_robots/urdf/planar_r2.urdf";
    Eigen::Vector2d qDes = {M_PI/4.0, M_PI/3.0};
    if (argc == 3){
        qDes(0) = atof(argv[1])/180.0*M_PI;
        qDes(1) = atof(argv[2])/180.0*M_PI;
    }
    
    PlanarR1Controller controller(qDes, pathToURDF);
    controller.run();

}

