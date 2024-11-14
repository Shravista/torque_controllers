#include "torque_control_tests/inverseDynamicsControl.hpp"

int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);

    InverseDynamicsControl controller("iiwa14");
    controller.gravityCompensation();
    rclcpp::shutdown();

}