#include "torque_control_tests/inverseDynamicsControl.hpp"
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <vector>

int main(int argc, char* argv[]){
    std::vector<std::string> joint_names{"fr3_joint1","fr3_joint2","fr3_joint3",
                                         "fr3_joint4","fr3_joint5","fr3_joint6",
                                        "fr3_joint7"};
    InverseDynamicsControl controller(joint_names);

    // take inputs from the command line
    InputParser input(argc, argv);
    Eigen::VectorXd qDes = Eigen::VectorXd::Zero(7);
    
    if (input.cmdOptionExists("-j")){
        auto val = input.getCmdOption("-j");
        std::cout << "val = " << val << std::endl;
        std::stringstream ss(val);
        int counter = 0;
        std::string word;
        while(!ss.eof()){
            std::getline(ss, word, ' ');
            counter ++;
            if (counter < 8)
                qDes(counter-1) = std::stod(word)*M_PI/180.0;
        }
        std::cout << qDes.transpose() << std::endl;
        if (counter < 7){
            std::cout << "not enough inputs\nshutting down!!!" << std::endl;
            rclcpp::shutdown();
        }
    }
    rclcpp::shutdown();
}