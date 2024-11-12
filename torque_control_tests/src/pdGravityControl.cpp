#include "torque_control_tests/inverseDynamicsControl.hpp"
#include <iostream>

int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);

    Eigen::VectorXd qDes = Eigen::VectorXd::Zero(7);
    std::cout << "# args = " << argc << std::endl;
    for (int i = 1; i < argc; i++)
        std::cout << "# " << i << " arg: " << argv[i] << std::endl;
    if (argc == 11){
        for (int i = 0; i < 7; i++)
            qDes(i) = atof(argv[i+1])/180.0*M_PI;
    }
    std::cout << "qDes = " << qDes.transpose() << std::endl;
    InverseDynamicsControl controller("iiwa14");
    controller.pdGravityControl(qDes);
    rclcpp::shutdown();

}
