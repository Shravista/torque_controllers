#include "torque_control_tests/inverseDynamicsControl.hpp"

int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);

    Eigen::VectorXd qDes = Eigen::VectorXd::Zero(7);
    if (argc == 8){
        for (int i = 0; i < argc-1; i++)
            qDes(i) = atof(argv[i+1])/180.0*M_PI;
    }
    
    InverseDynamicsControl controller("iiwa14");
    rclcpp::shutdown();

}
