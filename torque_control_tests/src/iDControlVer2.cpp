#include "torque_control_tests/inverseDynamicsControl.hpp"


#define IDC 0 // inverse dynamics control
#define PDG 1 // pdGravityControl
#define GC  2 // gravity compensation

int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);

    Eigen::VectorXd qDes = Eigen::VectorXd::Zero(7);
    int method = IDC;
    if (argc == 11){
        for (int i = 0; i < argc-1; i++)
            qDes(i) = atof(argv[i+1])/180.0*M_PI;
    }
    
    
    InverseDynamicsControl controller("iiwa14", "Ver2");

    if (method == IDC)
        controller.run2(qDes);
    else if (method == PDG)
        controller.pdGravityControl2(qDes);
    else
        controller.gravityCompensation2();
    rclcpp::shutdown();

}
