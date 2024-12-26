#include "torque_control_tests/inverseDynamicsControl.hpp"
#include <fstream>
#include <chrono>
#include <string>
#include <sstream>
#include <signal.h>
#include <vector>

#define IDC 0 // inverse dynamics control
#define PDG 1 // pdGravityControl
#define GC  2 // gravity compensation
#define IDCTRAJ 3 // traj track

using namespace std;

void signalHandler( int signum ) {
    std::cout << "\033[0;"+std::to_string(31)+"m" <<"Keyboard Interrupt is detected: " << signum << std::endl;
    rclcpp::shutdown();
    std::abort();
}
vector<vector<double>> csv2mat(string fileName){
    cout << " Iam callaed" << endl;
    ifstream file(fileName);
    cout << "File Exists" << std::boolalpha << file.good() << endl;
    vector<vector<double>> vals;
    string line, word;
    int iter = 0;
    while (getline(file, line)){
        stringstream ss(line);
        vector <double> val;
        // cout << "i am called" << iter << endl;
        while (getline(ss, word, ',')){
            // cout << word << endl;
            val.push_back(stold(word));
        }
        vals.push_back(val);
        // iter++;
    }
        cout << "i am called" << iter << endl;

    file.close();
    return vals;
}

void csv2mat(string fileName, Eigen::MatrixXd& data){
    vector<vector<double>> vals = csv2mat(fileName);
    data.resize(vals.size(), vals[0].size());
    cout << "data " << data.rows() << ", " << data.cols() << endl;
    for (size_t col = 0; col <  vals[0].size(); col++ ){
        for(size_t row = 0; row < vals.size();row++){
            data(row,col) = vals[row][col];
        }
    }
}

#define PRINT(VAR, VAL) std::cout << "MEthod = " #VAR << "" #VAL " = " << VAL <<std::endl;

int main(int argc, char* argv[]){
    
    rclcpp::init(argc, argv);

    // change the desired joint angles here
    Eigen::VectorXd qDes = Eigen::VectorXd::Zero(7);

    // default method for running this control method is set poing tracking
    int method = IDC;

    /**
     * the above method can be modified by the user with the passing argument at command line 
     * the first argument is the method type. The possible values for the same are
     * 0: set point tracking with inverse dynamics control
     * 1: pd gravity control
     * 2: gravity compensation
     * 3: trajectory tracking with inverse dynamics control
     */
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Description: the method used in this node can be modified by the user with the passing argument at command line " <<
     "* the first argument is the method type. The possible values for the same are\n" << 
     "* 0: set point tracking with inverse dynamics control\n" <<
     "* 1: pd gravity control\n" << 
     "* 2: gravity compensation\n" <<
     "* 3: trajectory tracking with inverse dynamics control");
     
    if (argc > 2){
        method = atoi(argv[1]); // just a fix needs to be changed more appropriately
    }
    
    std::string fileName = "/home/shravista/muse_ws/src/torque_controllers/torque_control_tests/data/dataMinJerkSample_1.csv";
    Eigen::MatrixXd vals;
    csv2mat(fileName, vals);
    InverseDynamicsControl controller("iiwa14", "Ver2");

    if (method == IDC)
        controller.run2(qDes);
    else if (method == PDG)
        controller.pdGravityControl2(qDes);
    else if (method==GC)
        controller.gravityCompensation2();
    else{
        PRINT(method, IDCTRAJ)
        auto q0 = vals.block(0,0,1,7).transpose();
        auto qf = vals.block(vals.rows()-1,0,1,7).transpose();
        PRINT(q0, q0.transpose())
        PRINT(qf, qf.transpose())
        
        controller.run2(q0,qf,10,0.001);
    }
    rclcpp::shutdown();

}
