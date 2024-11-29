
#include <fstream>
#include <chrono>
#include <string>
#include <sstream>
#include <signal.h>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <iomanip>
using namespace std;
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
    for (int col = 0; col <  vals[0].size(); col++ ){
        for(int row = 0; row < vals.size();row++){
            data(row,col) = vals[row][col];
        }
    }
}
;
int main(){
    std::string fileName = "/home/shravista/muse_ws/src/torque_controllers/torque_control_tests/data/dataMinJerkSample_1.csv";
    Eigen::MatrixXd vals;

    std::string fileName = "/home/shravista/muse_ws/src/torque_controllers/torque_control_tests/data/dataMinJerkSample_1.csv";
    csv2mat(fileName, vals);
    Eigen::VectorXd qDes(7), qdDes(7), qddDes(7);
    for (int iter = 0; iter < vals.rows(); iter++){

        // extract data
        qDes = vals.block<1,7>(iter,0);
        qdDes = vals.block<1,7>(iter,7);
        qddDes = vals.block<1,7>(iter,14);
    }
}