
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
    int nDof = 7;
    Eigen::VectorXd qDes(nDof), qdDes(nDof), qddDes(nDof);
    double t = 0, tf = 10, dt = 0.001;;
    int numSteps = tf/dt + 1;

    Eigen::MatrixXd A(6,6);
    A << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5),
         0, 1, 2*t, 3*pow(t, 2), 4*pow(t, 3), 5*pow(t, 4),
         0, 0, 2, 6*t, 12*pow(t, 2), 20*pow(t, 3),
         1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5),
         0, 1, 2*tf, 3*pow(tf, 2), 4*pow(tf, 3), 5*pow(tf, 4),
         0, 0, 2, 6*tf, 12*pow(tf, 2), 20*pow(tf, 3);
    
    Eigen::MatrixXd B(6,nDof);
    B.setZero();
    Eigen::VectorXd qf(nDof);
    qf << 1.3919,0.575147,-2.52887, -1.58961,2.85785, -0.0134066, -2.91741;
    B.block(3, 0, 1, nDof) = qf.transpose();
    Eigen::  MatrixXd coeff = (A.inverse()*B).transpose();

    cout << A << endl;;
    cout << B << endl;
    cout << coeff << endl;

    auto pars = [](double t){
        Eigen::MatrixXd times(6,3);
        times << 1, 0, 0,
                t, 1, 0,
                pow(t, 2), 2*t, 1,
                pow(t, 3), 3*pow(t, 2), 3*t,
                pow(t, 4), 4*pow(t, 3), 6*pow(t, 2),
                pow(t, 5), 5*pow(t, 4), 10*pow(t, 3);
        return times;
    };
    std::ofstream file("test.csv");
    for (int iter = 0; iter < numSteps+1; iter++){
        t = iter*dt;
        Eigen::MatrixXd traj = coeff*pars(t);
        cout << "traj =  << " << traj.rows() << ", " << traj.cols() << "\n" << traj << endl;
        // extract data
        qDes = traj.block<7,1>(0,0);
        qdDes = traj.block<7,1>(0,1);
        qddDes = traj.block<7,1>(0,2);
        file << qDes(0) << "," << qDes(1) << "," << qDes(2) << "," << qDes(3) << "," << qDes(4) << "," << qDes(5) << "," << qDes(6) << ","
             << qdDes(0) << "," << qdDes(1) << "," << qdDes(2) << "," << qdDes(3) << "," << qdDes(4) << "," << qdDes(5) << "," << qdDes(6) << ","
             << qddDes(0) << "," << qddDes(1) << "," << qddDes(2) << "," << qddDes(3) << "," << qddDes(4) << "," << qddDes(5) << "," << qddDes(6) << endl;
        // file << qDes(0) << "," << qdDes.transpose() << "," << qddDes.transpose() << endl;
    }
    file.close();
}