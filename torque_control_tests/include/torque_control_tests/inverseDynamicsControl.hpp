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
#include "franka_example_controllers/motion_generator.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

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

        rclcpp::Time _start_time;
        std::unique_ptr<MotionGenerator> _motion_generator;

        // states
        Eigen::VectorXd _q, _qdot, _qDes, _qdDes, _qddDes;
        double alpha = 0.99;

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
         * \brief The following constructor to be used when the topic for commands provied by the 
         * torque_controller/TorqueController and the topic provided by this controller is of
         * <controller name>/torque. 
         * @param name = (std::string) node name
         * @param joint_names = (std::vector<std::string>) vector of joint names
         */
        InverseDynamicsControl(std::string name, std::vector<std::string> joint_names);
        ~InverseDynamicsControl(){};

        /**
         * \brief The program run is for  set point tracking using inverse dynamics control
         * @param target = (Eigen::VectorXd) target joint angles
         */
        void run(Eigen::VectorXd target);

        /**
         * \brief The program run2 is  overloaded function for trajectory tracking using inverse dynamics control.
         * Note that initial condition for the trajectory is q0 + offset
         * @param qf = (Eigen::VectorXd) final joint angles
         * @param offset = (Eigen::VectorXd) offset from initial conditions
         * @param duration = (double) duration of the trajectory
         * @param dt = (double) time step
         */
        void run(Eigen::VectorXd qf, Eigen::VectorXd offset, double duration, double dt);

        /**
         * \brief The program sampleTest is to just actuate joints and see what is 
         * the minimum torque applied is required
         * @param jointIndex = (int) which index of the joint to actuate
         */
        void sampleTest(int jointIndex);

        /**
         * \brief The method pdControl for the set-point tracking
         * @param qDes = (Eigen::VectorXd) desired joint position -> use wisely
         */
        void pdControl(Eigen::VectorXd qDes);

        /**
         * \brief The method collectSamples for the collection of the states data
         */
        void collectSamples(std::string fileName);

        std::vector<std::vector<double>> csv2mat(std::string fileName){
            cout << " Iam callaed" << endl;
            ifstream file(fileName);
            cout << "File Exists" << std::boolalpha << file.good() << endl;
            std::vector<std::vector<double>> vals;
            std::string line, word;
            int iter = 0;
            while (getline(file, line)){
                stringstream ss(line);
                std::vector <double> val;
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
        
        void csv2mat(std::string fileName, Eigen::MatrixXd& data){
            std::vector<std::vector<double>> vals = csv2mat(fileName);
            data.resize(vals.size(), vals[0].size());
            cout << "data " << data.rows() << ", " << data.cols() << endl;
            for (size_t col = 0; col <  vals[0].size(); col++ ){
                for(size_t row = 0; row < vals.size();row++){
                    data(row,col) = vals[row][col];
                }
            }
        }
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
