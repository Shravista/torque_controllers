#ifndef _TORQUE_CONTROLLER_HPP_
#define _TORQUE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "torque_controller/visibility_controller.h"
#include "torque_msgs/msg/commands.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"

#include "eigen3/Eigen/Eigen"
#include <chrono>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp" 
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp" 
#include "pinocchio/serialization/model.hpp"
#include "pinocchio/algorithm/crba.hpp"

/* Auto generated by the library*/
#include "torque_controller_parameters.hpp"

namespace torque_controller{
    
using torqueCmd = torque_msgs::msg::Commands;
using stateMsg = sensor_msgs::msg::JointState;
/**
 * TorqueConntroller is a constructor for the class TorqueController derived from the 
 * ControllerInterface. By virture of derivation, this constructor has to be called with no args.
 * Moreover, this implementation claims the hardware_interface::HW_IF_EFFORT for the torque control
 */
class TorqueController : public controller_interface::ControllerInterface {
    public:
        TORQUE_CONTROLLER_PUBLIC TorqueController();
        TORQUE_CONTROLLER_PUBLIC ~TorqueController() = default;

        /**
         * This method configures the command interfaces as declared in the parameter list set
         * by the user. This method is only called after the method @on_configure. 
         * @inputs: 
         *          No args
         * @outputs:
         *          controller_interface::InterfaceConfiguration (struct) with command interface
         *          names (<joint>/effort) and interface type (ALL, INDIVIDUAL, NONE).
         */
        TORQUE_CONTROLLER_PUBLIC 
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        /**
         * This method configures the state interfaces as declared in the parameter list set
         * by the user. This method is only called after the method @on_configure. 
         * @inputs: 
         *          No args
         * @outputs:
         *          controller_interface::InterfaceConfiguration (struct) with state interface
         *          names (<joint>/<state interface> e.g. joint1/position, joint1/velocity) and
         *          interface type (ALL, INDIVIDUAL, NONE).
         */
        TORQUE_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        /**
         * This method initializes various parameter of the node, memory reservation. Note that
         * any error handling has to be carried out if required.
         * @inputs:
         *          No args
         * @outputs:
         *          controller_interface::CallbackReturn (SUCCESS, FAILURE, ERROR)
         */
        TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        /**
         * This method reads the parameters that supplied by the user. It also does the exception handling
         * for node declared parameters if the user provides the parameters in wrong format.
         * @inputs:
         *      previous_state = rclcpp_lifecycle::State (not used in this implementation)
         * @outputs:
         *      state of success = controller_interface::CallbackReturn (SUCCESS, FAILURE, ERROR) 
         */
        TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        /**
         * This method assigns the state and command interfaces.
         * 
         * @inputs:
         *      previous_state = rclcpp_lifecycle::State (not used in this implementation)
         * @outputs:
         *      state of success = controller_interface::CallbackReturn (SUCCESS, FAILURE, ERROR) 
         */
        TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        /**
         * This method clears the state and command interfaces.
         * 
         * @inputs:
         *      previous_state = rclcpp_lifecycle::State (not used in this implementation)
         * @outputs:
         *      state of success = controller_interface::CallbackReturn (SUCCESS, FAILURE, ERROR) 
         */
        TORQUE_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        /**
         * This method steps through subscribed topic ("<controller_namer>/torque") message to provide to the robot.
         * Moreover, if no publishers to the topic ("<controller_name>/torque") is provided then this method 
         * implements @sendStaticInput. 
         * @warning: this method has to be made real time for practical applications.
         * @inputs:
         *          time   = (rclcpp::Time) current ROS time (not utilized)
         *          period = (rclcpp::Duration) controller rate (not utilized)
         * @outputs:
         *          return return_type::ERROR if it could not parse the message from the ros topic else return_type::SUCCESS 
         */
        TORQUE_CONTROLLER_PUBLIC
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    
    protected:
        // functions
        /**
         * This method declare the definition of parameters that are required by the node
         * @params:
         *          joints             = (string array) name of the joints that needs to be actuated
         *          K                  = (double array) gains that gives weight to the joint velocity of the robot 
         *                                  on how fast it converges to zero
         *          urdf_relative_path = (string array of size 2) [<packageName>, <pathToURDF Relative To Package>]
         */
        void declare_parameters();

        /**
         * This method reads the parameters that are defined by the user and handles the error if user-specified it incorrectly
         */
        controller_interface::CallbackReturn read_parameters();

        /**
         * This method is used within the @update method to calculate the required joint commands to the robot 
         * to maintain it in stationary mode iff no joint commands are written from the topic. This utitlized the classic
         * inverse dynamics control (or feedback linearization or control law partitioning) to perform this update. 
         * The inverse dynamics control strategy is implemented with no target on position and desired velocity to be zero at
         * every instant of the time. The resultant effect for the system is gravity compensation for the robot
         * manipulator. It is important to note that currently this notion requires dependency of model parameters 
         * of the system which is ideally not available for practical reasons. Future updates for this method is to 
         * include the approximation-free control. 
         * @warning: Because of the asymptotic nature of the equillibrium, the velocity state goes to zero as time tends to infinity.
         *           Therefore, the position may eventually drift from the current position. Although the effect is very slow, user is 
         *           warned to not keep the robot turned for very long duration of the time. This drift effect can be minimized by setting 
         *           large gains (@see declare params: K) for faster convergence and very small magnitude of joint angular velocity.
         *          
         */
        void sendStaticInput();

        // controller parameters
        std::vector<std::string> _joint_names;
        std::string _interface_name;
        Eigen::MatrixXd _K;
        Eigen::VectorXd _q, _qdot, _u_static;

        // pinocchio
        pinocchio::Model _model;
        pinocchio::Data _data;

        // ros 2
        std::vector<std::string> _command_interface_types, _state_interface_types;
        realtime_tools::RealtimeBuffer<std::shared_ptr<torqueCmd>> _rt_torque_command_ptr;
        realtime_tools::RealtimeBuffer<std::shared_ptr<stateMsg>> _rt_state_ptr;
        rclcpp::Subscription<torqueCmd>::SharedPtr _torque_commands_subs;
        // rclcpp::Subscription<stateMsg>::SharedPtr _state_subs;

        // interfaces
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> _cmd_interfaces;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> _state_interfaces;

        // params
        std::shared_ptr<ParamListener> _param_listener;
        Params _params;
};
} // torque_controller

#endif //_TORQUE_CONTROLLER_HPP_