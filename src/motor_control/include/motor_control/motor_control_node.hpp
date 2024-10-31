#ifndef MOTOR_CONTROL_NODE_HPP
#define MOTOR_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include "../include/motor_control/motorControl.hpp"
#include "control_lib/kinematic_transforms.hpp"
#include "control_lib/PID_controller.hpp"
#include "control_lib/auxilary_arduino.hpp"

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode();
    ~MotorControlNode();
    
private:
    // Subscriber callback for JointTrajectory messages
    void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

    // Timer callback for the 20Hz control loop (prototype, no code needed inside)
    void controlLoop();
    
    // PID controller
    double Kp_ = 0.1; double Ki_ = 0.06; double Kd_ = 0;
    PIDController u_PID_;

    // motor control
    MotorControl motor;

    // auxilary arduino control
    AuxilaryArduino arduino;

    // Subscriber and publisher
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub_;
    rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_state_pub_;
    
    // Timer for the control loop at 20Hz
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    // Private vars for reference joint state:
    double u_ref_, alpha_ref_;
    double udot_ref_,  alphadot_ref_;

    // Private variables for measured joint state:
    double u_meas_, alpha_meas_;        // Positions
    double udot_meas_, alphadot_meas_;  // Velocities
};

#endif // MOTOR_CONTROL_NODE_HPP
