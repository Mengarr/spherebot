#ifndef MOTOR_CONTROL_NODE_HPP
#define MOTOR_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include "../include/motor_control/motorControl.hpp"
#include "control_lib/kinematic_transforms.hpp"
#include "control_lib/PID_controller.hpp"
#include "control_lib/auxilary_arduino.hpp"
#include "sensor_msgs/msg/imu.hpp" // For smart u and phi control
#include "std_msgs/msg/int8.hpp"

enum class State : int8_t {
    MANUAL = 0,       // No phi or u control
    PHI_CONTROL = 1,  // phi control
    U_CONTROL = 2     // u control
};

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode();
    ~MotorControlNode();
    
private:
    // Subscriber callback for JointTrajectory messages
    void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

    // state callback
    void stateCallback(const std_msgs::msg::Int8::SharedPtr msg);

    // Timer callback for the 20Hz control loop (prototype, no code needed inside)
    void controlLoop();
    
    // PID controllers
    double Kp_u_ = 0.0; double Ki_u_ = 0.0; double Kd_u_ = 0.0;
    double Kp_phi_ = 0.0; double Ki_phi_ = 0.0; double Kd_phi_ = 0.0;
    PIDController u_PID_;
    PIDController phi_PID_;

    // motor control
    MotorControl motor;

    // auxilary arduino control
    AuxilaryArduino arduino;

    // Imu stuff for phi control
    float roll_ = 0.0;
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Subscriber and publisher
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr state_sub_; // for motor control state
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub_;
    rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_state_pub_;
    
    // Timer for the control loop at 20Hz
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    // States
    State current_state_;

    // For phi control
    double phi_ref_, phidot_ref_;

    // Private vars for reference joint state:
    double u_ref_, alpha_ref_;
    double udot_ref_,  alphadot_ref_;

    // Private variables for measured joint state:
    double u_meas_, alpha_meas_;        // Positions
    double udot_meas_, alphadot_meas_;  // Velocities
};

#endif // MOTOR_CONTROL_NODE_HPP
