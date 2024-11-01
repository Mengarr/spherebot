#ifndef REMOTE_CONTROL_NODE_HPP_
#define REMOTE_CONTROL_NODE_HPP_

#define ALPHA_DOT_SCALE_FACTOR 3.5
#define U_DOT_SCALE_FACTOR 0.3
#define RPM_TO_RPS 60 // RPM TO RPS = 60:1

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/bool.hpp>
#include "control_lib/auxilary_arduino.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

class RemoteControlNode : public rclcpp::Node
{
public:
    RemoteControlNode();
private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void headingCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void jointTrajectoryStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
    void timerCallback();
    
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_override_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mag_sub_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_trajectory_state_sub_;
   
    rclcpp::TimerBase::SharedPtr timer_;

    void publishJointTrajectory(); // Helper function to publish messages

    float _lat = 0.0;
    float _long = 0.0;
    float _alt = 0.0;
    bool _gps_fix = false;

    // Heading data
    float heading_ = 0.0;

    // Mapped axes
    double mapped_axes1_;
    double mapped_axes3_;

    // Button states
    bool X_BUTTON_;
    bool O_BUTTON_;

    // Previous button states for edge detection
    bool prev_x_button_;
    bool prev_o_button_;

    AuxilaryArduino arduino;
    
    double u_ref_ = 0.0;
    double alphadot_ref_ = 0.0;
    double u_dot_ref = 0.0;
    
    // Private variables for measured joint state:
    double u_meas_, alpha_meas_;        // Positions
    double udot_meas_, alphadot_meas_;  // Velocities

};

#endif // REMOTE_CONTROL_NODE_HPP_
