#ifndef REMOTE_CONTROL_NODE_HPP_
#define REMOTE_CONTROL_NODE_HPP_

#define ALPHA_DOT_SCALE_FACTOR 3.5
#define U_DOT_SCALE_FACTOR 0.3
#define RPM_TO_RPS 60 // RPM TO RPS = 60:1

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include <fstream>
#include <iomanip>
#include <filesystem> // C++17 library for directory handling
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
    void timerCallback();
    
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mag_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float _lat;
    float _long;
    float _alt;
    bool _gps_fix;

    // Heading data
    float heading_;

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

};

#endif // REMOTE_CONTROL_NODE_HPP_
