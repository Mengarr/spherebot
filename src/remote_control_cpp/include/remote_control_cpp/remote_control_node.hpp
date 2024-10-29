#ifndef REMOTE_CONTROL_NODE_HPP_
#define REMOTE_CONTROL_NODE_HPP_

#define ALPHA_DOT_SCALE_FACTOR 2.5
#define U_DOT_SCALE_FACTOR 0.6
#define RPM_TO_RPS 60 // RPM TO RPS = 60:1

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include <fstream>
#include <iomanip>
#include <filesystem> // C++17 library for directory handling
#include "control_lib/auxilary_arduino.hpp"
#include "control_lib/motorControl.hpp"
#include "control_lib/kinematic_transforms.hpp"
#include "control_lib/geodeticConverter.hpp"


class RemoteControlNode : public rclcpp::Node
{
public:
    RemoteControlNode();
    ~RemoteControlNode();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void headingCallback(const std_msgs::msg::Float32::SharedPtr msg);

    void timerCallback();
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mag_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data logging 
    void init_csv_file();
    double t0_;

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

    // For motor control
    MOTOR_CONTROL motor;
    AUX_ARDUINO arduino;

    // GPS Class
    GeodeticConverter _geodeticConverter;
    bool _gps_fix = false;
    float _lat;
    float _long;
    float _alt;
    bool init = false;
};

#endif // REMOTE_CONTROL_NODE_HPP_
