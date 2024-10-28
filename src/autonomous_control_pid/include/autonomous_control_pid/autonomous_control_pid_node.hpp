#ifndef AUTONOMOUS_CONTOL_NODE_PID_HPP_
#define AUTONOMOUS_CONTOL_NODE_PID_HPP_

#define controlLoopHZ 30 // 30Hz

#define ALPHA_DOT_SCALE_FACTOR 1
#define U_DOT_SCALE_FACTOR 1
#define RPM_TO_RPS 60 // RPM TO RPS = 60:1

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "control_lib/auxilary_arduino.hpp"
#include "control_lib/motorControl.hpp"
#include "control_lib/geodeticConverter.hpp"
#include "control_lib/stanleyControl.hpp"
#include "control_lib/load_path_json"
#include <cmath> // For abs()
#include <utility> // For pair

typedef enum class STATES {
    INITIALIZING,
    PATH_FOLLOWING,
    PROBE_SOIL,
    FINISH
} STATES;

class AutonomousControlNodePID : public rclcpp::Node
{
public:
    AutonomousControlNodePID();
    ~AutonomousControlNodePID() = default;

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void magnetometerCallBack(const std_msgs::msg::Float32::SharedPtr msg);
    void gpsCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void controlLoop();

    StanleyController _stanley(0.1, 0.01, 1, 0);
    PathDataLoader _pathData("../data/test_path.json");
    GeodeticConverter _geodeticConverter();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gps_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data
    float _heading; // magnetometer heading
    float _lat;
    float _long;
    float _alt;
    bool _gps_fix = false;

    // State machine
    STATES current_state_;
    void nextStateLogic()
    
    // Button states
    bool X_BUTTON_;
    bool prev_x_button_;     // Previous button states for edge detection

    // For motor control
    MOTOR_CONTROL motor;
    AUX_ARDUINO arduino;
};

#endif // AUTONOMOUS_CONTOL_NODE_PID_HPP_
