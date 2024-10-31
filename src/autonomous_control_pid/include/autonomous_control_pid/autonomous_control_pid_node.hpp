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
#include "control_lib/kinematic_transforms.hpp"
#include "control_lib/geodeticConverter.hpp"
#include "control_lib/PID_controller.hpp"
#include "control_lib/load_path_json.cpp"
#include "control_lib/LowpassFilter.hpp"
#include <cmath> // For abs()
#include <utility> // For pair

typedef enum class STATES {
    INITIALIZING,
    PATH_FOLLOWING,
    CALIBRATE_PHI,
    CALIBRATE_U,
    PROBE_SOIL,
    FINISH
} STATES;

class AutonomousControlNodePID : public rclcpp::Node
{
public:
    AutonomousControlNodePID();
    ~AutonomousControlNodePID() = default;

private:
    // Callbacks ----
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void headingCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void gpsCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    // Functions ----
    void controlLoop(); // Control loop timer callback

    // Controllers ----
    // PID controller gains
    double Kp_u_ = 0.1; double Ki_u_ = 0.06; double Kd_u_ = 0; // For u
    double Kp_phi_ = 0.1; double Ki_phi_ = 0.06; double Kd_phi_ = 0; // For phi
    // Stanley control gains 
    double k_ = 0.1; double k_s_ = 0.01; double L_ = 1;
    
    // Classes ----
    StanleyController _stanley;
    PathDataLoader _pathData;
    GeodeticConverter _geodeticConverter;
    MotorControl motor;
    AuxilaryArduino arduino;
    PIDController u_PID_;
    PIDController phi_PID_;

    // Publishers ---


    // Subscribers ----
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback Data ----
    // float _AccelX, _AccelY, _AccelZ; // acceleromter callbackd data +- 1g
    float _pitch, _roll;    // From accelerometer, in rad
    float _heading;         // magnetometer heading
    float _lat;             // gps latitude
    float _long;            // gps longitude
    float _alt;             // gps altitude
    bool _gps_fix = false;

    // Calibration ----
    // U and Phi calibration tollerances
    float t_hold_phi      = 4.0;  // must hold within the tollerance for this time
    float t_hold_u        = 3.0;
    float u_tolerance = 1.0;  // 1mm tollerance
    float phi_tolerance = 5.0; // 5 degrees tolerance
    // calibrated values
    float u_calib = 0.0; // u displacement as calibrated from CALIBRATE_PHI

    // State machine ----
    STATES current_state_;
    STATES next_state_;
    void nextStateLogic()

    // Lowpass filters


    // Data logging variables ----
    
    // Button states ----
    bool X_BUTTON_;
    bool prev_x_button_;     // Previous button states for edge detection
};

#endif // AUTONOMOUS_CONTOL_NODE_PID_HPP_
