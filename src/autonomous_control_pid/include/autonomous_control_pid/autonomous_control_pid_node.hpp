#ifndef AUTONOMOUS_CONTOL_NODE_PID_HPP_
#define AUTONOMOUS_CONTOL_NODE_PID_HPP_

#define ALPHA_DOT_SCALE_FACTOR 1
#define U_DOT_SCALE_FACTOR 1
#define RPM_TO_RPS 60 // RPM TO RPS = 60:1

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>              // For controller input
#include "std_msgs/msg/float32.hpp"             // For heading data
#include "sensor_msgs/msg/nav_sat_fix.hpp"      // For gps fix
#include "geometry_msgs/msg/point.hpp"          // for x,y coordinates
#include "control_lib/auxilary_arduino.hpp"
#include "control_lib/PathDataLoader.hpp"
#include "control_lib/kinematic_transforms.hpp"
#include "control_lib/stanleyControl.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>                 // For controlling the motor
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>   // For reading motor data
#include <cmath> // For abs()
#include <utility> // For pair
#include <chrono> // for time 
#include <vector> // for vector
#include <map>
#include <string>
#include "std_msgs/msg/int8.hpp"


enum class MotorState : int8_t {
    MANUAL = 0,       // No phi or u control
    PHI_CONTROL = 1,  // phi control
    U_CONTROL = 2     // u control
};


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
    // Loop freqency
    const float controlLoopHZ = 20.0; // 20Hz

    // Callbacks ----
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void headingCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void gpsLatLongCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void gpsCoordsCallBack(const geometry_msgs::msg::Point::SharedPtr msg);
    void jointTrajectoryStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
    
    // Functions ----
    void controlLoop(); // Control loop timer callback
    void publishJointTrajectory(); // Helper function to publish messages

    // Controllers ----
    // Final point tolerance
    double tolerance_ = 4; // 4m tolerance

    // Stanley control gains 
    double k_ = 0.1; double k_s_ = 0.01; double L_ = 1;
    
    // Classes ----
    StanleyController _stanley;
    PathDataLoader _pathData;

    // Publishers ---
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_; // For motor commands
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr stanley_heading_ref_pub_; // For stanley controller reference heading
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_pub_; // for motor control state
    // Subscribers ----
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_trajectory_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_latlong_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr gps_coords_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mag_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback Data ----
    // float _AccelX, _AccelY, _AccelZ; // acceleromter callbackd data +- 1g
    double _pos_x, _pos_y, _pos_z;   // From gps in m (enu converted)
    float _heading;                 // magnetometer heading
    bool _gps_fix = false;

    // Calibration ----
    // const float u_calib_ = 30;
    // Manually calibrate the system before hand then reset the arduino so this is the zero point.
    
    // State machine ----
    STATES current_state_;
    STATES next_state_;
    void nextStateLogic();
    MotorState motor_state_

    // Foward velocity control ----
    double phi_ref_ = 0.0;
    double u_ref_ = 0.0;
    double alphadot_ref_ = (100 / RPM_TO_RPS) / ALPHA_DOT_SCALE_FACTOR;  // Good foward velocity
    // Private variables for measured joint state:
    double u_meas_, alpha_meas_;        // Positions
    double udot_meas_, alphadot_meas_;  // Velocities

    // Data logging variables ----
    // Heading allready comes from mag node
    double x_coord_ = 0.0;
    double y_coord_ = 0.0;
    double ref_heading = 0.0;
    double u_error_ = 0.0;
    double vf_error_ = 0.0;

    // Define the params map
    std::map<std::string, float> params;

    // Button states ----
    bool X_BUTTON_;
    bool prev_x_button_;     // Previous button states for edge detection
};

#endif // AUTONOMOUS_CONTOL_NODE_PID_HPP_
