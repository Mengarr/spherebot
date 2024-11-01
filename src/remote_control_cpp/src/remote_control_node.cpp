#include "../include/remote_control_cpp/remote_control_node.hpp"

RemoteControlNode::RemoteControlNode()
: Node("remote_control_node"),
  mapped_axes1_(0.0),
  mapped_axes3_(0.0),
  X_BUTTON_(false),
  O_BUTTON_(false),
  prev_x_button_(false),
  prev_o_button_(false),
  arduino(AUX_ARDUINO_I2C_ADDR),
  phi_ref_(0.0),
  u_meas_(0.0),
  alpha_meas_(0.0),
  udot_meas_(0.0),
  alphadot_meas_(0.0)
{   
    RCLCPP_INFO(this->get_logger(), "Ardino Init:");
    arduino.init();
    RCLCPP_INFO(this->get_logger(), "Initalisation Complete");
    // Initialize subscriber to "joy" topic
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        10,
        std::bind(&RemoteControlNode::joyCallback, this, std::placeholders::_1)
    );

    // Initialize subscriber to gps lat long topic
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix",
        10,
        std::bind(&RemoteControlNode::gpsCallback, this, std::placeholders::_1)
    );

    // Initialize subscriber to "mag" topic
    mag_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "compensated_heading",
        10,
        std::bind(&RemoteControlNode::headingCallback, this, std::placeholders::_1)
    );

    // Initalize publisher to motor control node:
    joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "motor/joint_vars", 1
    );

    // Initalize subscriber to joint trajectory state
    joint_trajectory_state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "motor/joint_vars_state",
        2,
        std::bind(&RemoteControlNode::jointTrajectoryStateCallback, this, std::placeholders::_1)
    );
    
    // 
    state_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        "motor/motor_control_state", 1
    );

    // Initialize a timer that runs at 10 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 100 ms interval corresponds to 10 Hz
        std::bind(&RemoteControlNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Remote Control Node has been started.");
}


// void RemoteControlNode::phi_calibration() {
//     // Function to calibrate phi based on IMU measurments
// }

void RemoteControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Ensure that there are enough axes and buttons
    if (msg->axes.size() < 4 || msg->buttons.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Received Joy message with insufficient axes or buttons.");
        return;
    }

    // Map axes[1] from [-1, 1] to [-100, 100]
    mapped_axes1_ = msg->axes[1] * 100.0;

    // Map axes[3] from [-1, 1] to [-100, 100]
    mapped_axes3_ = msg->axes[3] * 100.0;

    // Handle X_BUTTON toggle (buttons[0])
    bool current_x_button = (msg->buttons[0] == 1);
    if (current_x_button && !prev_x_button_) {
        X_BUTTON_ = !X_BUTTON_;
    }
    prev_x_button_ = current_x_button;

    // Handle O_BUTTON toggle (buttons[1])
    bool current_o_button = (msg->buttons[1] == 1);
    if (current_o_button && !prev_o_button_) {
        O_BUTTON_ = !O_BUTTON_;
    }
    prev_o_button_ = current_o_button;
}

void RemoteControlNode::headingCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    heading_ = msg->data;
}

void RemoteControlNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    _lat = msg->latitude;
    _long = msg->longitude;
    _alt = msg->altitude;
    
    // Check fix
    _gps_fix = (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX);
}

void RemoteControlNode::jointTrajectoryStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
    // Check if the feedback has sufficient positions and velocities
    if (msg->feedback.positions.size() < 2 || msg->feedback.velocities.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Received invalid JointTrajectoryControllerState message");
        return;
    }

    // Update joint variables from the feedback
    u_meas_ = msg->feedback.positions[0];
    alpha_meas_ = msg->feedback.positions[1];
    udot_meas_ = msg->feedback.velocities[0];
    alphadot_meas_ = msg->feedback.velocities[1];
}


void RemoteControlNode::publishJointTrajectory()
{
    // Create the JointTrajectory message
    auto joint_vars_msg = trajectory_msgs::msg::JointTrajectory();

    // Set the joint names for each joint
    joint_vars_msg.joint_names = {"c_drive"}; 


    // Create a JointTrajectoryPoint to hold the positions and velocities
    trajectory_msgs::msg::JointTrajectoryPoint point;

    // Set positions for each joint
    point.positions.push_back(u_ref_);         // u_ref_ value
    point.positions.push_back(0.0);            // alpha_ref value (not used)

    // Set velocities for each joint
    point.velocities.push_back(u_dot_ref);           // Velocity of joint u, must be 0.0
    point.velocities.push_back(alphadot_ref_); // Velocity of joint alpha

    // Add PID efforts
    point.effort.push_back(0.0);            // Kd
    point.effort.push_back(0.06);           // Ki
    point.effort.push_back(0.1);            // Kp

    // Add the point to the JointTrajectory message
    joint_vars_msg.points.push_back(point);

    // Create a JointTrajectoryPoint to hold the positions and velocities
    trajectory_msgs::msg::JointTrajectoryPoint point_phi;

    // Set positions for each joint
    point_phi.positions.push_back(phi_ref_);         // u_ref_ value
    point_phi.positions.push_back(0.0);            // alpha_ref value (not used)

    // Set velocities for each joint
    point_phi.velocities.push_back(0.0);           // Velocity of joint u, must be 0.0
    point_phi.velocities.push_back(alphadot_ref_); // Velocity of joint alpha

    // Add PID efforts
    point_phi.effort.push_back(0.0);            // Kd
    point_phi.effort.push_back(0.1);           // Ki
    point_phi.effort.push_back(2);            // Kp


    // Add the point to the JointTrajectory message
    joint_vars_msg.points.push_back(point_phi);
    joint_trajectory_pub_->publish(joint_vars_msg);
}

void RemoteControlNode::timerCallback()
{   
    if (!X_BUTTON_) {

        if (fabs(u_meas_) > 40) {
            alphadot_ref_ = 0.0;
            u_ref_ = 0.0;
            phi_ref_ = 0.0;
            u_dot_ref = 0.0;
            publishJointTrajectory(); // Command Motors
            RCLCPP_WARN(this->get_logger(), "Motor soft limits reached. haulting robot");
            return;
        }
    }

    if (X_BUTTON_) {
        // Engaged
        if (O_BUTTON_) {
            arduino.setMotorSpeedDir(0x0F);
        }
        else {
            arduino.setMotorSpeedDir(0xF0);
        }
    }
    else {
        arduino.setMotorSpeedDir(0x00);
    }

    // Apply deadzone to control inputs (SCALE THE MAPPED AXES HERE SO THAT THE OUTPUT is sensible)
    alphadot_ref_ = (mapped_axes1_ / RPM_TO_RPS) / ALPHA_DOT_SCALE_FACTOR;

    u_dot_ref = (mapped_axes3_ / RPM_TO_RPS)  / U_DOT_SCALE_FACTOR;

    publishJointTrajectory(); // Command Motors

    std_msgs::msg::Int8 state_msg;
    state_msg.data = static_cast<int8_t>(MotorState::MANUAL);
    state_pub_->publish(state_msg);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

