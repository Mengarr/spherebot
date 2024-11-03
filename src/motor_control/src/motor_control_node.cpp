#include "../include/motor_control/motor_control_node.hpp"

MotorControlNode::MotorControlNode() : 
  Node("motor_control"), 
  u_PID_(Kp_u_, Ki_u_, Kd_u_),
  phi_PID_(Kd_phi_,Ki_phi_,Kd_phi_),
  motor(MOTORS_I2C_ADDR),
  arduino(AUX_ARDUINO_I2C_ADDR),
  current_state_(State::MANUAL),
  phi_ref_(0.0),
  phidot_ref_(0.0),
  u_ref_(0.0), 
  alpha_ref_(0.0), 
  udot_ref_(0.0), 
  alphadot_ref_(0.0),
  u_meas_(0.0), 
  alpha_meas_(0.0), 
  udot_meas_(0.0), 
  alphadot_meas_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Motor && Arduino Init:");
    motor.init();
    arduino.init();
    RCLCPP_INFO(this->get_logger(), "Initalisation Complete");

    // Subscriber for JointTrajectory messages
    joint_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "motor/joint_vars", 5,
        std::bind(&MotorControlNode::jointTrajectoryCallback, this, std::placeholders::_1));

    // Subscriber for u_control overide messages
    state_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "motor/motor_control_state", 5,
        std::bind(&MotorControlNode::stateCallback, this, std::placeholders::_1));

    // Publisher for JointTrajectoryControllerState messages
    joint_state_pub_ = this->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
        "motor/joint_vars_state", 5);

    // Subscriber for imu data
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        5,
        std::bind(&MotorControlNode::imuCallback, this, std::placeholders::_1)
    );

    // Timer for control loop at 15Hz (75ms interval)
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(75),
        std::bind(&MotorControlNode::controlLoop, this));
    
    u_PID_.setOutputLimits(-0.7, 0.7);
    phi_PID_.setOutputLimits(-0.7, 0.7);
}

MotorControlNode::~MotorControlNode() {
    motor.reset();
    arduino.reset();
    RCLCPP_INFO(this->get_logger(), "Arduinos Reset");
}

void MotorControlNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    float ax = msg->linear_acceleration.x;
    float az = msg->linear_acceleration.z;
    roll_ = std::atan2(ax, az + 1e-8); // For phi
}

void MotorControlNode::stateCallback(const std_msgs::msg::Int8::SharedPtr msg) 
{
    current_state_ = static_cast<State>(msg->data);
}

// Callback for receiving JointTrajectory message and updating state
void MotorControlNode::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (msg->points.empty() || 
        msg->points[0].positions.size() < 2 || 
        msg->points[0].velocities.size() < 2 ||
        msg->points[0].effort.size() < 3 ||
        msg->points[1].positions.size() < 2 || 
        msg->points[1].velocities.size() < 2 ||
        msg->points[1].effort.size() < 3) {
        RCLCPP_WARN(this->get_logger(), "Received invalid JointTrajectory message");
        return;
    }
    
    if (current_state_ == State::MANUAL || current_state_ == State::U_CONTROL) {
        // Update joint variables from the message
        u_ref_ = msg->points[0].positions[0];
        alpha_ref_ = msg->points[0].positions[1];
        udot_ref_ = msg->points[0].velocities[0];
        alphadot_ref_ = msg->points[0].velocities[1];

        // Update PID gains from effort
        Kp_u_ = msg->points[0].effort[0];
        Ki_u_ = msg->points[0].effort[1];
        Kp_u_ = msg->points[0].effort[2];

        u_PID_.setGains(Kp_u_, Ki_u_, Kd_u_);
    } else if (current_state_ == State::PHI_CONTROL) {
        // Use points 0 for u and points 1 for phi
        phi_ref_ = msg->points[1].positions[0];
        alpha_ref_ = msg->points[1].positions[1];
        phidot_ref_ = msg->points[1].velocities[0];
        alphadot_ref_ = msg->points[1].velocities[1];

        // Update PID gains from effort
        Kd_phi_ = msg->points[1].effort[0];
        Ki_phi_ = msg->points[1].effort[1];
        Kp_phi_ = msg->points[1].effort[2];

        phi_PID_.setGains(Kp_phi_, Ki_phi_, Kd_phi_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Received unknown state: %d", static_cast<int8_t>(current_state_));
    }

    // Create and populate the JointTrajectoryControllerState message
    control_msgs::msg::JointTrajectoryControllerState state_msg;
    state_msg.joint_names = msg->joint_names;

    // Populate the feedback field with measured values
    state_msg.feedback.positions = {u_meas_, alpha_meas_, roll_};
    state_msg.feedback.velocities = {udot_meas_, alphadot_meas_};

    joint_state_pub_->publish(state_msg);
}

// Control loop prototype (20Hz)
void MotorControlNode::controlLoop()
{
    // Set motor speeds based on joint variable transformation
    std::pair<float, float> jointVariableVelocity = computeJointVariables(alphadot_ref_, udot_ref_);

    // Remove noisy signal from controller 
    if (fabs(jointVariableVelocity.first) < 0.08) {
        jointVariableVelocity.first = 0.0;
    }
    
    if (fabs(jointVariableVelocity.second) < 0.08) {
        jointVariableVelocity.second = 0.0;
    }

    // U and alpha measured values:
    int32_t motorACount;
    int32_t motorBCount;

    motor.readMotorACount(&motorACount);
    motor.readMotorBCount(&motorBCount);

    std::pair<float,float> u_alpha = computeJointVariablesInverse(motorACount, motorBCount);
    u_alpha.second = u_alpha.second * (1000 / CPR); // convert to mm

    u_meas_ = static_cast<double>(u_alpha.second);
    alpha_meas_ =  static_cast<double>(u_alpha.first);

    // Apply PID motor control
    if (current_state_ == State::U_CONTROL) {
        u_PID_.setSetpoint(u_ref_);
        double u_output = u_PID_.compute(u_alpha.second);
        jointVariableVelocity.first = jointVariableVelocity.first + u_output;
    } else if (current_state_ == State::PHI_CONTROL) {
        phi_PID_.setSetpoint(phi_ref_);
        double phi_output = phi_PID_.compute(roll_);     // Compute based on roll
        // RCLCPP_INFO(this->get_logger(), "Ref, Roll, Cmd  %.2f, %.2f, %.2f", phi_ref_, roll_, phi_output);
        jointVariableVelocity.first = jointVariableVelocity.first + phi_output;
    }
    
    // Apply hard constraintss
    std::pair<bool, bool> limit_switch_states;
    arduino.readLimitSwitches(&limit_switch_states);
    if (limit_switch_states.first || limit_switch_states.second) {
        // RCLCPP_INFO(this->get_logger(), "Limit Switched Toggled");
        motor.setMotorASpeed(0);
        motor.setMotorBSpeed(0);
    } else {
        motor.setMotorASpeed(jointVariableVelocity.first);
        motor.setMotorBSpeed(jointVariableVelocity.second);
    }

    // Read and send  motor speed
    float motorASpeed;
    float motorBSpeed;

    motor.readMotorASpeed(&motorASpeed);
    motor.readMotorBSpeed(&motorBSpeed);

    std::pair<float,float> u_alpha_dot = computeJointVariablesInverse(motorASpeed, motorBSpeed);
    u_alpha_dot.second = u_alpha_dot.second * (1000 / CPR); // convert to mm/s
    udot_meas_ =  static_cast<double>(u_alpha_dot.second);
    alphadot_meas_ =  static_cast<double>(u_alpha_dot.first); // in rad/s
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}