#include "../include/motor_control/motor_control_node.hpp"

MotorControlNode::MotorControlNode() : 
  Node("motor_control"), 
  u_PID_(Kp_, Ki_, Kd_),
  motor(MOTORS_I2C_ADDR),
  arduino(AUX_ARDUINO_I2C_ADDR),
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
        "motor/joint_vars", 10,
        std::bind(&MotorControlNode::jointTrajectoryCallback, this, std::placeholders::_1));

    // Publisher for JointTrajectoryControllerState messages
    joint_state_pub_ = this->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
        "motor/joint_vars_state", 10);

    // Timer for control loop at 15Hz (75ms interval)
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(75),
        std::bind(&MotorControlNode::controlLoop, this));
}

// Callback for receiving JointTrajectory message and updating state
void MotorControlNode::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (msg->points.empty() || msg->points[0].positions.size() < 2 || msg->points[0].velocities.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Received invalid JointTrajectory message");
        return;
    }

    // Update joint variables from the message
    u_ref_ = msg->points[0].positions[0];
    alpha_ref_ = msg->points[0].positions[1];
    udot_ref_ = msg->points[0].velocities[0];
    alphadot_ref_ = msg->points[0].velocities[1];

    // Create and populate the JointTrajectoryControllerState message
    control_msgs::msg::JointTrajectoryControllerState state_msg;
    state_msg.joint_names = msg->joint_names;

    // Populate the feedback field with measured values
    state_msg.feedback.positions = {u_meas_, alpha_meas_};
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

    u_PID_.setSetpoint(u_ref_);
  
    double u_output = u_PID_.compute(u_alpha.second);

    // Apply PID motor control for u_meas_
    jointVariableVelocity.first = jointVariableVelocity.first + u_output;

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

    // Read motor speed
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