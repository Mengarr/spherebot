#include "../include/remote_control_cpp/remote_control_node.hpp"

RemoteControlNode::RemoteControlNode()
: Node("remote_control_node"),
  mapped_axes1_(0.0),
  mapped_axes3_(0.0),
  X_BUTTON_(false),
  O_BUTTON_(false),
  prev_x_button_(false),
  prev_o_button_(false),
  motor(MOTORS_I2C_ADDR),
  arduino(AUX_ARDUINO_I2C_ADDR),
  _geodeticConverter()
{   
    RCLCPP_INFO(this->get_logger(), "Motor Init:");
    motor.init();
    arduino.init();
    RCLCPP_INFO(this->get_logger(), "Initalisation Complete");
    // Initialize subscriber to "joy" topic
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        10,
        std::bind(&RemoteControlNode::joyCallback, this, std::placeholders::_1)
    );

    // Initialize subscriber to "gps" topic
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix",
        10,
        std::bind(&RemoteControlNode::gpsCallback, this, std::placeholders::_1)
    );

    // Initialize subscriber to "mag" topic
    mag_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "adjusted_heading",
        10,
        std::bind(&RemoteControlNode::headingCallback, this, std::placeholders::_1)
    );


    // Initialize a timer that runs at 10 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 100 ms interval corresponds to 10 Hz
        std::bind(&RemoteControlNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Remote Control Node has been started.");
}

RemoteControlNode::~RemoteControlNode() {
    motor.reset();
    arduino.reset();
    RCLCPP_INFO(this->get_logger(), "Arduinos Succesfully Reset");
}

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


void RemoteControlNode::timerCallback()
{   
    // if (!init) {
    //         // Set reference GPS position
    //     if (!_gps_fix) {
    //         RCLCPP_INFO(this->get_logger(), "Waiting for GPS fix");
    //         return;
    //     } else {
    //         init = true;
    //         RCLCPP_INFO(this->get_logger(), "GPS Fix!");
    //         _geodeticConverter.setReferenceOrigin(_lat, _long, _alt); // set origin
    //     }
    // } else {
    //     ENU coords = _geodeticConverter.geodeticToENU(_lat, _long, _alt);
    //     RCLCPP_INFO(this->get_logger(), "(x, y): (%.2f, %.2f)", coords.east, coords.north);
    // }

    RCLCPP_INFO(this->get_logger(), "Adjusted Heading: %.2f", heading_);

    float alpha_dot_ref = 0.0;
    float u_dot_ref = 0.0;
    std::pair<bool, bool> limit_switch_states;

    // Apply deadzone to control inputs (SCALE THE MAPPED AXES HERE SO THAT THE OUTPUT is sensible)
    if (fabs(mapped_axes1_) > 5.0) {
        alpha_dot_ref = (mapped_axes1_ / RPM_TO_RPS) / ALPHA_DOT_SCALE_FACTOR;
    }

    if (fabs(mapped_axes3_) > 5.0) {
        u_dot_ref = (mapped_axes3_ / RPM_TO_RPS)  / U_DOT_SCALE_FACTOR;
    }

    // Set motor speeds based on joint variable transformation
    std::pair<float, float> jointVariableVelocity = computeJointVariables(alpha_dot_ref, u_dot_ref);
    
    // Limit switch logic, 
    arduino.readLimitSwitches(&limit_switch_states);
    if (limit_switch_states.first || limit_switch_states.second) {
        RCLCPP_INFO(this->get_logger(), "Limit Switched Toggled");
        motor.setMotorASpeed(0);
        motor.setMotorBSpeed(0);
    } else {
        motor.setMotorASpeed(jointVariableVelocity.first);
        motor.setMotorBSpeed(jointVariableVelocity.second);
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

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

