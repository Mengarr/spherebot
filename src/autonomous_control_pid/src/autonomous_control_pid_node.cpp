#include "../include/remote_control_cpp/remote_control_node.hpp"

AutonomousControlNodePID::AutonomousControlNodePID()
: Node("remote_control_node"),
  X_BUTTON_(false),
  prev_x_button_(false),
  current_state_(STATES::INITIALIZING),
  motor(MOTORS_I2C_ADDR),
  arduino(AUX_ARDUINO_I2C_ADDR)
{   
    RCLCPP_INFO(this->get_logger(), "Motor Init:");
    motor.init();
    RCLCPP_INFO(this->get_logger(), "Auxilary Arduino Init:");
    arduino.init();

    RCLCPP_INFO(this->get_logger(), "Initalisation Complete");
    // Initialize subscriber to "joy" topic
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        10,
        std::bind(&AutonomousControlNodePID::joyCallback, this, std::placeholders::_1)
    );

    // Initialize subscriber to "gps" topic
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix",
        10,
        std::bind(&AutonomousControlNodePID::gpsCallback, this, std::placeholders::_1)
    );

    // Initialize control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((1/controlLoopHZ)*1000), 
        std::bind(&AutonomousControlNodePID::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Autonomous Node has been started.");
}

void AutonomousControlNodePID::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Ensure that there are enough axes and buttons
    if ( msg->buttons.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Received Joy message with insufficient axes or buttons.");
        return;
    }

    // Handle X_BUTTON toggle (buttons[0])
    bool current_x_button = (msg->buttons[0] == 1);
    if (current_x_button && !prev_x_button_) {
        X_BUTTON_ = !X_BUTTON_;
    }
    prev_x_button_ = current_x_button;

}

// void AutonomousControlNodePID::imuCallBack()
// {
     
// }

void magnetometerCallBack(const std_msgs::msg::Float32::SharedPtr msg)
{
    _heading = msg->data;
}

void AutonomousControlNodePID::gpsCallBack()
{
    _lat = msg.latitude;
    _long = msg.longitude;
    _alt = msg.altitude;
    if (msg.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
        _gps_fix = true;
    } else {
        _gps_fix = false;
    }
}

void AutonomousControlNodePID::controlLoop()
{  
    // Check limit switches havent been toggled and that the measured u isnt too close to limit.
    std::pair<float, float> u_alpha = motor.get_u_alpha();
    if (fabs(u_alpha.first)*1000 > U_LIM) {
        RCLCPP_INFO(this->get_logger(), "U Limit reached!");
        // set motor speed to zero
    }

    nextStateLogic();
}

// State machine with the following states
// Initalising / zeroing state: to zero the c drive back to zero, will transsition to next state on button press
// Path Following state: will follow given path autonomously, will transition to a sample state given a close enough proximity to a sample waypoint
// Sample state: samples soil moisture
// Finish State: All goal positions reached, node will exit
void AutonomousControlNodePID::nextStateLogic() {
    switch (current_state_) {
        case STATES::INITIALIZING:
            // Load json data and update stanley controller
            _pathData.loadJsonData(_jsonfn);
            std::vector<double> x_coords = _pathData.getXCoords();
            std::vector<double> y_coords = _pathData.getYCoords();
            std::vector<int> probing_indicies = _pathData.getProbingIndices();
            _stanley.updateWaypoints(x_coords, y_coords);

            // Set reference GPS position
            while (!_gps_fix) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                RCLCPP_INFO(this->get_logger(), "Waiting for GPS fix");
            }
            RCLCPP_INFO(this->get_logger(), "GPS Fix!");
            _geodeticConverter.setReferenceOrigin(_lat, _long, _alt); // set origin

        case STATES::PATH_FOLLOWING:
            // Determine current x and y coordinates:
            ENU coords = _geodeticConverter.geodeticToENU(_lat, _long, _alt);

            double heading_error;
            double steering_angle;
            // Go foward at set velocity, check limit switches, apply PID loop for u
            if (!computeSteering(coords.east, coords.north, _heading, _vf, steering_angle, heading_error)) {
                RCLCPP_INFO(this->get_logger(), "Stanley Controller Error has Occured");
            }

            // Compute u_ref
            // use measured u and apply PID control
            
        case STATES::PROBE_SOIL:
            // Go foward slowly until IR sensor comes on, then stop, wait a few secconds, extend and retract probe whilst publishing soil moisuture data, go back to path following state

        case STATES::FINISH:
            
        default:
            current_state_ = STATES::FINISH;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousControlNodePID>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
