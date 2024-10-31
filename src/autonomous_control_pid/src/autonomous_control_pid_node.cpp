#include "../include/remote_control_cpp/remote_control_node.hpp"

AutonomousControlNodePID::AutonomousControlNodePID()
: Node("remote_control_node"),
  _stanley(k_, k_s_, L_, tolerance_),
  _pathData("/home/rohan/spherebot/src/autonomous_control_pid/data/test_path.json"),
  _geodeticConverter(),
  u_PID_(Kp_u_, Ki_u_, Kd_u_),
  phi_PID_(Kp_phi_, Ki_phi_, Kd_phi_),
  X_BUTTON_(false),
  prev_x_button_(false),
  current_state_(STATES::INITIALIZING),
  next_state_(STATES::INITIALIZING),
  motor(MOTORS_I2C_ADDR),
  arduino(AUX_ARDUINO_I2C_ADDR),
  lpf_roll_pitch_(0.5f, static_cast<size_t>(2)),
  lpf_heading_(0.5f, static_cast<size_t>(1))
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

    // Initalize subscriber to heading topic
    mag_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "compensated_heading",
        10,
        std::bind(&AutonomousControlNodePID::headingCallback, this, std::placeholders::_1)
    );

    // Initalize subscriber to imu topic
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        10,
        std::bind(&AutonomousControlNodePID::imuCallback, this, std::placeholders::_1)
    );

    // Initialize control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds((1/controlLoopHZ)*1000), 
        std::bind(&AutonomousControlNodePID::controlLoop, this)
    );

    // PID controllers limits
    u_PID_.setOutputLimits(-0.7, 0.7);
    phi_PID_.setOutputLimits(-0.7, 0.7);

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

void AutonomousControlNodePID::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    float AccelX = msg->linear_acceleration.x;
    float AccelY = msg->linear_acceleration.y;
    float AccelZ = msg->linear_acceleration.z;

    const float epsilon = 1e-8; // To avoid div by 0
    float roll = std::atan2(AccelX, -AccelZ + epsilon);
    float pitch = std::atan2(-AccelY, std::sqrt(AccelX * AccelX + (-AccelZ) * (-AccelZ)) + epsilon);
    
    // Low pass filter
    std::vector<float> filtered_roll_pitch = lpf_roll_pitch_.filter({roll, pitch});
    _roll = filtered_roll_pitch[0];
    _pitch = filtered_roll_pitch[1];
}

void AutonomousControlNodePID::headingCallback(const std_msgs::msg::Float32::SharedPtr msg)
{   
    std::vector<float> filtered_heading = lpf_heading_.filter({msg->data});
    _heading = filtered_heading[0];
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
    // std::pair<float, float> u_alpha = motor.get_u_alpha();
    // if (fabs(u_alpha.first)*1000 > U_LIM) {
    //     RCLCPP_INFO(this->get_logger(), "U Limit reached!");
    //     // set motor speed to zero
    // }

    nextStateLogic();
}

std::pair<float, float> AutonomousControlNodePID::getUAlpha()
{
    // Returns pair of floats corresponding to u, alpha in (mm), rad respectively
    int32_t motorACount;
    int32_t motorBCount;
    motor.readMotorACount(&motorACount);
    motor.readMotorBCount(&motorBCount);
    std::pair<float,float> alpha_u = computeJointVariablesInverse(motorACount, motorBCount);
    alpha_u.second = alpha_u.second * (1000 / CPR); // convert to mm
    return std::make_pair(alpha_u.second, alpha_u.first);
}

// State machine with the following states
// Initalising / zeroing state: to zero the c drive back to zero, will transsition to next state on button press
// Path Following state: will follow given path autonomously, will transition to a sample state given a close enough proximity to a sample waypoint
// Sample state: samples soil moisture
// Finish State: All goal positions reached, node will exit
void AutonomousControlNodePID::nextStateLogic() {
    static auto startTime = std::chrono::steady_clock::now();
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

            // Calibrate Phi and reccord u
            next_state_ = STATES::CALIBRATE_PHI;
            startTime = std::chrono::steady_clock::now()

        case STATES::CALIBRATE_PHI:
            auto currentTime = std::chrono::steady_clock::now()
            std::chrono::duration<float> elapsed = currentTime - startTime;
            if (_roll < phi_tolerance && elapsed.count() > t_hold_phi)
            {   
                std::pair<float, float> u_alpha = getUAlpha();
                u_calib = u_alpha.first;
                RCLCPP_INFO(this->get_logger(), "Phi calibrated with u_calib: %.2f", u_calib);
                next_state_ = STATES::PATH_FOLLOWING;
                break;
            }
            double output = phi_PID_.compute(_roll);
            setSafeMotorSpeed(output, 0.0);
            next_state_ = STATES::CALIBRATE_PHI;
        case STATES::CALIBRATE_U:
            break;

        case STATES::PATH_FOLLOWING:
            // Determine current x and y coordinates:
            ENU coords = _geodeticConverter.geodeticToENU(_lat, _long, _alt);

            // Determine dtheta ----
            float motorASpeed;
            float motorBSpeed;

            motor.readMotorASpeed(&motorASpeed);
            motor.readMotorBSpeed(&motorBSpeed);

            std::pair<float, float> jointVariableVelocity = computeJointVariablesInverse(motorASpeed, motorBSpeed);
            float dalpha = jointVariableVelocity.first;
            float dtheta = dalpha; // Assume dtheta == dalpha

            // Compute steering angle ---
            double heading_error;
            double steering_angle;
            // Go foward at set velocity, check limit switches, apply PID loop for u
            if (!_stanley.computeSteering(coords.east, coords.north, _heading,  params.at("R") * dtheta, steering_angle, heading_error)) {
                RCLCPP_ERROR(this->get_logger(), "Stanley Controller Error has Occured");
            }

            // Calculate reference foward velocity ---
            alpha_dot_ref = (alpha_dot_ref / RPM_TO_RPS) / ALPHA_DOT_SCALE_FACTOR;
            float u_dot_ref = 0.0; // not relevent here as this is just for foward velocity control

            // Set motor speeds based on joint variable transformation
            std::pair<float, float> jointVariableVelocity = computeJointVariables(alpha_dot_ref, u_dot_ref);

            // Determine Reference u ---
            float rc =  params.at("R") * dtheta / steering_angle;                 // Compute radius of curviture
            float uref = calculate_u_ref(0.0, 0.0, dtheta, rc, params);         // Compute reference u

            // use measured u and apply PID control
            std::pair<float, float> u_alpha = getUAlpha();
            u_calib = u_alpha.first;
            u_PID_.setSetpoint(uref - u_calib);
            double u_output = u_PID_.compute(u_alpha.second);

            jointVariableVelocity.first += u_output; // Add controller foward velocity

            // Remove noisy signals from controllers
            if (fabs(jointVariableVelocity.first) < 0.08) {
                jointVariableVelocity.first = 0.0;
            }
            
            if (fabs(jointVariableVelocity.second) < 0.08) {
                jointVariableVelocity.second = 0.0;
            }

            setSafeMotorSpeed(jointVariableVelocity.first, jointVariableVelocity.second);
            
            // Check if the bot has reached the final waypoint
            if (_stanley.reachedFinal(coords.east, coords.north)){
                next_state_ = STATES::FINISH;
                startTime = std::chrono::steady_clock::now()
                RCLCPP_INFO(this->get_logger(), "Final Position Reached!");
            } else {
                next_state_ = STATES::PATH_FOLLOWING;
            }
        case STATES::PROBE_SOIL:
            // Go foward slowly until IR sensor comes on, then stop, wait a few secconds, extend and retract probe whilst publishing soil moisuture data, go back to path following state
            break;
        case STATES::FINISH:
            // Stop the motors and hang the process

        default:
            current_state_ = STATES::FINISH;
    }

    current_state_ = next_state_;
}


void AutonomousControlNodePID::setSafeMotorSpeed(float phi_L, float phi_R) {
    arduino.readLimitSwitches(&limit_switch_states);
    if (limit_switch_states.first || limit_switch_states.second) {
        // RCLCPP_INFO(this->get_logger(), "Limit Switched Toggled");
        motor.setMotorASpeed(0);
        motor.setMotorBSpeed(0);
    } else {
        motor.setMotorASpeed(phi_L);
        motor.setMotorBSpeed(phi_R);
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
