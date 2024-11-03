#include "../include/autonomous_control_pid/autonomous_control_pid_node.hpp"

AutonomousControlNodePID::AutonomousControlNodePID()
: Node("autonomous_control_node"),
  _stanley(),
  _pathData("/home/rohan/spherebot/src/autonomous_control_pid/data/path_north.json"),
  current_state_(STATES::INITIALIZING),
  next_state_(STATES::INITIALIZING),
  X_BUTTON_(false),
  prev_x_button_(false)
{   
    // Declare parameters with default variables, K, K_s_, L_, tolerance_, Kp_u_, Ki_u_, Kd_u_, Kp_phi_, Ki_phi_, Kd_phi, control_u_ (bool)
    this->declare_parameter<bool>("control_u_", true); // True for controling u, false to control using phi_
    this->declare_parameter<double>("k_", 0.1);
    this->declare_parameter<double>("k_s_", 0.01);
    this->declare_parameter<double>("L_", 1);
    this->declare_parameter<double>("tolerance_", 4);

    this->declare_parameter<float>("Kp_u_", 0.1);
    this->declare_parameter<float>("Ki_u_", 0.06);
    this->declare_parameter<float>("Kd_u_", 0.0);
    this->declare_parameter<float>("Kp_phi_", 2);
    this->declare_parameter<float>("Ki_phi_", 0.1);
    this->declare_parameter<float>("Kd_phi_", 0.0);

    this->declare_parameter<float>("foward_roll_timeout", 4);
    this->declare_parameter<float>("foward_velocity", 60);
    

    // Retrieve parameters
    this->get_parameter("control_u_", control_u_); // True for controling u, false to control using phi_
    this->get_parameter("k_", k_);
    this->get_parameter("k_s_", k_s_);
    this->get_parameter("L_", L_);
    this->get_parameter("tolerance_", tolerance_);

    this->get_parameter("Kp_u_", Kp_u_);
    this->get_parameter("Ki_u_", Ki_u_);
    this->get_parameter("Kd_u_", Kd_u_);
    this->get_parameter("Kp_phi_", Kp_phi_);
    this->get_parameter("Ki_phi_", Ki_phi_);
    this->get_parameter("Kd_phi_", Kd_phi_);

    this->get_parameter("foward_roll_timeout", foward_roll_timeout_);
    this->get_parameter("foward_velocity", foward_vel_);
    

    // Print all parameters and their values
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "control_u_: %s", control_u_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "k_: %f", k_);
    RCLCPP_INFO(this->get_logger(), "k_s_: %f", k_s_);
    RCLCPP_INFO(this->get_logger(), "L_: %f", L_);
    RCLCPP_INFO(this->get_logger(), "tolerance_: %f", tolerance_);

    RCLCPP_INFO(this->get_logger(), "Kp_u_: %f", Kp_u_);
    RCLCPP_INFO(this->get_logger(), "Ki_u_: %f", Ki_u_);
    RCLCPP_INFO(this->get_logger(), "Kd_u_: %f", Kd_u_);

    RCLCPP_INFO(this->get_logger(), "Kp_phi_: %f", Kp_phi_);
    RCLCPP_INFO(this->get_logger(), "Ki_phi_: %f", Ki_phi_);
    RCLCPP_INFO(this->get_logger(), "Kd_phi_: %f", Kd_phi_);

    RCLCPP_INFO(this->get_logger(), "Fowar Vel: %f", foward_vel_);
    RCLCPP_INFO(this->get_logger(), "Foward timeout: %f", foward_roll_timeout_);

    // Stanley controller ctor
    _stanley.setParams(k_, k_s_, L_, tolerance_);

    alphadot_ref_ = (foward_vel_ / RPM_TO_RPS) / ALPHA_DOT_SCALE_FACTOR;

    // Motor control state
    motor_state_ = (control_u_ == true) ?  MotorState::U_CONTROL :  MotorState::PHI_CONTROL;

    // Initialize subscriber to "joy" topic
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy",
        2,
        std::bind(&AutonomousControlNodePID::joyCallback, this, std::placeholders::_1)
    );

    // Initialize subscriber to gps latlong topic
    gps_latlong_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps/fix",
        2,
        std::bind(&AutonomousControlNodePID::gpsLatLongCallBack, this, std::placeholders::_1)
    );

    // Initialize subscriber to gps coords topic
    gps_coords_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "gps/point_data",
        2,
        std::bind(&AutonomousControlNodePID::gpsCoordsCallBack, this, std::placeholders::_1)
    );

    // Initalize subscriber to heading topic
    mag_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "compensated_heading",
        2,
        std::bind(&AutonomousControlNodePID::headingCallback, this, std::placeholders::_1)
    );

    // Initalize subscriber to joint trajectory state
    joint_trajectory_state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "motor/joint_vars_state",
        2,
        std::bind(&AutonomousControlNodePID::jointTrajectoryStateCallback, this, std::placeholders::_1)
    );

    // Initalize publisher to joint trajectory 
    joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "motor/joint_vars", 1
    );

    // Initalize publisher to joint trajectory 
    stanley_heading_ref_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "heading_ref", 1
    );
        // 
    state_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        "motor/motor_control_state", 1
    );

    
    // Initialize control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), 
        std::bind(&AutonomousControlNodePID::controlLoop, this)
    );

    // Populate the params map with values
    params["g"] = -9.81f;     // Gravitational constant
    params["r"] = 0.05f;      // CoM to x axes (in m)
    params["R"] = 0.220f;    // Shell Radius 
    params["Is"] = 1.0f;     // Moment of inertia of the shell
    params["mp"] = 5.5f;     // Mass of inernal pendulum
    params["ms"] = 1.8f;     // Mass of sphere

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

// void AutonomousControlNodePID::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
//     float AccelX = msg->linear_acceleration.x;
//     float AccelY = msg->linear_acceleration.y;
//     float AccelZ = msg->linear_acceleration.z;

//     const float epsilon = 1e-8; // To avoid div by 0
//     float roll = std::atan2(AccelX, -AccelZ + epsilon);
//     float pitch = std::atan2(-AccelY, std::sqrt(AccelX * AccelX + (-AccelZ) * (-AccelZ)) + epsilon);
    
//     // Low pass filter
//     std::vector<float> filtered_roll_pitch = lpf_roll_pitch_.filter({roll, pitch});
//     _roll = filtered_roll_pitch[0];
//     _pitch = filtered_roll_pitch[1];
// }

void AutonomousControlNodePID::headingCallback(const std_msgs::msg::Float32::SharedPtr msg)
{   
    _heading = msg->data;
}

void AutonomousControlNodePID::gpsLatLongCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    _gps_fix = (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX);
}

void AutonomousControlNodePID::gpsCoordsCallBack(const geometry_msgs::msg::Point::SharedPtr msg)
{
    _pos_x = msg->x;
    _pos_y = msg->y;
    _pos_z = msg->z;
}

void AutonomousControlNodePID::jointTrajectoryStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
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

void AutonomousControlNodePID::controlLoop()
{  
    if (fabs(u_meas_) > 42) {
        alphadot_ref_ = 0.0;
        u_ref_ = 0.0;
        publishJointTrajectory(); // Command Motors
        RCLCPP_WARN(this->get_logger(), "Motor soft limits reached. haulting robot");
    } else {
        nextStateLogic();
    }
}

void AutonomousControlNodePID::publishJointTrajectory()
{
    // Create the JointTrajectory message
    auto joint_vars_msg = trajectory_msgs::msg::JointTrajectory();

    // Set the joint names for each joint
    joint_vars_msg.joint_names = {"c_drive"}; 


    // Create a JointTrajectoryPoint to hold the positions and velocities
    trajectory_msgs::msg::JointTrajectoryPoint point;

    // Set positions for each joint
    point.positions.push_back(u_ref_*1000);         // u_ref_ value, convert to mm from m
    point.positions.push_back(0.0);            // alpha_ref value (not used)

    // Set velocities for each joint
    point.velocities.push_back(0.0);           // Velocity of joint u, must be 0.0
    point.velocities.push_back(-alphadot_ref_); // Velocity of joint alpha

    // Add PID efforts
    point.effort.push_back(Kd_u_);            // Kd
    point.effort.push_back(Ki_u_);           // Ki
    point.effort.push_back(Kp_u_);            // Kp

    // Add the point to the JointTrajectory message
    joint_vars_msg.points.push_back(point);

    // Create a JointTrajectoryPoint to hold the positions and velocities
    trajectory_msgs::msg::JointTrajectoryPoint point_phi;

    // Set positions for each joint
    point_phi.positions.push_back(phi_ref_);         // u_ref_ value
    point_phi.positions.push_back(0.0);            // alpha_ref value (not used)

    // Set velocities for each joint
    point_phi.velocities.push_back(0.0);           // Velocity of joint u, must be 0.0
    point_phi.velocities.push_back(-alphadot_ref_); // Velocity of joint alpha

    // Add PID efforts
    point_phi.effort.push_back(Kd_phi_);            // Kd
    point_phi.effort.push_back(Ki_phi_);           // Ki
    point_phi.effort.push_back(Kp_phi_);            // Kp

    // Add the point to the JointTrajectory message
    joint_vars_msg.points.push_back(point_phi);
    joint_trajectory_pub_->publish(joint_vars_msg);

    std_msgs::msg::Int8 state_msg;
    state_msg.data = static_cast<int8_t>(motor_state_);
    state_pub_->publish(state_msg);
}

void AutonomousControlNodePID::nextStateLogic() {
    static auto startTime = std::chrono::steady_clock::now();

    switch (current_state_) {
        case STATES::INITIALIZING: {
            // Load json data and update stanley controller
            std::vector<double> x_coords = _pathData.getXCoords();
            std::vector<double> y_coords = _pathData.getYCoords();
            std::vector<int> probing_indicies = _pathData.getProbingIndices();
            _stanley.updateWaypoints(x_coords, y_coords);

            // Set reference GPS position
            auto currentTime = std::chrono::steady_clock::now();
            std::chrono::duration<float> elapsed = currentTime - startTime;
            if (!_gps_fix && (elapsed.count() > 2)) {
                // Send message ever 2 secconds
                RCLCPP_INFO(this->get_logger(), "Waiting for GPS fix");
                startTime = std::chrono::steady_clock::now();
            } else if (_gps_fix && (elapsed.count() > 2)) {
                RCLCPP_INFO(this->get_logger(), "GPS Fix!");
                startTime = std::chrono::steady_clock::now();
            }

            if (_gps_fix && X_BUTTON_){ // If button is pressed and gps fix then start!
                next_state_ = STATES::PATH_FOLLOWING;
                RCLCPP_INFO(this->get_logger(), "Path Following Begining!");
                startTime = std::chrono::steady_clock::now();
            }
            break;
        }
        case STATES::PATH_FOLLOWING: {

            auto currentTime = std::chrono::steady_clock::now();
            std::chrono::duration<float> elapsed = currentTime - startTime;
            if (elapsed.count() < foward_roll_timeout_){
                phi_ref_ = 0.0;
                next_state_ = STATES::PATH_FOLLOWING;
                publishJointTrajectory();
                break;
            }

            // Compute steering angle ---
            double heading_error;
            double steering_angle;

            //double wrappedHeading =  static_cast<double>(wrapToPi(_heading * (M_PI / 180))); // convert to rad and wrap to +- 180 deg

            // Go foward at set velocity, assumes alphadot_meas_ = theatadot_meas_
            if (!_stanley.computeSteering(_pos_x, _pos_y, _heading * (M_PI / 180),  params.at("R") * alphadot_meas_, steering_angle, heading_error)) {
                RCLCPP_ERROR(this->get_logger(), "Stanley Controller Error has Occured");
            }
            auto stanley_msg = std_msgs::msg::Float32();
            stanley_msg.data = static_cast<float>(steering_angle);
            stanley_heading_ref_pub_->publish(stanley_msg);

            RCLCPP_INFO(this->get_logger(), "Heading Error %.2f", heading_error * (180/M_PI));
            // Determine Reference u ---
            float rc =  - params.at("R") * alphadot_meas_ / steering_angle;                 // Compute radius of curviture
            u_ref_ = calculate_u_ref(0.0, 0.0, static_cast<float>(alphadot_meas_), rc, params);               // Compute reference u
            phi_ref_ = u_phi_eq(u_ref_, params.at("r"));
            // phi_ref_: 
            RCLCPP_INFO(this->get_logger(), "Phi REF: %.2f", phi_ref_ * (180/M_PI));
            publishJointTrajectory(); // Command Motors

            // Check if the bot has reached the final waypoint
            if (_stanley.reachedFinal(_pos_x, _pos_y)){
                next_state_ = STATES::FINISH;
                startTime = std::chrono::steady_clock::now();
                RCLCPP_INFO(this->get_logger(), "Final Position Reached!");
            } else {
                next_state_ = STATES::PATH_FOLLOWING;
            }
            break;
        }
        case STATES::PROBE_SOIL: {
            // Go foward slowly until IR sensor comes on, then stop, wait a few secconds, extend and retract probe whilst publishing soil moisuture data, go back to path following state
            break;
        }
        case STATES::FINISH: {
            // Stop the motors and hang the process
            auto currentTime = std::chrono::steady_clock::now();
            std::chrono::duration<float> elapsed = currentTime - startTime;
            if (elapsed.count() > 2) {
                RCLCPP_INFO(this->get_logger(), "Final Waypoint Reached!");
                startTime = std::chrono::steady_clock::now();
            }
            alphadot_ref_ = 0.0;
            u_ref_ = 0.0;
            publishJointTrajectory(); // Command Motors
            next_state_ = STATES::FINISH;
            break;
        }
        default: {
            next_state_ = STATES::FINISH;
        }
    }

    current_state_ = next_state_;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousControlNodePID>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


        // case STATES::CALIBRATE_PHI:
        //     auto currentTime = std::chrono::steady_clock::now()
        //     std::chrono::duration<float> elapsed = currentTime - startTime;
        //     if (_roll < phi_tolerance && elapsed.count() > t_hold_phi)
        //     {   
        //         std::pair<float, float> u_alpha = getUAlpha();
        //         u_calib = u_alpha.first;
        //         RCLCPP_INFO(this->get_logger(), "Phi calibrated with u_calib: %.2f", u_calib);
        //         next_state_ = STATES::PATH_FOLLOWING;
        //         break;
        //     }
        //     double output = phi_PID_.compute(_roll);
        //     setSafeMotorSpeed(output, 0.0);
        //     next_state_ = STATES::CALIBRATE_PHI;
        // case STATES::CALIBRATE_U:
        //     break;