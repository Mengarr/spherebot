#include "../include/dfr_10_dof_pkg/magnetometer_data_node.hpp"

MagnetometerPublisher::MagnetometerPublisher()
    : Node("imu_data"),
    magnetometer(VCM5883L_ADDRESS),
    _AccelX(0),
    _AccelY(0),
    _AccelZ(1)
{   
    RCLCPP_INFO(this->get_logger(), "Magnetometer Node has been started");

    // publisher
    mag_publisher_ = this->create_publisher<std_msgs::msg::Float32>("compensated_heading", 10);

    // Timer for data publishing
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), // 20 Hz
    std::bind(&MagnetometerPublisher::publish_data, this));

    // Subscriber
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        10,
        std::bind(&MagnetometerPublisher::imuCallback, this, std::placeholders::_1)
    );

    // magnetometer data
    RCLCPP_INFO(this->get_logger(), "Initalising magnetometer");
    magnetometer.init();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Sleep for one seccond
    declinationAngle_ = (12 + (50 / 60.0)) / (180 / PI); // from https://github.com/DFRobot/DFRobot_QMC5883/blob/master/examples/getCompassdata/getCompassdata.ino
    // Note that the declination angle is in rad
    // magnetometer.setDeclinationAngle(declinationAngle_); 
    // magnetometer.setHardIronOffsets(Xoffset_, Yoffset_, Zoffset_);
    magnetometer.setMeasurementMode(VCM5883L_CONTINOUS);
    magnetometer.setDataRate(VCM5883L_DATARATE_200HZ);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Sleep for one seccond
    RCLCPP_INFO(this->get_logger(), "Magnometer Node Initalisation Complete");
}

void MagnetometerPublisher::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    _AccelX = msg->linear_acceleration.x;
    _AccelY = msg->linear_acceleration.y;
    _AccelZ = msg->linear_acceleration.z;
}

std::vector<float> MagnetometerPublisher::getCalibratedValues(float Mx, float My, float Mz)
{
    // Expects magnetometer values in uT!

    float xm_off = Mx - hard_iron_bias_x;
    float ym_off = My - hard_iron_bias_y;
    float zm_off = Mz - hard_iron_bias_z;

    float xm_cal = xm_off * soft_iron_bias_xx + ym_off * soft_iron_bias_yx + zm_off * soft_iron_bias_zx;
    float ym_cal = xm_off * soft_iron_bias_xy + ym_off * soft_iron_bias_yy + zm_off * soft_iron_bias_zy;
    float zm_cal = xm_off * soft_iron_bias_xz + ym_off * soft_iron_bias_yz + zm_off * soft_iron_bias_zz;

    return {xm_cal, ym_cal, zm_cal};
}


void MagnetometerPublisher::publish_data()
{
    auto mag_msg = std_msgs::msg::Float32();
    // Magnometer data
    sVector_t magData = magnetometer.readRaw();

    const int16_t INVALID_MAG_VALUE = -4096;

    // Check for invalid magnetometer values
    if (magData.XAxis == INVALID_MAG_VALUE || magData.YAxis == INVALID_MAG_VALUE || magData.ZAxis == INVALID_MAG_VALUE)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid magnetometer reading, publishing last valid value");

        // If no valid magnetometer data has been received yet, return 0.0f
        mag_msg.data = compensatedHeading_;

        // Publish the IMU data
        mag_publisher_->publish(mag_msg);

        return;
    } 

    // Magnetometer values in uT
    float Mx_uT = static_cast<float>(magData.XAxis) * conv_factor;
    float My_uT = static_cast<float>(magData.YAxis) * conv_factor;
    float Mz_uT = static_cast<float>(magData.ZAxis) * conv_factor;

    std::vector<float> calibratedValues = getCalibratedValues(Mx_uT, My_uT, Mz_uT);

    compensatedHeading_ = tilt_compensated_heading(
        calibratedValues[0],
        calibratedValues[1],
        calibratedValues[2],
        _AccelY,
        _AccelX,
        -_AccelZ
    );

    RCLCPP_INFO(this->get_logger(), "Compensated Heading : %.4f", compensatedHeading_);

    // float headingDegrees = magnetometer.getHeadingDegrees();

    // RCLCPP_INFO(this->get_logger(), "Adjusted Heading : %.4f", adjustedHeading);
    // RCLCPP_INFO(this->get_logger(), "Unajusted Heading : %.4f", headingDegrees);

    mag_msg.data = compensatedHeading_;
    
    // Publish the IMU data
    mag_publisher_->publish(mag_msg);
}

float MagnetometerPublisher::tilt_compensated_heading(float Mx, float My, float Mz, float ax, float ay, float az)
{
    // Small epsilon to prevent division by zero
    const float epsilon = 1e-8;

    // Step 1: Calculate Roll (φ) and Pitch (θ)
    float roll = std::atan2(ay, az + epsilon);
    float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az) + epsilon);

    // Step 2: Apply Tilt Compensation
    float x_mag_comp = Mx * std::cos(roll) 
                     - My * std::sin(pitch) * std::sin(roll) 
                     + Mz * std::cos(pitch) * std::sin(roll);

    float y_mag_comp = My * std::cos(pitch) + Mz * std::sin(pitch);

    // Step 3: Calculate Heading
    float heading_rad = std::atan2(y_mag_comp, x_mag_comp);

    // Step 4: Add magnetic declination
    heading_rad += declinationAngle_;

    // Convert heading from radians to degrees
    float heading_deg = heading_rad * (180.0 / M_PI);

    // Normalize heading to 0-360 degrees
    if (heading_deg < 0) {
        heading_deg += 360;
    }

    return heading_deg;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MagnetometerPublisher>());
    rclcpp::shutdown();
    return 0;
}


// float MagnetometerPublisher::calculateAdjustedHeading(int16_t mag_x, int16_t mag_y, int16_t mag_z,
//                                                      float acc_x, float acc_y, float acc_z)
// {
//     // Static variables to store the last valid magnetometer readings
//     static float last_mag_x = 0.0f;
//     static float last_mag_y = 0.0f;
//     static float last_mag_z = 0.0f;
//     static bool has_last_valid_mag = false;

//     const int16_t INVALID_MAG_VALUE = -4096;

//     // Check for invalid magnetometer values
//     if (mag_x == INVALID_MAG_VALUE || mag_y == INVALID_MAG_VALUE || mag_z == INVALID_MAG_VALUE)
//     {
//         std::cerr << "Invalid magnetometer value detected. Using last valid value.\n";
//         if (!has_last_valid_mag)
//         {
//             // If no valid magnetometer data has been received yet, return 0.0f
//             return 0.0f;
//         }
//         // Else, retain the last valid magnetometer values
//     }
//     else
//     {
//         // Update last valid magnetometer readings by applying hard iron offsets
//         last_mag_x = static_cast<float>(mag_x) - Xoffset_;
//         last_mag_y = static_cast<float>(mag_y) - Yoffset_;
//         last_mag_z = static_cast<float>(mag_z) - Zoffset_;
//         has_last_valid_mag = true;
//     }

//     float ax = acc_x;
//     float ay = acc_y;
//     float az = acc_z;

//     // Prevent division by zero in pitch calculation
//     float denominator = std::sqrt(ay * ay + az * az);
//     if (denominator == 0.0f)
//     {
//         std::cerr << "Invalid accelerometer data: denominator is zero.\n";
//         return 0.0f;
//     }

//     // Calculate roll (φ) and pitch (θ) angles in radians
//     float pitch = std::atan2(ay, az);
//     float roll = std::atan(-ax / denominator);
//     // RCLCPP_INFO(this->get_logger(), "Roll, Pitch, ax: %.2f, %.2f, %.2f", roll*180/M_PI, pitch*180/M_PI, ax);

//     pitch = 00;
//     roll = 0.0;

//     // Calculate sine and cosine of roll and pitch for reuse
//     float sin_roll = std::sin(roll);
//     float cos_roll = std::cos(roll);
//     float sin_pitch = std::sin(pitch);
//     float cos_pitch = std::cos(pitch);

//     // Compensate magnetometer readings for tilt (roll and pitch)
//     float mag_x_comp = last_mag_x * cos_pitch + last_mag_z * sin_pitch;
//     float mag_y_comp = last_mag_x * sin_roll * sin_pitch +
//                        last_mag_y * cos_roll -
//                        last_mag_z * sin_roll * cos_pitch;

//     // Compute the heading in radians using the compensated magnetometer readings
//     float heading_rad = std::atan2(mag_y_comp, mag_x_comp);
//     heading_rad += declinationAngle_; // Add declination angle

//     // Convert heading from radians to degrees
//     float heading_deg = heading_rad * (180.0f / static_cast<float>(M_PI));


//     // Normalize heading to 0° - 360°
//     if (heading_deg < 0.0f)
//     {
//         heading_deg += 360.0f;
//     }
//     else if (heading_deg >= 360.0f)
//     {
//         heading_deg -= 360.0f;
//     }

//     return heading_deg;
// }
