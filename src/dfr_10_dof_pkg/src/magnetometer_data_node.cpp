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
    mag_publisher_ = this->create_publisher<std_msgs::msg::Float32>("adjusted_heading", 10);

    // Timer for data publishing
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), // 20 Hz
    std::bind(&MagnetometerPublisher::publish_data, this));

    // magnetometer data
    RCLCPP_INFO(this->get_logger(), "Initalising magnetometer");
    magnetometer.init();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Sleep for one seccond
    float declinationAngle = (12 + (50 / 60.0)) / (180 / PI); // from https://github.com/DFRobot/DFRobot_QMC5883/blob/master/examples/getCompassdata/getCompassdata.ino
    magnetometer.setDeclinationAngle(declinationAngle);
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

void MagnetometerPublisher::publish_data()
{
    auto mag_msg = std_msgs::msg::Float32();
    // Magnometer data
    //sVector_t magData = magnetometer.readRaw();
    magnetometer.readRaw();
    // float adjustedHeading = calculateAdjustedHeading(
    //     magData.XAxis,
    //     magData.YAxis,
    //     magData.ZAxis,
    //     _AccelX,
    //     _AccelY,
    //     _AccelZ
    // );
    float headingDegrees = magnetometer.getHeadingDegrees();
    //RCLCPP_INFO(this->get_logger(), "Adjusted Heading : %.4f", adjustedHeading);
    //RCLCPP_INFO(this->get_logger(), "Unajusted Heading : %.4f", headingDegrees);

    mag_msg.data = headingDegrees;
    
    // Publish the IMU data
    mag_publisher_->publish(mag_msg);
}

/**
 * @brief Calculates the adjusted heading based on magnetometer and accelerometer readings.
 *
 * @param mag_x Magnetometer X-axis value (int16_t)
 * @param mag_y Magnetometer Y-axis value (int16_t)
 * @param mag_z Magnetometer Z-axis value (int16_t)

 * @return float Adjusted heading in degrees (0° to 360°)
 */
// 
float MagnetometerPublisher::calculateAdjustedHeading(int16_t mag_x, int16_t mag_y, int16_t mag_z,
                              float acc_x, float acc_y, float acc_z)
{
    // Static variables to store the last valid magnetometer readings
    static float last_mag_x = 0;
    static float last_mag_y = 0;
    static float last_mag_z = 0;
    static bool has_last_valid_mag = false;

    const int16_t INVALID_MAG_VALUE = -4096;

    // Check for invalid magnetometer values
    if (mag_x == INVALID_MAG_VALUE || mag_y == INVALID_MAG_VALUE || mag_z == INVALID_MAG_VALUE)
    {
        std::cerr << "Invalid magnetometer value detected. Using last valid value.\n";
        if (!has_last_valid_mag)
        {
            // If no valid magnetometer data has been received yet, return 0.0f
            return 0.0f;
        }
        // Else, retain the last valid magnetometer values
    }
    else
    {
        // Update last valid magnetometer readings
        last_mag_x = mag_x;
        last_mag_y = mag_y;
        last_mag_z = mag_z;
        has_last_valid_mag = true;
    }
    float ax = acc_x;
    float ay = acc_y;
    float az = acc_z;
    // Prevent division by zero in pitch calculation
    float denominator = std::sqrt(ay * ay + az * az);
    if (denominator == 0.0f)
    {
        std::cerr << "Invalid accelerometer data: denominator is zero.\n";
        return 0.0f;
    }

    // Calculate roll (φ) and pitch (θ) angles in radians
    float roll = std::atan2(ay, az);
    float pitch = std::atan(-ax / denominator);

    // Calculate sine and cosine of roll and pitch for reuse
    float sin_roll = std::sin(roll);
    float cos_roll = std::cos(roll);
    float sin_pitch = std::sin(pitch);
    float cos_pitch = std::cos(pitch);

    // Compensate magnetometer readings for tilt
    float mag_x_comp = static_cast<float>(last_mag_x) * cos_pitch + static_cast<float>(last_mag_z) * sin_pitch;
    float mag_y_comp = static_cast<float>(last_mag_x) * sin_roll * sin_pitch +
                       static_cast<float>(last_mag_y) * cos_roll -
                       static_cast<float>(last_mag_z) * sin_roll * cos_pitch;

    // Compute the heading in radians
    float heading_rad = std::atan2(mag_y_comp, mag_x_comp);

    // Convert heading from radians to degrees
    float heading_deg = heading_rad * (180.0f / static_cast<float>(M_PI));

    // Normalize heading to 0° - 360°
    if (heading_deg < 0.0f)
    {
        heading_deg += 360.0f;
    }
    else if (heading_deg >= 360.0f)
    {
        heading_deg -= 360.0f;
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