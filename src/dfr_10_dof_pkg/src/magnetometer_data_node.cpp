#include "../include/dfr_10_dof_pkg/magnetometer_data_node.hpp"

MagnetometerPublisher::MagnetometerPublisher()
    : Node("imu_data"),
    magnetometer(VCM5883L_ADDRESS),
    lpf_heading_(3), // window size of 3
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
    declinationAngle_ = (12 + (50 / 60.0)) / (180 / PI); // In rad, from https://github.com/DFRobot/DFRobot_QMC5883/blob/master/examples/getCompassdata/getCompassdata.ino

    eSamples_t samples = magnetometer.getSamples();
    eMode_t mm  = magnetometer.getMeasurementMode();
    eDataRate_t dr = magnetometer.getDataRate();

    RCLCPP_INFO(this->get_logger(), "Configuration %d, %d, %d", samples, mm, dr);
    // Note that the declination angle is in rad
    // magnetometer.setDeclinationAngle(declinationAngle_); 
    // magnetometer.setHardIronOffsets(Xoffset_, Yoffset_, Zoffset_);
    //magnetometer.setMeasurementMode(VCM5883L_CONTINOUS);
    //magnetometer.setDataRate(VCM5883L_DATARATE_200HZ);
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

    //RCLCPP_INFO(this->get_logger(), "Mx, My, Mz: %d, %d, %d", magData.XAxis, magData.YAxis, magData.ZAxis);
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
    
    float Mx_uT = -static_cast<float>(magData.XAxis) * conv_factor;
    float My_uT = -static_cast<float>(magData.YAxis) * conv_factor;
    float Mz_uT = -static_cast<float>(magData.ZAxis) * conv_factor;

    std::vector<float> calibratedValues = getCalibratedValues(Mx_uT, My_uT, Mz_uT);

    //RCLCPP_INFO(this->get_logger(), "Mx_u, My_uT, Mz_uT: %.2f, %.2f, %.2f", calibratedValues[0], calibratedValues[1], calibratedValues[2]);

    compensatedHeading_ = tilt_compensated_heading(
        calibratedValues[0],
        calibratedValues[1],
        calibratedValues[2],
        _AccelY,
        _AccelX,
        _AccelZ
    );

    // Low pass filter to remove high frequency noise
    // float filtered_heading = lpf_heading_.filter(compensatedHeading_);

    mag_msg.data = compensatedHeading_;

    //RCLCPP_INFO(this->get_logger(), "Un filtered Heading: %.2f", compensatedHeading_);
    
    // Publish the IMU data
    mag_publisher_->publish(mag_msg);

    // RCLCPP_INFO(this->get_logger(), "Compensated Heading : %.4f", compensatedHeading_);
    // float headingDegrees = magnetometer.getHeadingDegrees();
    // RCLCPP_INFO(this->get_logger(), "Adjusted Heading : %.4f", adjustedHeading);
    // RCLCPP_INFO(this->get_logger(), "Unajusted Heading : %.4f", headingDegrees);
}

float wrapTo360(float value, float m) {
    // Shift the value by subtracting the starting range m
    float mapped_value = value - m;
    
    // Use the fmod function to wrap the mapped value within 360
    mapped_value = fmod(mapped_value, 360.0);
    
    // Adjust if the result is negative to stay within the 0 to 360 range
    if (mapped_value < 0) {
        mapped_value += 360.0;
    }

    return mapped_value;
}

float MagnetometerPublisher::tilt_compensated_heading(float Mx, float My, float Mz, float ax, float ay, float az)
{
    // Small epsilon to prevent division by zero
    const float epsilon = 1e-8;

    // Step 1: Calculate Roll (φ) and Pitch (θ)
    float roll = std::atan2(ay, az + epsilon);
    float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az) + epsilon);
    //RCLCPP_INFO(this->get_logger(), "Pitch, Roll: %.2f, %.2f", pitch * (180 / M_PI), roll*(180 / M_PI));

    // Step 2: Apply Tilt Compensation
    float x_mag_comp = Mx * std::cos(roll) 
                     - My * std::sin(pitch) * std::sin(roll) 
                     + Mz * std::cos(pitch) * std::sin(roll);

    float y_mag_comp = My * std::cos(pitch) + Mz * std::sin(pitch);
    //RCLCPP_INFO(this->get_logger(), "Comp values: x,y : %.2f, %.2f",x_mag_comp,y_mag_comp);
    //    RCLCPP_INFO(this->get_logger(), "declinationAngle_: %.2f", declinationAngle_ * 180 / M_PI);
    // // Step 3: Calculate Heading
    //RCLCPP_INFO(this->get_logger(), "Tilt Heading before additions %.2f", std::atan2(y_mag_comp,x_mag_comp)  * (180.0 / M_PI));
    //RCLCPP_INFO(this->get_logger(), "No Tilt Heading2 %.2f", std::atan2(My,Mx)  * (180.0 / M_PI));
    // Assuming declination_rad includes all necessary offsets
    // Apply rotation matrix to magnetometer components

    float heading_rad = std::atan2(y_mag_comp,-x_mag_comp);
    heading_rad += declinationAngle_;
    // heading_rad -= M_PI;
    float heading_deg = heading_rad * (180.0f / M_PI);
    heading_deg = std::fmod(heading_deg + 180.0f, 360.0f);
    if (heading_deg < 0)
        heading_deg += 360.0f;
    heading_deg -= 180.0f;


    return heading_deg;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MagnetometerPublisher>());
    rclcpp::shutdown();
    return 0;
}
