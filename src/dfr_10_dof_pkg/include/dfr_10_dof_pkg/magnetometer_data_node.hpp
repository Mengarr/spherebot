#ifndef MAGNOMETER_DATA_NODE_HPP
#define MAGNOMETER_DATA_NODE_HPP

#include "VCM5883L.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "control_lib/LowPassFilter.hpp"
#include "sensor_msgs/msg/imu.hpp"

struct Orientation {
    float roll;   // Rotation around X-axis in radians
    float pitch;  // Rotation around Y-axis in radians
    float yaw;    // Rotation around Z-axis in radians
};

class MagnetometerPublisher : public rclcpp::Node
{
    public:
        MagnetometerPublisher();
    private:
        void publish_data();

        VCM5883L magnetometer;

        float declinationAngle_ = 0.0f; // Initalise to zero

        float compensatedHeading_ = 0.0f;

        float hard_iron_bias_x = -6.432633835053031;
        float hard_iron_bias_y = 5.2253566921278205;
        float hard_iron_bias_z = 3.788405358792561;

        float soft_iron_bias_xx = 2.53884825615019;
        float soft_iron_bias_xy = -0.004782928012666182;
        float soft_iron_bias_xz = -0.037274579675119185;

        float soft_iron_bias_yx = -0.004782928012666306;
        float soft_iron_bias_yy = 2.455924582285725;
        float soft_iron_bias_yz = 0.20887190823645405;

        float soft_iron_bias_zx = -0.037274579675119233;
        float soft_iron_bias_zy = 0.2088719082364538;
        float soft_iron_bias_zz = 2.2948842404204663;

        // conversion factor to convert +-8g to uT
        const float conv_factor = 0.0244140625;

        // Functions
        float tilt_compensated_heading(float Mx, float My, float Mz, float ax, float ay, float az);
        std::vector<float> getCalibratedValues(float Mx, float My, float Mz);

        // Low pass filter
        LowPassFilter lpf_heading_; // (0.3f, static_cast<size_t>(1))

        // Imu stuff
        float _AccelX, _AccelY, _AccelZ; // +- 1g
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mag_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MAGNOMETER_DATA_NODE_HPP