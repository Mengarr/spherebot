#ifndef MAGNOMETER_DATA_NODE_HPP
#define MAGNOMETER_DATA_NODE_HPP

#include "VCM5883L.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "control_lib/LowPassFilter.hpp"
#include "sensor_msgs/msg/imu.hpp"

class MagnetometerPublisher : public rclcpp::Node
{
    public:
        MagnetometerPublisher();
    private:
        void publish_data();

        VCM5883L magnetometer;

        float declinationAngle_ = 0.0f; // Initalise to zero

        float compensatedHeading_ = 0.0f;

    float hard_iron_bias_x = -9.293616653744394f;
    float hard_iron_bias_y = 10.357573182856601f;
    float hard_iron_bias_z = 1.5388180681094008f;

    float soft_iron_bias_xx = 2.582395659431281f;
    float soft_iron_bias_xy = 0.003499380180901916f;
    float soft_iron_bias_xz = 0.01771280289939006f;

    float soft_iron_bias_yx = 0.0034993801809018463f;
    float soft_iron_bias_yy = 2.546391727520486f;
    float soft_iron_bias_yz = 0.13898612615765935f;

    float soft_iron_bias_zx = 0.017712802899389923f;
    float soft_iron_bias_zy = 0.13898612615765907f;
    float soft_iron_bias_zz = 2.5790663103590514f;

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