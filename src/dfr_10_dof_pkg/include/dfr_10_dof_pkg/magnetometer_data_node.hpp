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

        float hard_iron_bias_x = -9.231161165183828f;
        float hard_iron_bias_y = 11.958511901460092f;
        float hard_iron_bias_z = 2.463278088131798f;

        float soft_iron_bias_xx = 2.5615340163289146f;
        float soft_iron_bias_xy = -0.022110984866741405f;
        float soft_iron_bias_xz = -0.01714388180116863f;

        float soft_iron_bias_yx = -0.02211098486674125f;
        float soft_iron_bias_yy = 2.549414766531213f;
        float soft_iron_bias_yz = 0.0888132393196553f;

        float soft_iron_bias_zx = -0.01714388180116865f;
        float soft_iron_bias_zy = 0.08881323931965523f;
        float soft_iron_bias_zz = 2.498477501481151f;


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