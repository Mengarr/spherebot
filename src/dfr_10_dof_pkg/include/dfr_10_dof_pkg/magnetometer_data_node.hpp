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

        // float hard_iron_bias_x = -6.263105902035311;
        // float hard_iron_bias_y = 4.292579497477027;
        // float hard_iron_bias_z = 3.3543842551502108;

        // float soft_iron_bias_xx = 2.3912019992637563;
        // float soft_iron_bias_xy = -0.017923377264954937;
        // float soft_iron_bias_xz = -0.06794333395212657;

        // float soft_iron_bias_yx = -0.01792337726495499;
        // float soft_iron_bias_yy = 2.3365235008621434;
        // float soft_iron_bias_yz = 0.1252346512198512;

        // float soft_iron_bias_zx = -0.06794333395212654;
        // float soft_iron_bias_zy = 0.12523465121985125;
        // float soft_iron_bias_zz = 2.2155018362436585;

        float hard_iron_bias_x = -4.167842441428256;
        float hard_iron_bias_y = -9.569898747781659;
        float hard_iron_bias_z = 25.106024745932398;

        float soft_iron_bias_xx = 2.47922351462819;
        float soft_iron_bias_xy = -0.032118054883568545;
        float soft_iron_bias_xz = 0.004917413394485149;

        float soft_iron_bias_yx = -0.0321180548835687;
        float soft_iron_bias_yy = 2.5356096459546005;
        float soft_iron_bias_yz = 0.1641655446789155;

        float soft_iron_bias_zx = 0.004917413394485104;
        float soft_iron_bias_zy = 0.16416554467891525;
        float soft_iron_bias_zz = 2.3692364611493066;

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