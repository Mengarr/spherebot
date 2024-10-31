#ifndef MAGNOMETER_DATA_NODE_HPP
#define MAGNOMETER_DATA_NODE_HPP

#include "VCM5883L.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "control_lib/LowpassFilter.hpp"
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

        // Hard iron offsets
        float hard_iron_bias_x =  5.546126519735883;
        float hard_iron_bias_y =  -2.1566405156685073;
        float hard_iron_bias_z =  -8.016331894986969;
        
        // Soft iron offsets
        float soft_iron_bias_xx =  3.260518828218278;
        float soft_iron_bias_xy =  -0.03963877040661722;
        float soft_iron_bias_xz =  -0.16006485672169196;


        float soft_iron_bias_yx =  -0.03963877040661716;
        float soft_iron_bias_yy =  2.6699307594697257;
        float soft_iron_bias_yz =  0.15367305936821996;


        float soft_iron_bias_zx =  -0.16006485672169185;
        float soft_iron_bias_zy =  0.15367305936822004;
        float soft_iron_bias_zz =  2.172672993775984;
        
        // conversion factor to convert +-8g to uT
        float conv_factor = 0.0244140625;

        // Functions
        float tilt_compensated_heading(float Mx, float My, float Mz, float ax, float ay, float az);
        std::vector<float> getCalibratedValues(float Mx, float My, float Mz);

        // Low pass filter
        LowPassFilter lpf_heading_; // (0.3f, static_cast<size_t>(1))
        float _alpha = 1.0f;

        // Imu stuff
        float _AccelX, _AccelY, _AccelZ; // +- 1g
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mag_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MAGNOMETER_DATA_NODE_HPP