#ifndef IMU_SENSOR_DATA_H
#define IMU_SENSOR_DATA_H

#define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

#include "FIMU_ITG3200.h"
#include "FIMU_ADXL345.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuPublisher : public rclcpp::Node
{
    public:
        ImuPublisher();
    private:
        void publish_data();

        void getQ(float *q);
        void getQ();
        void getValues(float * values);
        void getEuler(float * angles);
        void getAngles(float * angles);
        void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
        void getYawPitchRoll(float * ypr);

        ITG3200 gyro;
        ADXL345 accel;
        
        float _GyroX, _GyroY, _GyroZ; // In rad/s
        int16_t _AccelX, _AccelY, _AccelZ; // raw accel data

        float iq0, iq1, iq2, iq3;
        float exInt, eyInt, ezInt;  // scaled integral error
        volatile float twoKp;      // 2 * proportional gain (Kp)
        volatile float twoKi;      // 2 * integral gain (Ki)
        volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
        volatile float integralFBx,  integralFBy, integralFBz;
        std::chrono::_V2::system_clock::time_point lastUpdate, now; // sample period expressed in milliseconds
        double sampleFreq; // half the sample period expressed in seconds
        int startLoopTime;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // IMU_SENSOR_DATA_H