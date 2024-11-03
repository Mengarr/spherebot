#ifndef MAGNOMETER_CALIB_NODE_HPP
#define MAGNOMETER_CALIB_NODE_HPP

#include "VCM5883L.h"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <chrono>
#include <thread>

class MagnetometerCalibration : public rclcpp::Node
{
    public:
        MagnetometerCalibration();
        ~MagnetometerCalibration();
    private:
        void timerCallback();

        VCM5883L magnetometer;
        std::ofstream data_file_;  // File stream for CSV output

        const float conv_factor = 0.0244140625;

        rclcpp::TimerBase::SharedPtr timer_;
};

#endif // MAGNOMETER_CALIB_NODE_HPP