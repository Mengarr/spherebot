#ifndef ENVIRONMENTAL_DATA_NODE_H
#define ENVIRONMENTAL_DATA_NODE_H

#include "BMP280.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class EnvironmentalPublisher : public rclcpp::Node
{
    public:
        EnvironmentalPublisher();
    private:
        void publish_data();

        DFRobot_BMP280 bmp280;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pressure_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;
        
};

#endif // ENVIRONMENTAL_DATA_NODE_H