#ifndef JOYSTICK_READER_NODE_HPP
#define JOYSTICK_READER_NODE_HPP

#include <fcntl.h>
#include <linux/joystick.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <unistd.h>
#include <vector>  // Include vector header

class JoystickReaderNode : public rclcpp::Node
{
public:
    JoystickReaderNode();
    ~JoystickReaderNode();

private:
    void read_joystick_data();
    int js_fd_;  // File descriptor for the joystick

    std::vector<float> axes_;      // Vector to store axis states
    std::vector<int32_t> buttons_; // Vector to store button states

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joystick_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // JOYSTICK_READER_NODE_HPP
