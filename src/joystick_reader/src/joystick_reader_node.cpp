#include "../include/joystick_reader/joystick_reader_node.hpp"  // Use quotes for local includes

JoystickReaderNode::JoystickReaderNode() 
: Node("joystick_reader_node"),
  axes_(8, 0.0f),      // Initialize axes_ with 8 elements
  buttons_(14, 0)      // Initialize buttons_ with 12 elements
{
    // Create the publisher for sensor_msgs::msg::Joy
    joystick_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

    // Open the joystick device
    js_fd_ = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (js_fd_ == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/input/js0");
        rclcpp::shutdown();
        return; // Exit constructor if joystick fails to open
    }

    // Start the timer to read joystick data
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JoystickReaderNode::read_joystick_data, this)
    );
}

JoystickReaderNode::~JoystickReaderNode()
{
    if (js_fd_ != -1)
    {
        close(js_fd_);
    }
}

void JoystickReaderNode::read_joystick_data()
{
    struct js_event js;

    while (read(js_fd_, &js, sizeof(struct js_event)) > 0)
    {
        js.type &= ~JS_EVENT_INIT;  // Mask to ignore JS_EVENT_INIT flag

        // Handle axis events
        if (js.type == JS_EVENT_AXIS)
        {
            if (js.number < axes_.size())
            {
                // Optionally normalize the axis value
                // axes_[js.number] = js.value / 32767.0f;  // Normalize to [-1, 1]
                axes_[js.number] = static_cast<float>(js.value) / 32767.0f;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Axis number %d out of range", js.number);
            }
        }
        // Handle button events
        else if (js.type == JS_EVENT_BUTTON)
        {
            if (js.number < buttons_.size())
            {
                buttons_[js.number] = js.value;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Button number %d out of range", js.number);
            }
        }
    }

    // Publish the joystick data using sensor_msgs::msg::Joy
    auto msg = sensor_msgs::msg::Joy();
    msg.header.stamp = this->now();
    msg.axes = axes_;       // Correctly assign axes_
    msg.buttons = buttons_; // Correctly assign buttons_
    joystick_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickReaderNode>());
    rclcpp::shutdown();
    return 0;
}
