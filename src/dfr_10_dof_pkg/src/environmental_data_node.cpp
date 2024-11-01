#include "../include/dfr_10_dof_pkg/environmental_data_node.hpp"

EnvironmentalPublisher::EnvironmentalPublisher()
    : Node("environment_data"),
    bmp280(0x77)
{   
    RCLCPP_INFO(this->get_logger(), "Environmental Data Node has been Started");
    // Temp
    temp_publisher_ = this->create_publisher<std_msgs::msg::Float32>("environment_data/temp_degrees", 10);

    // Pressure
    pressure_publisher_ = this->create_publisher<std_msgs::msg::Float32>("environment_data/pressure_pa", 10);

    // Timer for data publishing
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), // 10 Hz
    std::bind(&EnvironmentalPublisher::publish_data, this));

    // Barometer and temperature data
    RCLCPP_INFO(this->get_logger(), "Initalising Barometer");
    bmp280.begin();

    RCLCPP_INFO(this->get_logger(), "Initalisation Complete");
}

void EnvironmentalPublisher::publish_data()
{
    auto pressure_msg = std_msgs::msg::Float32();
    auto temp_msg = std_msgs::msg::Float32();

    uint32_t pressure = bmp280.getPressure();
    float temperature = bmp280.getTemperature();

    pressure_msg.data = (float)pressure;

    temp_msg.data = temperature;

    pressure_publisher_->publish(pressure_msg);
    temp_publisher_->publish(temp_msg);

    // RCLCPP_INFO(this->get_logger(), "Temperature : %.4f *C", temperature);
    // RCLCPP_INFO(this->get_logger(), "Pressure : %d Pa", pressure);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EnvironmentalPublisher>());
    rclcpp::shutdown();
    return 0;
}