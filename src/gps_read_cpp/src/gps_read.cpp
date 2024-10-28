#include "../include/gps_read_cpp/gps_read.h"

void GPSread::configurePort() {
        termios options;
        tcgetattr(fd, &options);
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        options.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(fd, TCSANOW, &options);
}

GPSread::~GPSread()
{
    close(fd);
}

GPSread::GPSread()
    : Node("gps_publisher")
{   
    RCLCPP_INFO(this->get_logger(), "GPS reading node has been started.");

    gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
    timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { publish_GPS_data(); });
    
    // Start listening to UART port
    fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Unable to open serial port" << ": " << strerror(errno) << std::endl;
        exit(1);
    }
    configurePort();
} 

void GPSread::publish_GPS_data() 
{
  // Read gps data from UART:
  char c;
  while (read(fd, &c, 1) > 0) {
      gps.encode(c);
  }

  // msg data
  auto msg = sensor_msgs::msg::NavSatFix();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "gps_frame";

  if (gps.location.isValid()) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  //
  if (gps.charsProcessed() < 10)
    RCLCPP_INFO(this->get_logger(), "WARNING: No GPS data.  Check wiring.");

  msg.latitude = gps.location.lat();   // Example latitude
  msg.longitude = gps.location.lng(); // Example longitude
  msg.altitude = gps.altitude.meters();      // Example altitude

  // Optional: Set covariance
  msg.position_covariance[0] = 0.0;
  msg.position_covariance[4] = 0.0;
  msg.position_covariance[8] = 0.0;
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  // Publish the data
  gps_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSread>());
    rclcpp::shutdown();
    return 0;
}
