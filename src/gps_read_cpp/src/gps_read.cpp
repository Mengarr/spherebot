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
    : Node("gps_publisher"),
    _geodeticConverter()
{   
    RCLCPP_INFO(this->get_logger(), "GPS reading node has been started.");

    gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 5);
    coords_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("gps/point_data", 5);

    // Initialize a timer that runs at 1 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),  // 1000 ms interval corresponds to 1 Hz
        std::bind(&GPSread::publish_data, this)
    );
    
    // Start listening to UART port
    fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Unable to open serial port" << ": " << strerror(errno) << std::endl;
        exit(1);
    }
    configurePort();
} 

void GPSread::publish_data() 
{
  // Read gps data from UART:
  char c;
  while (read(fd, &c, 1) > 0) {
      gps.encode(c);
  }
  // coords data
  auto msg = geometry_msgs::msg::Point();

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

  if (gps.charsProcessed() < 10)
    RCLCPP_INFO(this->get_logger(), "WARNING: No GPS data.  Check wiring.");

  _lat = gps.location.lat();      // latitude
  _long = gps.location.lng();     //  longitude
  _alt = gps.altitude.meters();   //  altitude

  msg.latitude = _lat;
  msg.longitude = _long;
  msg.altitude = _alt;

  // Set covariance
  msg.position_covariance[0] = 0.0;
  msg.position_covariance[4] = 0.0;
  msg.position_covariance[8] = 0.0;
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  // Conversion to enu coordinates
  if (!init_ &&  msg.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    _geodeticConverter.setReferenceOrigin(_lat, _long, _alt)
    init_ = true;
    coords_msg.x = 0.0;
    coords_msg.y = 0.0;
    coords_msg.z = 0.0;
  } 

  if (init_) {
    ENU coords = _geodeticConverter.geodeticToENU(_lat, _long, _alt);
    coords_msg.x = coords.east;
    coords_msg.y = coords.north;
    coords_msg.z = coords.up;
  }

  // Publish the data
  gps_publisher_->publish(msg);
  coords_publisher_->publish(coords_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSread>());
    rclcpp::shutdown();
    return 0;
}
