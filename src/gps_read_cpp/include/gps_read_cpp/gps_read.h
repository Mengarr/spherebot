#ifndef GPS_READ_H
#define GPS_READ_H

#include "TinyGPS++.h"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>      // For strerror()
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point.hpp" // for x,y coordinates
#include "control_lib/geodeticConverter.hpp"

const char PORT[13]  = "/dev/ttyAMA0";

class GPSread : public rclcpp::Node
{
private:
    // ROS Stuff
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr coords_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // GPS conversion
    GeodeticConverter _geodeticConverter;
    bool init_ = false;
    double _lat = 0.0; double _long = 0.0; double _alt = 0.0;

    // UART Stuff
    int fd;
    void configurePort();
    
    // TinyGPS
    TinyGPSPlus gps;

    // Publishing function
    void publish_data();
public:
    GPSread();
    ~GPSread();
};


#endif // GPS_READ_H