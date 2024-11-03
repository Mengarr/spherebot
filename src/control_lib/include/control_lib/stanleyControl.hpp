#ifndef STANLEY_CONTROLLER_HPP
#define STANLEY_CONTROLLER_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

class StanleyController {
public:
    // Struct to represent a 2D point
    struct Point {
        double x;
        double y;
    };

    // Constructor that initializes the controller with waypoints and optional parameters
    StanleyController(const std::vector<double>& waypoints_x,
                      const std::vector<double>& waypoints_y,
                      double k = 1.0,
                      double k_s = 0.01,
                      double L = 1.0,
                      double max_steer = M_PI / 4,
                      double tolerance = 0.5);

    // Default constructor
    StanleyController(double k = 1.0,
                      double k_s = 0.01,
                      double L = 1.0,
                      double tolerance = 0.5);

    // Update waypoints using separate x and y vectors
    void updateWaypoints(const std::vector<double>& waypoints_x,
                         const std::vector<double>& waypoints_y);

    // Set params
    void setParams(double k, double k_s, double L, double tolerance);

    // Update waypoints using a vector of Point structs
    void updateWaypoints(const std::vector<Point>& waypoints);

    // Compute steering angle based on current vehicle state and waypoints
    bool computeSteering(double x, double y, double yaw, double v,
                         double& steering_angle, double& heading_error) const;

    // Check if the final waypoint is reached
    bool reachedFinal(double x, double y) const;

private:
    std::vector<Point> waypoints_;
    double k_;         // Control gain
    double k_s_;       // Small gain to prevent division by zero
    double L_;         // Vehicle wheelbase
    double max_steer_=  M_PI / 4; // Maximum steering angle in radians
    double tolerance_;    // Tolerance distance to consider the final waypoint reached

    // Helper functions
    static double normalizeAngle(double angle);
    static double clamp(double value, double min_val, double max_val);
};

#endif // STANLEY_CONTROLLER_HPP
