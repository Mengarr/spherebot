#include "../include/control_lib/stanleyControl.hpp"

// Constructor with waypoints
StanleyController::StanleyController(const std::vector<double>& waypoints_x,
                                     const std::vector<double>& waypoints_y,
                                     double k, double k_s, double L, double max_steer, double tolerance)
    : k_(k), k_s_(k_s), L_(L), max_steer_(max_steer), tolerance_(tolerance) {
    if (waypoints_x.size() != waypoints_y.size()) {
        throw std::invalid_argument("Waypoints x and y vectors must have the same size.");
    }

    for (size_t i = 0; i < waypoints_x.size(); ++i) {
        waypoints_.emplace_back(Point{waypoints_x[i], waypoints_y[i]});
    }
}

// Default constructor
StanleyController::StanleyController(double k, double k_s, double L, double max_steer, double tolerance)
    : k_(k), k_s_(k_s), L_(L), max_steer_(max_steer), tolerance_(tolerance) {}

// Update waypoints using separate x and y vectors
void StanleyController::updateWaypoints(const std::vector<double>& waypoints_x,
                                        const std::vector<double>& waypoints_y) {
    if (waypoints_x.size() != waypoints_y.size()) {
        throw std::invalid_argument("Waypoints x and y vectors must have the same size.");
    }

    waypoints_.clear();
    for (size_t i = 0; i < waypoints_x.size(); ++i) {
        waypoints_.emplace_back(Point{waypoints_x[i], waypoints_y[i]});
    }
}

// Update waypoints using a vector of Point structs
void StanleyController::updateWaypoints(const std::vector<Point>& waypoints) {
    waypoints_ = waypoints;
}

// Compute steering angle based on the current state and waypoints
bool StanleyController::computeSteering(double x, double y, double yaw, double v,
                                        double& steering_angle, double& heading_error) const {
    if (waypoints_.empty()) {
        return false; // No waypoints available
    }

    // Find the nearest waypoint index
    size_t nearest_index = 0;
    double min_dist = std::hypot(waypoints_[0].x - x, waypoints_[0].y - y);

    for (size_t i = 1; i < waypoints_.size(); ++i) {
        double dist = std::hypot(waypoints_[i].x - x, waypoints_[i].y - y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_index = i;
        }
    }

    // Get nearest waypoint and next waypoint
    const Point& nearest_wp = waypoints_[nearest_index];
    const Point& next_wp = (nearest_index < waypoints_.size() - 1) ? waypoints_[nearest_index + 1] : nearest_wp;

    // Line equation parameters: ax + by + c = 0
    double a = next_wp.y - nearest_wp.y;
    double b = nearest_wp.x - next_wp.x;
    double c = next_wp.x * nearest_wp.y - next_wp.y * nearest_wp.x;

    // Cross-track error
    double e = (a * x + b * y + c) / std::sqrt(a * a + b * b);

    // Path yaw
    double path_yaw = std::atan2(next_wp.y - nearest_wp.y, next_wp.x - nearest_wp.x);

    // Heading error and normalization to [-pi, pi]
    heading_error = normalizeAngle(path_yaw - yaw);

    // Cross-track steering
    double cross_track_steer = std::atan2(k_ * e, v + k_s_);

    // Total steering angle
    steering_angle = heading_error + cross_track_steer;

    return true;
}

// Normalize an angle to the range [-pi, pi]
double StanleyController::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Clamp a value between a minimum and maximum
double StanleyController::clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Function to check if the final waypoint is reached
bool StanleyController::reachedFinal(double x, double y) const {
    if (waypoints_.empty()) {
        return false;
    }

    // Get the final waypoint
    const Point& final_waypoint = waypoints_.back();

    // Calculate the Euclidean distance to the final waypoint
    double distance = std::sqrt(std::pow(final_waypoint.x - x, 2) +
                                std::pow(final_waypoint.y - y, 2));

    // Return true if within the tolerance
    return distance < tolerance_;
}