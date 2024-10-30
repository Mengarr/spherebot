#include "../include/control_lib/PID_controller.hpp"
#include <algorithm> // For std::min and std::max

PIDController::PIDController(double kp, double ki, double kd, double setpoint)
    : kp_(kp), ki_(ki), kd_(kd), setpoint_(setpoint),
      prev_error_(0.0), integral_(0.0),
      min_output_(std::numeric_limits<double>::lowest()),
      max_output_(std::numeric_limits<double>::max()),
      last_time_(std::chrono::steady_clock::now())
{
}

double PIDController::compute(double measurement)
{
    // Calculate error
    double error = setpoint_ - measurement;

    // Get current time and calculate time difference
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_diff = current_time - last_time_;
    double dt = time_diff.count();
    last_time_ = current_time;

    if (dt <= 0.0)
    {
        dt = 1e-3; // Prevent division by zero
    }

    // Proportional term
    double P_out = kp_ * error;

    // Integral term with anti-windup via clamping
    integral_ += error * dt;
    double I_out = ki_ * integral_;

    // Derivative term
    double derivative = (error - prev_error_) / dt;
    double D_out = kd_ * derivative;

    // Save error for next derivative calculation
    prev_error_ = error;

    // Total output before clamping
    double output = P_out + I_out + D_out;

    // Clamp output to min and max limits
    output = clamp(output, min_output_, max_output_);

    // Optional: If output is clamped, you might want to prevent integral windup
    // This can be implemented by checking if output was clamped and adjusting integral_
    // accordingly. For simplicity, it's not included here.

    return output;
}

void PIDController::setGains(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setSetpoint(double setpoint)
{
    setpoint_ = setpoint;
}

void PIDController::setOutputLimits(double min_output, double max_output)
{
    if (min_output > max_output)
    {
        // Swap if min is greater than max
        std::swap(min_output, max_output);
    }
    min_output_ = min_output;
    max_output_ = max_output;

    // // Optional: Clamp integral term to prevent windup within new output limits
    // integral_ = clamp(integral_, min_output_ / ki_, max_output_ / ki_);
}

void PIDController::reset()
{
    prev_error_ = 0.0;
    integral_ = 0.0;
    last_time_ = std::chrono::steady_clock::now();
}

double PIDController::clamp(double value, double min, double max)
{
    return std::max(min, std::min(value, max));
}
