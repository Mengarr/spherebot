#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>

class PIDController
{
public:
    /**
     * @brief Constructor to initialize PID gains and setpoint.
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param setpoint Desired target value
     */
    PIDController(double kp, double ki, double kd, double setpoint = 0.0);

    /**
     * @brief Update the PID controller with the current measurement.
     * @param measurement The current measured value
     * @return Control output
     */
    double compute(double measurement);

    /**
     * @brief Set new PID gains.
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setGains(double kp, double ki, double kd);

    /**
     * @brief Set a new setpoint.
     * @param setpoint Desired target value
     */
    void setSetpoint(double setpoint);

    /**
     * @brief Set output limits for the controller.
     * @param min_output Minimum allowable output
     * @param max_output Maximum allowable output
     */
    void setOutputLimits(double min_output, double max_output);

    /**
     * @brief Reset the PID controller history.
     */
    void reset();

private:
    double kp_;
    double ki_;
    double kd_;
    double setpoint_;

    double prev_error_;
    double integral_;

    double min_output_;
    double max_output_;

    std::chrono::steady_clock::time_point last_time_;

    /**
     * @brief Clamp a value between a minimum and maximum.
     * @param value The value to clamp
     * @param min The minimum limit
     * @param max The maximum limit
     * @return The clamped value
     */
    double clamp(double value, double min, double max);
};

#endif // PID_CONTROLLER_HPP
