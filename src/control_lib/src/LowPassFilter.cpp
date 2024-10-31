#include "../include/control_lib/LowPassFilter.hpp"

// Constructor implementation
LowPassFilter::LowPassFilter(size_t window_size) : window_size_(window_size) {}

// Filter method implementation
float LowPassFilter::filter(float input) {
    // Add the new input to the window
    window_.push_back(input);

    // If the window exceeds the specified size, remove the oldest value
    if (window_.size() > window_size_) {
        window_.pop_front();
    }

    // Calculate the average of the values in the window
    float sum = std::accumulate(window_.begin(), window_.end(), 0.0f);
    return sum / window_.size();
}
