#include "../include/control_lib/LowpassFilter.hpp"

// Constructor implementation
LowPassFilter::LowPassFilter(float alpha, size_t size) 
    : alpha_(alpha), filtered_values(size, 0.0f) {}

// Filter method implementation
std::vector<float> LowPassFilter::filter(const std::vector<float>& input) {
    std::vector<float> output(input.size());

    // Apply the low-pass filter to each element
    for (size_t i = 0; i < input.size(); ++i) {
        filtered_values[i] = alpha_ * input[i] + (1 - alpha_) * filtered_values[i];
        output[i] = filtered_values[i];
    }

    return output;
}
