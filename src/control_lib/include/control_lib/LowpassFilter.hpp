// LowPassFilter.hpp
#ifndef LOWPASSFILTER_HPP
#define LOWPASSFILTER_HPP

#include <vector>

class LowPassFilter {
public:
    // Constructor
    LowPassFilter(float alpha, size_t size);
    
    // Method to filter a fixed-length vector of floats
    std::vector<float> filter(const std::vector<float>& input);

private:
    float alpha_;  // Smoothing factor
    std::vector<float> filtered_values;  // Stores the last filtered values
};

#endif // LOWPASSFILTER_HPP
