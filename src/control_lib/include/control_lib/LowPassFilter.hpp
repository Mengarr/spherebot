#ifndef LOWPASSFILTER_HPP
#define LOWPASSFILTER_HPP

#include <vector>
#include <deque>
#include <numeric>

class LowPassFilter {
public:
    // Constructor that sets the window size
    LowPassFilter(size_t window_size);

    // Method to filter input values and return the filtered result
    float filter(float input);

private:
    size_t window_size_;               // Size of the moving window
    std::deque<float> window_;         // Deque to store recent values
};

#endif // LOWPASSFILTER_HPP
