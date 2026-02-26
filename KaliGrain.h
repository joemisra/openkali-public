#pragma once
#ifndef DSY_DELAY_GRAIN_H
#define DSY_DELAY_GRAIN_H
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
namespace daisysp
{
// Generic Grain class that can operate on any buffer
class KaliGrain
{
public:
    // Processes a single sample with windowing and returns it
    float Process(const float* buffer, size_t buffer_length, size_t pos, float playback_rate, size_t grain_size)
    {
        if (current_index_ >= grain_size) {
            current_index_ = 0;
            return 0.0f;  // Stop processing if grain is complete
        }

        // Apply Hann window for smooth grain edges
        float window = 0.5f * (1.0f - cosf(2.0f * 3.14159265f * current_index_ / (grain_size - 1)));

        // Read from buffer with wrapping
        size_t read_idx = pos - (current_index_ * playback_rate);
        float sample = buffer[read_idx]; // * window;

        // Advance index and read position
        current_index_++;

        return sample;
    }

private:
    size_t current_index_ = 0;   // Current index within the grain
};

} // namespace daisysp
#endif