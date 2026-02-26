#pragma once
#ifndef KALI_FIR_H
#define KALI_FIR_H

#include <math.h>
#include <algorithm>
class OptimizedFIR
{
private:
    static constexpr size_t NUM_TAPS = 17;
    float delayLine[NUM_TAPS];
    float coefficients[NUM_TAPS];
    float window[NUM_TAPS];
    float last_cutoff;

public:
    void Init()
    {
        last_cutoff = 0.0f;

        // Clear buffers and compute window
        for (size_t i = 0; i < NUM_TAPS; i++)
        {
            delayLine[i] = 0.0f;
            coefficients[i] = 0.0f;

            // Compute Blackman window
            const float n = i;
            const float N = NUM_TAPS - 1;
            window[i] = 0.42f - 0.5f * cosf(2.0f * M_PI * n / N) +
                        0.08f * cosf(4.0f * M_PI * n / N);
        }

        // Initialize with wide open cutoff
        UpdateFilter(20000.0f, 48000.0f);
    }

    void UpdateFilter(float cutoff_freq, float sample_rate)
    {
        // Map cutoff exponentially across full range
        const float min_freq = 10.0f; // Lower minimum for more closing
        const float max_freq = sample_rate * 0.48f;

        // Only update if change is significant
        float threshold = cutoff_freq * 0.005f; // Reduced threshold for finer control
        if (fabsf(cutoff_freq - last_cutoff) < threshold)
        {
            return;
        }
        last_cutoff = cutoff_freq;

        // Clamp frequency and normalize
        cutoff_freq = DSY_CLAMP(cutoff_freq, min_freq, max_freq);
        float fc = cutoff_freq / sample_rate;

        const int M = NUM_TAPS - 1;
        const float center = M / 2.0f;
        float sum = 0.0f;

        // Compute filter coefficients
        for (int i = 0; i < NUM_TAPS; i++)
        {
            float n = i - center;

            if (fabsf(n) < 1e-6)
            {
                coefficients[i] = 2.0f * fc;
            }
            else
            {
                coefficients[i] = sinf(2.0f * M_PI * fc * n) / (M_PI * n);
            }

            // Apply window
            coefficients[i] *= window[i];
            sum += coefficients[i];
        }

        // Normalize for unity gain at DC
        if (sum != 0.0f)
        {
            const float scale = 1.0f / sum;
            for (int i = 0; i < NUM_TAPS; i++)
            {
                coefficients[i] *= scale;
            }
        }

        // Extra attenuation for very low frequencies
        if (cutoff_freq < 30.0f)
        {
            float atten = (cutoff_freq - min_freq) / (30.0f - min_freq);
            atten = atten * atten; // Square for sharper rolloff
            for (int i = 0; i < NUM_TAPS; i++)
            {
                coefficients[i] *= atten;
            }
        }
    }

    float ProcessSample(float input)
    {
        if (!isfinite(input))
        {
            return 0.0f;
        }

        // Shift delay line
        for (int i = NUM_TAPS - 1; i > 0; i--)
        {
            delayLine[i] = delayLine[i - 1];
        }
        delayLine[0] = input;

        // Apply filter
        float output = 0.0f;
        for (int i = 0; i < NUM_TAPS; i++)
        {
            output += delayLine[i] * coefficients[i];
        }

        if (!isfinite(output))
        {
            for (int i = 0; i < NUM_TAPS; i++)
            {
                delayLine[i] = 0.0f;
            }
            return 0.0f;
        }

        return output;
    }

    void Process(const float *input, float *output, size_t size)
    {
        for (size_t i = 0; i < size; i++)
        {
            output[i] = ProcessSample(input[i]);
        }
    }
};

#endif