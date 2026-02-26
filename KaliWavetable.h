#pragma once
#ifndef KALIWAVETABLE_H
#define KALIWAVETABLE_H

#include "daisy.h"
#include "daisysp.h"

class KaliWavetable
{
public:
    float wavetable[16];

    KaliWavetable()
    {
        for (int i = 0; i < 16; i++)
            wavetable[i] = i * 16;
    }

    float ReadWavetableLERP(float pos)
    {
        // Ensure pos is within 0-16 range and wrap around if needed
        pos = fmod(pos, 16.0f);
        if (pos < 0.0f)
            pos += 16.0f;

        // Get integer indices for interpolation
        int idx_a = static_cast<int>(pos) % 16;
        int idx_b = (idx_a + 1) % 16;

        // Calculate fractional part for interpolation
        float frac = pos - static_cast<float>(idx_a);

        // Get the two values to interpolate between
        float value_a = static_cast<float>(wavetable[idx_a]) / 255.0f;
        float value_b = static_cast<float>(wavetable[idx_b]) / 255.0f;

        // Perform linear interpolation
        float interpolated = value_a * (1.0f - frac) + value_b * frac;
        return interpolated;
    }

    uint8_t Peek(uint8_t idx)
    {
        return wavetable[idx];
    }

    void Poke(uint8_t idx, uint8_t val)
    {
        if (idx < 16)
        {
            wavetable[idx] = val;
        }
    }
};
#endif