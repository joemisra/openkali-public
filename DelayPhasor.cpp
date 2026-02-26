#include "DelayPhasor.h"
#include <cmath>

float Smooth(float current, float target, float smoothingFactor)
{
    return current + (target - current) * smoothingFactor;
}

// Fast sine approximation using Taylor series
float DelayPhasor::FastSin(float x) const
{
    // Wrap input to -PI to PI
    x = x - ((int)(x * 0.159155f) * 6.28318f);

    // 4th order Taylor approximation
    float x2 = x * x;
    return x * (1.0f - x2 * (1.0f / 6.0f - x2 * (1.0f / 120.0f)));
}

// Fast modulo for 0-1 range
float DelayPhasor::FastMod(float x) const
{
    return x - static_cast<int>(x);
}

float DelayPhasor::Process()
{
    // Update main phase
    phase_ += mainFreq_ / sampleRate_;
    phase_ = FastMod(phase_);

    // Update modulation phase with frequency ratio
    modPhase_ += ((mainFreq_ * modFreqRatio_) / sampleRate_);
    modPhase_ = FastMod(modPhase_);

    return phase_;
}

float DelayPhasor::GetModulatedPosition(bool triangle) const
{
    // Calculate modulation amount using triangle wave
    float mod = 2.0f * fabs(FastMod(modPhase_ + 0.25f) - 0.5f);

    // Scale modulation by depth
    mod = mod * 1.0f;

    // Add modulation to base phase and wrap
    float modulated_phase = FastMod(phase_ + mod);

    modulated_phase = triangle ? MakeTriangle(modulated_phase) : modulated_phase; // triangle

    return modulated_phase;
}

float DelayPhasor::GetModulatedSamples(bool triangle) const
{
    float modulated_phase = GetModulatedPosition(triangle);

    // Convert phase to delay position
    float delay_pos = modulated_phase * delaySamples_;

    // Ensure we stay within buffer bounds
    return delay_pos;
}