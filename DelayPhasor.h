#pragma once
#ifndef KALIDELAYPHASOR_H
#define KALIDELAYPHASOR_H

#include "consts.h"
#include <cstddef>
#include <algorithm>

class DelayPhasor
{
public:
    DelayPhasor() : phase_(0.0f),
                    modPhase_(0.0f),
                    sampleRate_(48000.0f),
                    mainFreq_(1.0f),
                    delaySamples_(0.0f),
                    modFreqRatio_(1.0f),
                    modDepth_(1.0f),
                    write_ptr_(0),
                    max_size_(MAX_DELAY) {}

    inline void Init(float sampleRate)
    {
        sampleRate_ = sampleRate;
        Reset();
    }

    inline void Reset()
    {
        phase_ = 0.0f;
        modPhase_ = 0.0f;
    }

    // Set base delay time in samples
    inline void SetDelaySamples(float samples)
    {
        delaySamples_ = std::max(1.0f, samples);
        mainFreq_ = sampleRate_ / delaySamples_;
    }

    // Set modulation frequency relative to main frequency
    inline void SetModFreqRatio(float ratio)
    {
        modFreqRatio_ = ratio;
    }

    // Set modulation depth (0-20.f)
    inline void SetModDepth(float depth)
    {
        modDepth_ = std::max(0.0f, std::min(20.0f, depth));
    }

    // Update write pointer from delay line
    inline void SetWritePtr(size_t ptr)
    {
        write_ptr_ = ptr;
    }

    // Set maximum buffer size
    inline void SetMaxSize(size_t size)
    {
        max_size_ = size;
    }

    // Process both phasors and return main phase
    float Process();

    // Getters for internal state
    inline float GetMainPhase() const { return phase_; }
    inline float GetModPhase() const { return modPhase_; }
    inline float GetModDepth() const { return modDepth_; }
    inline float GetDelaySamples() const { return delaySamples_; }
    float GetModulatedSamples(bool triangle = false) const;

    static inline float MakeTriangle(float pos)
    {
        return (1.f - pos) * 0.5f + 0.25f;
    }

    // Get the current base delay position
    inline float GetDelayPosition() const
    {
        return phase_ * delaySamples_;
    }

    // Get the modulated read position relative to write pointer
    float GetModulatedPosition(bool triangle = false) const;

private:
    float phase_;        // Main phase (0-1)
    float modPhase_;     // Modulation phase (0-1)
    float sampleRate_;   // System sample rate
    float mainFreq_;     // Base frequency from delay time
    float delaySamples_; // Base delay time in samples
    float modFreqRatio_; // Modulation frequency multiplier
    float modDepth_;     // Modulation depth (0-1)
    size_t write_ptr_;   // Current write position
    size_t max_size_;    // Maximum buffer size

    // Fast approximations for common functions
    float FastSin(float x) const; // Sine approximation
    float FastMod(float x) const; // Fast modulo for 0-1 range
};

#endif // KALIDELAYPHASOR_H