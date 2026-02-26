#pragma once
#ifndef KALIOSC_H
#define KALIOSC_H

#include "daisy.h"
#include "daisysp.h"
#include "KaliOptions.h"
#include "EnvelopeFollower.h" // Include the new header
#include <memory>

using namespace daisy;
using namespace daisysp;
using namespace std;

class Kali;

/**
 * @brief Extended oscillator with multiple LFO modes and modulation capabilities
 *
 * KaliOscillator extends the standard Oscillator with additional features:
 * - Multiple LFO modes (sync, random, envelope following, etc)
 * - Wavetable capabilities with interpolation
 * - Extensive modulation options
 * - Visual feedback for UI display
 * - Clock sync and division features
 */
class KaliOscillator : public Oscillator
{
public:
    KaliPreset preset;

    // Core processing parameters
    int waveform;
    float frequency;
    float last;          // Current output value
    float last_unscaled; // Raw output before scaling
    float displayfrequency;

    // LFO and modulation control
    float lfo_adjust; // Adjusts LFO behavior based on mode
    bool flip;        // Signal inversion flag
    EnvelopeFollower<2, float> *Follower;

    // Randomization/timing components
    Maytrig mt;      // Probabilistic trigger
    SampleHold th;   // Track and hold
    Adsr adsr;       // ADSR envelope
    ClockedNoise cn; // Clocked noise source

    // Clock handling
    bool flipTrack; // Track flip state
    bool is_clock;  // Whether this oscillator is used as a clock
    int clockdiv;   // Clock divider counter

    // Wavetable functionality
    uint8_t wavetable[16];

    // Visualization data (for UI)
    float last32[64]; // Buffer for visualization
    int last32pos;    // Current position in visualization buffer
    int last32posstepped;
    int last32possteppedmax;

    // Output formatting
    float attenuate;     // Output attenuator (0-100%)
    float offset;        // Offset added to output
    float attenuate_cal; // Calibrated attenuator
    float offset_cal;    // Calibrated offset

    // Mode settings
    unsigned int mode;  // Current LFO mode
    unsigned int index; // Index (0-5 for regular LFOs)
    float meta;         // Frequency multiplier
    int meta_divider;   // Frequency divider
    uint8_t bipolar;    // Output polarity setting (0=unipolar+, 1=bipolar, 2=unipolar-, 3=invert)

    // Modulation
    float follow_level;    // For envelope follower modes
    float fm;              // FM modulation input
    float am;              // AM modulation input
    float global_lfo_rate; // Global rate multiplier

    // Gate handling
    bool gate;
    bool last_gate;

    /**
     * @brief Initialize the oscillator
     * @param sample_rate Sample rate in Hz
     */
    void Init(float sample_rate);

    /**
     * @brief Process gate input for envelope triggering
     * @param gate_ Current gate state
     */
    void UpdateGate(bool gate_);

    /**
     * @brief Calculate effective rate based on meta and divider
     * @return Calculated frequency multiplier
     */
    float MetaDividerToFloat();

    /**
     * @brief Set a value in the wavetable
     * @param idx Index in wavetable (0-15)
     * @param val Value to set (0-255)
     */
    void PokeWavetable(uint8_t idx, uint8_t val);

    /**
     * @brief Read from wavetable with linear interpolation
     * @param pos Position in wavetable (continuous value)
     */
    void ReadWavetableLERP(float pos);

    /**
     * @brief Invert output signal
     */
    void Flip();

    /**
     * @brief Update settings from preset
     */
    void UpdateLocalParams();

    /**
     * @brief Set oscillator frequency
     * @param f Frequency in Hz
     */
    void SetFreq(float f);

    /**
     * @brief Set LFO mode
     * @param mode_ New mode value
     */
    void SetMode(unsigned int mode_);

    /**
     * @brief Set oscillator waveform
     * @param wf Waveform index
     */
    void SetWaveform(uint8_t wf);

    /**
     * @brief Set random waveform
     */
    void SetRandomWaveform();

    /**
     * @brief Set LFO adjustment parameter
     * @param amt Adjustment amount (0.0-1.0)
     */
    void SetLFOAdjust(float amt);

    /**
     * @brief Process envelope follower data
     */
    void UpdateFollow();

    /**
     * @brief Process one sample
     * @return Processed output value
     */
    float Process();

    /**
     * @brief Process one sample with access to other oscillators
     * @param myfriends Pointer to array of oscillators for cross-modulation
     * @return Processed output value
     */
    float Process(KaliOscillator *myfriends);

    /**
     * @brief Get scaled output value for CV output
     * @return Scaled output value
     */
    float GetScaled()
    {
        return (4095.f - ((last + offset_cal) * (attenuate_cal * 0.01f)));
    }

    /**
     * @brief Set parent for parameter access
     * @param parent Shared pointer to parent Kali object
     */
    void SetParent(std::shared_ptr<Kali> parent);

private:
    static constexpr float oneover4096 = 1.f / 4096.f;
    bool resetonnext = false;
    uint8_t eoc_count = 0;
    std::weak_ptr<Kali> kali_;
    float noise_value = 0.0f;
    float noise_phase = 0.0f;
    float noise_filter = 0.0f;
    float sample_rate_ = 0.0f;
};

#endif