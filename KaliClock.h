// Created with assistance from Claude 3.7 Sonnet
// https://claude.anthropic.com/chat/

#pragma once
#ifndef KALICLOCK_H
#define KALICLOCK_H

#include "daisy.h"
#include "dpt/daisy_dpt.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

class KaliClock
{
public:
    enum KaliClockMode
    {
        Internal,  // 0  - Internal clock generator
        ClockIn,   // 1  - External CV clock input
        MidiClock, // 2  - External MIDI clock
        None,      // if mod
        LAST_CLOCK_MODE
    };
    static const int MIDI_PPQN = 24; // Standard MIDI PPQN

    size_t size = 96; // buffer size

    // Trigger tracking
    int TriggerAccumulator;
    int internal_ppqn = 4;  // Internal clock PPQN (default 4)
    int external_ppqn = 24; // External CV clock PPQN (default 24)
    int midi_ppqn;          // MIDI clock PPQN (fixed at 24)

    // Sample tracking
    int SampleAccumulator; // For output clock generation
    int TimingAccumulator; // For input timing detection
    int spqn;              // Samples Per Quarter Note

    KaliClockMode Mode;

    // Moving average for tempo tracking - reduced for responsiveness
    static const uint8_t foravgsize = 3; // number for average
    float foravg[3];
    uint8_t foravgindex = 0;
    uint8_t full = 0;

    float one_ms = 48.f; // Default to 48 samples per ms - will be set correctly in Init

    // Gate handling
    bool gate = false;
    int gate_timer = 0;
    bool previous_trigger = false; // For edge detection

    // Clock maintaining variables
    float last_valid_spqn;     // Stores the last valid timing between pulses
    int clock_timeout;         // How many samples before we consider the clock lost
    int clock_timeout_counter; // Counter for tracking time since last pulse

    // Initialization tracking
    bool initialized = false;

    /**
     * Initialize the clock with basic parameters
     * @param samplerate_ System sample rate
     * @param size_ Buffer size
     * @param internal_ppqn_ Internal clock pulses per quarter note (default 4)
     * @param spqn_ Samples per quarter note (default 144)
     * @param Mode_ Clock mode (default Internal)
     */
    void Init(float samplerate_, size_t size_, int internal_ppqn_ = 4, int spqn_ = 144, KaliClockMode Mode_ = KaliClockMode::Internal);

    /**
     * Process clock tick
     * @param TriggerReceived Whether an external trigger was received
     * @return True if clock is running and in sync
     */
    bool Tick(bool TriggerReceived);

    /**
     * Process trigger from external source
     */
    void ProcessTrigger();

    /**
     * Process internal clock timing
     * @param maxsamples Maximum samples per tick
     * @return True if clock is running
     */
    bool ProcessInternalClock(float maxsamples);

    /**
     * Set internal pulses per quarter note
     * @param ppqn_ New internal PPQN value
     */
    void SetInternalPPQN(int ppqn_);

    /**
     * Set external CV clock pulses per quarter note
     * @param ppqn_ New external PPQN value (minimum 1)
     */
    void SetExternalPPQN(int ppqn_);

    /**
     * Get current PPQN based on mode
     * @return Current active PPQN
     */
    int GetCurrentPPQN() const;

    void SetSampleRate(float samplerate_);

    inline float GetSampleRate() { return samplerate; };

    /**
     * Set samples per quarter note
     * @param spqn_ New SPQN value
     */
    void SetSPQN(int spqn_);

    /**
     * Set BPM
     * @param bpm Beats per minute
     */
    void SetBPM(float bpm);

    /**
     * Get current BPM
     * @return Current BPM
     */
    float GetBPM();

    /**
     * Set clock frequency
     * @param freq Frequency in Hz
     */
    void SetFreq(float freq);

    /**
     * Get current frequency
     * @return Current frequency in Hz
     */
    float GetFreq();

    /**
     * Set samples directly
     * @param samples Number of samples
     */
    void SetSamples(float samples);

    /**
     * Set timeout duration for external clock loss detection
     * @param seconds Timeout duration in seconds
     */
    void SetClockTimeout(float seconds);

    /**
     * Check if external clock is present
     * @return True if external clock is active
     */
    bool IsExternalClockPresent() const;

    /**
     * Get last valid tempo before clock loss
     * @return Last valid tempo in BPM
     */
    float GetLastValidTempo() const;

    /**
     * Validate clock state
     * @return True if clock state is valid
     */
    bool ValidateState() const;

    /**
     * Change clock mode with proper reinitialization
     * @param newMode The new mode to switch to
     */
    void SetMode(KaliClockMode newMode);

    /**
     * Initialize mode-specific settings
     */
    void InitializeMode();

    /**
     * Helper function to round up to nearest multiple
     * @param numToRound Number to round
     * @param multiple Multiple to round to
     * @return Rounded number
     */
    static inline int roundUp(int numToRound, int multiple)
    {
        if (multiple == 0)
            return numToRound;

        int remainder = numToRound % multiple;
        if (remainder == 0)
            return numToRound;

        return numToRound + multiple - remainder;
    }

    // Get musically-consistent timing base regardless of clock mode
    float GetSamplesPerBeat() const
    {
        switch (Mode)
        {
        case KaliClockMode::MidiClock:
            // MIDI mode needs the 4x multiplier because MIDI clocks are 24 PPQN
            // while we're working in quarter notes (hence 24/4 = 6x ratio adjustment)
            return spqn * 4.0f;

        case KaliClockMode::ClockIn:
        case KaliClockMode::Internal:
        default:
            return spqn;
        }
    }

    // Helper for converting knob position to musical timing multiplier
    float GetTimingMultiplier(float knobValue) const
    {
        // Convert knob 0-1 position to musical divisions (1/24 note to whole note)
        return fmap(knobValue, 1.0f, 24.0f, Mapping::LOG);
    }

    /**
     * Get a human-readable description of what the current PPQN means musically
     * @return String description like "1/4 notes" or "1/16 triplets"
     */
    const char *GetCurrentPulseDescription() const
    {
        int ppqn = GetCurrentPPQN();

        // Common musical divisions
        switch (ppqn)
        {
        case 1:
            return "1 note";
        case 2:
            return "1/2 notes";
        case 4:
            return "1/4 notes";
        case 6:
            return "1/4 triplets";
        case 8:
            return "1/8 notes";
        case 12:
            return "1/8 triplets";
        case 16:
            return "1/16 notes";
        case 24:
            return "1/16 triplets";
        case 32:
            return "1/32 notes";
        case 48:
            return "1/32 triplets";
        default:
            if (ppqn % 3 == 0)
                return "triplet pulses";
            else
                return "pulses";
        }
    }

private:
    float samplerate = 48000.f;
};

#endif // KALICLOCK_H