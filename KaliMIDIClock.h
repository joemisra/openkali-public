#pragma once
#ifndef KALI_MIDI_CLOCK_H
#define KALI_MIDI_CLOCK_H

#include <stdint.h>

class KaliMIDIClock
{
public:
    static const uint8_t PPQN = 24; // MIDI standard: 24 pulses per quarter note

    enum State
    {
        STOPPED,
        RUNNING,
        PAUSED
    };

    KaliMIDIClock();
    void Init(float samplerate);

    // Core MIDI message handlers
    void HandleClock();    // For MIDI clock pulse
    void HandleStart();    // For MIDI start message
    void HandleStop();     // For MIDI stop message
    void HandleContinue(); // For MIDI continue message

    // Clock state and timing
    bool IsRunning() const { return state == RUNNING; }
    float GetBPM() const { return current_bpm; }
    float GetPhase() const { return phase; }
    uint32_t GetPulseCount() const { return pulse_count; }
    float GetPulseTime() const { return pulse_time_ms; }

    // Gate output handling
    bool GetGate() const { return gate_active; }
    void SetGateLength(float ms);
    void ProcessGate(); // Should be called at sample rate

    // Callback for pulse events
    using PulseCallback = void (*)(void *data);
    void SetPulseCallback(PulseCallback cb, void *data = nullptr)
    {
        pulse_callback = cb;
        callback_data = data;
    }

private:
    // Clock state
    State state;
    uint32_t pulse_count;       // Counts pulses (0-23)
    uint32_t total_pulse_count; // Total pulses since start
    float phase;                // Current phase (0-1)

    // Timing
    float samplerate;
    uint32_t last_pulse_time; // For tempo calculation
    float current_bpm;
    float pulse_time_ms; // Time between pulses in ms

    // Tempo calculation
    static const uint8_t TEMPO_AVG_SIZE = 8;
    float tempo_buffer[TEMPO_AVG_SIZE];
    uint8_t tempo_index;
    void UpdateTempo(uint32_t current_time);

    // Gate handling
    bool gate_active;
    float gate_length_ms;
    float gate_time_remaining;

    // Callback
    PulseCallback pulse_callback;
    void *callback_data;

    void Reset();
    void UpdatePhase();
};

#endif // KALI_MIDI_CLOCK_H