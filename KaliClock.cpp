// Created with assistance from Claude 3.7 Sonnet
// https://claude.anthropic.com/chat/

#include <algorithm>
#include "KaliClock.h"

void KaliClock::InitializeMode()
{
    // Full initialization of mode-specific timing
    if (Mode != KaliClockMode::Internal)
    {
        // Don't set a default SPQN for external modes - wait for actual sync
        spqn = 0; // No timing until we receive external pulses
        last_valid_spqn = 0;

        // Initialize timing accumulators
        SampleAccumulator = 0;
        TriggerAccumulator = 0;
        TimingAccumulator = 0;
    }
    initialized = true;
}

void KaliClock::SetSampleRate(float samplerate_)
{
    samplerate = samplerate_ > 0 ? samplerate_ : 48000;
    one_ms = samplerate / 1000.f;
}

void KaliClock::Init(float samplerate_, size_t size_, int internal_ppqn_, int spqn_, KaliClockMode Mode_)
{
    initialized = false;
    SetSampleRate(samplerate_);
    size = size_;
    internal_ppqn = internal_ppqn_;
    midi_ppqn = MIDI_PPQN; // Fixed at 24

    // Default tempo initialization
    float default_bpm = 120.0f;
    float samples_per_minute = samplerate * 60.f;
    int default_spqn = static_cast<int>(samples_per_minute / (default_bpm * GetCurrentPPQN()));

    // Use provided SPQN if valid, otherwise use default
    if (spqn_ > 0)
    {
        SetSPQN(spqn_);
    }
    else
    {
        SetSPQN(default_spqn);
    }

    last_valid_spqn = spqn;                              // Store initial SPQN as last valid
    clock_timeout = static_cast<int>(samplerate * 0.5f); // 500ms timeout
    clock_timeout_counter = 0;
    Mode = Mode_;

    // Initialize averaging buffer with default tempo values if in external mode
    foravgindex = 0;
    if (Mode != KaliClockMode::Internal)
    {
        // Fill averaging buffer with default SPQN value
        full = 1; // Start with one entry
        foravg[0] = spqn;
        for (int i = 1; i < foravgsize; i++)
        {
            foravg[i] = 0;
        }
    }
    else
    {
        full = 0;
        for (int i = 0; i < foravgsize; i++)
        {
            foravg[i] = 0;
        }
    }

    // Initialize state variables with non-zero values for external modes
    gate = false;
    gate_timer = 0;
    previous_trigger = false;
    TriggerAccumulator = 0;
    SampleAccumulator = 0;
    TimingAccumulator = 0;

    InitializeMode();
}

bool KaliClock::Tick(bool TriggerReceived)
{
    // Handle gate timing
    if (gate_timer > 0)
    {
        gate_timer--;
        gate = true;
    }
    else
    {
        gate = false;
    }

    // For external modes - detect incoming timing but generate unified outgoing clock
    if (Mode != KaliClockMode::Internal)
    {
        // Always increment our output clock accumulator
        SampleAccumulator++;

        // Always increment timing detection accumulator
        TimingAccumulator++;

        // Only process trigger on rising edge (not while gate is held high)
        bool trigger_edge = TriggerReceived && !previous_trigger;
        previous_trigger = TriggerReceived;

        if (trigger_edge)
        {
            // Reset timeout counter when we receive a trigger
            clock_timeout_counter = 0;

            // Process timing detection to update our base quarter note timing (spqn)
            ProcessTrigger();
        }
        else
        {
            // No trigger received - increment timeout
            clock_timeout_counter++;
        }

        // Generate outgoing clock using the same logic as internal mode
        // This ensures external and internal modes work identically for users
        if (spqn > 0 && internal_ppqn > 0)
        {
            float samples_per_output_pulse = spqn / static_cast<float>(internal_ppqn);
            if (SampleAccumulator >= samples_per_output_pulse)
            {
                TriggerAccumulator++;
                SampleAccumulator = 0;

                if (TriggerAccumulator >= internal_ppqn)
                {
                    gate = true;
                    gate_timer = static_cast<int>(one_ms * 5); // 5ms gate pulse
                    TriggerAccumulator = 0;
                    return true;
                }
            }
        }
        return false;
    }
    // Internal mode - works as before
    else
    {
        SampleAccumulator++;
        return ProcessInternalClock(spqn / static_cast<float>(internal_ppqn));
    }
}

void KaliClock::ProcessTrigger()
{
    // Don't generate gate here - that's handled by the unified clock generation
    // This function is purely for timing detection

    // If this is the first trigger, don't try to calculate timing yet
    if (full == 0)
    {
        full++;
        TimingAccumulator = 0;
        return;
    }

    // Store the raw sample count for timing calculations
    int rawSampleCount = TimingAccumulator;

    // Update moving average only if the accumulated samples are reasonable
    if (rawSampleCount > 0 && rawSampleCount < samplerate) // Limit to reasonable range
    {
        // Convert incoming pulse timing to quarter note timing (spqn)
        int scaledCount;

        if (Mode == KaliClockMode::ClockIn && external_ppqn > 1)
        {
            // External clock: scale up to quarter note duration
            // external_ppqn tells us how many pulses per quarter note we expect
            scaledCount = rawSampleCount * external_ppqn;
        }
        else if (Mode == KaliClockMode::MidiClock)
        {
            // MIDI clock: scale up to quarter note duration (24 PPQN standard)
            scaledCount = rawSampleCount * MIDI_PPQN;
        }
        else
        {
            // Single-pulse-per-quarter modes or ClockIn with PPQN=1
            scaledCount = rawSampleCount;
        }

        // Add to moving average buffer
        foravg[foravgindex] = scaledCount;
        foravgindex = (foravgindex + 1) % foravgsize;
        if (full < foravgsize)
            full++;

        // Calculate simple average
        float sum = 0;
        int count = full;
        for (int i = 0; i < count; i++)
        {
            sum += foravg[i];
        }
        float avgSamples = sum / count; // TEMPORARY: Disable smoothing to see raw timing data
        // Use direct average value with minimal bounds checking
        int new_spqn = static_cast<int>(avgSamples);

        // Only prevent obviously broken values (system limits)
        // Min: 10 samples (insanely fast, ~4800 BPM at 48kHz)
        // Max: 10 million samples (insanely slow, ~0.3 BPM at 48kHz)
        if (new_spqn >= 10 && new_spqn <= 10000000)
        {
            spqn = new_spqn;
        }

        // Store for timeout recovery
        if (spqn > 0)
        {
            last_valid_spqn = spqn;
        }
    }

    // Reset timing accumulator for next measurement
    TimingAccumulator = 0;
}

bool KaliClock::ProcessInternalClock(float maxsamples)
{
    if (SampleAccumulator >= maxsamples)
    {
        TriggerAccumulator++;
        SampleAccumulator = 0;

        if (TriggerAccumulator >= GetCurrentPPQN())
        {
            gate = true;
            gate_timer = static_cast<int>(one_ms * 5); // 5ms gate pulse
            TriggerAccumulator = 0;
            return true;
        }
    }
    return false;
}

void KaliClock::SetInternalPPQN(int ppqn_)
{
    if (ppqn_ > 0)
    {
        internal_ppqn = ppqn_;
    }
}

void KaliClock::SetExternalPPQN(int ppqn_)
{
    // Ensure we never have a zero or negative PPQN value
    if (ppqn_ > 0)
    {
        external_ppqn = ppqn_;
    }
    else
    {
        external_ppqn = 1; // Default to 1 as minimum
    }
}

int KaliClock::GetCurrentPPQN() const
{
    switch (Mode)
    {
    case KaliClockMode::MidiClock:
        return midi_ppqn; // Fixed at 24 PPQN (MIDI standard)
    case KaliClockMode::ClockIn:
        return external_ppqn; // Use dedicated external CV clock PPQN
    case KaliClockMode::Internal:
    default:
        return internal_ppqn; // Internal clock PPQN
    }
}
void KaliClock::SetSPQN(int spqn_)
{
    if (spqn_ > 0)
    {
        spqn = spqn_;
    }
}

float KaliClock::GetBPM()
{
    if (spqn <= 0)
    {
        return 120.0f; // Default BPM
    }

    float samples_per_minute = samplerate * 60.f;

    // spqn always represents samples per quarter note regardless of mode
    // This is because ProcessTrigger scales incoming timing to quarter notes
    float calculated_bpm = samples_per_minute / spqn;

    // Round to nearest integer to avoid display jumps for small fluctuations
    return roundf(calculated_bpm);
}

void KaliClock::SetFreq(float frequency)
{
    if (frequency > 0)
    {
        int new_spqn = static_cast<int>(samplerate / frequency);
        if (new_spqn > 0)
        {
            spqn = new_spqn;
        }
    }
}

float KaliClock::GetFreq()
{
    if (spqn <= 0)
    {
        return 1.0f; // Default 1Hz if no valid timing
    }
    return samplerate / spqn;
}

void KaliClock::SetBPM(float bpm)
{
    if (bpm > 0)
    {
        float samples_per_minute = samplerate * 60.f;
        int new_spqn = static_cast<int>(samples_per_minute / (bpm * GetCurrentPPQN()));
        if (new_spqn > 0)
        {
            spqn = new_spqn;
        }
    }
}

void KaliClock::SetSamples(float samples)
{
    if (samples > 0)
    {
        // Convert raw samples to SPQN (samples per quarter note)
        int current_ppqn = GetCurrentPPQN();
        if (current_ppqn > 0)
        {
            spqn = static_cast<int>(samples * current_ppqn);
        }
    }
}

void KaliClock::SetClockTimeout(float seconds)
{
    if (seconds > 0)
    {
        clock_timeout = static_cast<int>(samplerate * seconds);
    }
}

bool KaliClock::IsExternalClockPresent() const
{
    return clock_timeout_counter < clock_timeout;
}

float KaliClock::GetLastValidTempo() const
{
    int current_ppqn = GetCurrentPPQN();
    if (last_valid_spqn > 0 && current_ppqn > 0)
    {
        float samples_per_minute = samplerate * 60.f;
        return samples_per_minute / (last_valid_spqn * current_ppqn);
    }
    return 120.0f; // Default to 120 BPM if no valid timing
}

bool KaliClock::ValidateState() const
{
    return initialized && samplerate > 0 && GetCurrentPPQN() > 0 && spqn > 0;
}

void KaliClock::SetMode(KaliClockMode newMode)
{
    if (Mode != newMode)
    {
        Mode = newMode;
        InitializeMode();
    }
}