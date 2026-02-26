#include "KaliMIDIClock.h"
#include <algorithm>

KaliMIDIClock::KaliMIDIClock()
    : state(STOPPED), pulse_count(0), total_pulse_count(0), phase(0.0f), samplerate(0.0f), last_pulse_time(0), current_bpm(120.0f), pulse_time_ms(0.0f), tempo_index(0), gate_active(false), gate_length_ms(2.0f) // Default 2ms gate length
      ,
      gate_time_remaining(0.0f), pulse_callback(nullptr), callback_data(nullptr)
{
    for (int i = 0; i < TEMPO_AVG_SIZE; i++)
    {
        tempo_buffer[i] = 120.0f; // Default 120 BPM
    }
}

void KaliMIDIClock::Init(float samplerate_)
{
    samplerate = samplerate_;
    Reset();
}

void KaliMIDIClock::HandleClock()
{
    if (state != RUNNING)
    {
        return;
    }

    // Update pulse count
    pulse_count = (pulse_count + 1) % PPQN;
    total_pulse_count++;

    // Calculate timing
    uint32_t current_time = static_cast<uint32_t>(total_pulse_count * 1000.0f * 60.0f / (current_bpm * PPQN));
    UpdateTempo(current_time);

    // Update phase
    UpdatePhase();

    // Trigger gate
    gate_active = true;
    gate_time_remaining = gate_length_ms * (samplerate / 1000.0f);

    // Call callback if set
    if (pulse_callback)
    {
        pulse_callback(callback_data);
    }
}

void KaliMIDIClock::HandleStart()
{
    Reset();
    state = RUNNING;
}

void KaliMIDIClock::HandleStop()
{
    state = STOPPED;
    gate_active = false;
    gate_time_remaining = 0.0f;
}

void KaliMIDIClock::HandleContinue()
{
    state = RUNNING;
}

void KaliMIDIClock::SetGateLength(float ms)
{
    gate_length_ms = std::max(0.1f, std::min(ms, 100.0f)); // Limit between 0.1ms and 100ms
}

void KaliMIDIClock::ProcessGate()
{
    if (gate_active && gate_time_remaining > 0.0f)
    {
        gate_time_remaining--;
        if (gate_time_remaining <= 0.0f)
        {
            gate_active = false;
        }
    }
}

void KaliMIDIClock::Reset()
{
    pulse_count = 0;
    total_pulse_count = 0;
    phase = 0.0f;
    gate_active = false;
    gate_time_remaining = 0.0f;
    last_pulse_time = 0;

    // Reset tempo buffer
    tempo_index = 0;
    float initial_tempo = (state == RUNNING) ? current_bpm : 120.0f;
    for (int i = 0; i < TEMPO_AVG_SIZE; i++)
    {
        tempo_buffer[i] = initial_tempo;
    }
}

void KaliMIDIClock::UpdateTempo(uint32_t current_time)
{
    if (last_pulse_time > 0)
    {
        // Calculate time between pulses
        pulse_time_ms = static_cast<float>(current_time - last_pulse_time);

        // Convert to BPM
        float instantaneous_bpm = 60000.0f / (pulse_time_ms * PPQN);

        // Update running average
        tempo_buffer[tempo_index] = instantaneous_bpm;
        tempo_index = (tempo_index + 1) % TEMPO_AVG_SIZE;

        // Calculate average BPM
        float sum = 0.0f;
        for (int i = 0; i < TEMPO_AVG_SIZE; i++)
        {
            sum += tempo_buffer[i];
        }
        current_bpm = sum / TEMPO_AVG_SIZE;
    }

    last_pulse_time = current_time;
}

void KaliMIDIClock::UpdatePhase()
{
    phase = static_cast<float>(pulse_count) / PPQN;
}