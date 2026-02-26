// Created as part of conversation: https://claude.ai/chat/...

#include "KaliPlayheadEngine.h"
#include "KaliInputState.h"
#include "Kali.h"

void KaliPlayheadEngine::Init(sample_t samplerate)
{
    samplerate_ = samplerate;

    scan_phasor_.Init(samplerate);
    scan_phasor_.SetFreq(0.0f);

    play_phasor_.Init(samplerate);
    play_phasor_.SetFreq(0.0f);

    min_delay_samps_ = kMinDelayMs * (samplerate_ / 1000.0f);
    max_delay_samps_ = kMaxDelayMs * (samplerate_ / 1000.0f);
}

void KaliPlayheadEngine::SetPhaseAmount(normalized_t amount)
{
    phase_amount_ = std::fmin(1.0f, std::fmax(-1.0f, amount));
}

KaliPlayheadEngine::PlayheadParameters KaliPlayheadEngine::CalculateParameters(const KaliInputState &state) const
{
    PlayheadParameters params;

    // Meta1 controls scan rate from very slow to moderate speed
    // Using exponential mapping for more musical control
    params.scan_rate = fmap(state.inp->Meta, 0.01f, 2.0f, Mapping::EXP);

    // Meta2 controls playback rate/direction
    // Center is stopped (0.5 = 0.0 rate)
    // Left is reverse playback
    // Right is forward playback
    float centered_meta2 = (state.inp->Meta2 * 2.0f) - 1.0f; // Convert 0-1 to -1 to 1
    params.play_rate = centered_meta2 * 2.0f;                // Scale to reasonable playback rates

    // TimeL and TimeR map to delay buffer ranges
    params.base_delay = fmap(state.inp->TimeL, min_delay_samps_, max_delay_samps_);
    params.right_delay = fmap(state.inp->TimeR, min_delay_samps_, max_delay_samps_);
    params.is_unlinked = state.k->IsUnlinked();

    return params;
}

void KaliPlayheadEngine::Process(KaliInputState &state)
{
    PlayheadParameters params = CalculateParameters(state);

    // Update phasor rates
    scan_phasor_.SetFreq(params.scan_rate);
    play_phasor_.SetFreq(params.play_rate);

    // Calculate combined position
    normalized_t scan_pos = NormalizePhasorOutput(scan_phasor_.Process());
    normalized_t play_pos = NormalizePhasorOutput(play_phasor_.Process());
    normalized_t position = std::fmin(1.0f, std::fmax(-1.0f, scan_pos + (play_pos * phase_amount_)));

    // Calculate delay times and apply smoothing
    sample_t left_time = params.base_delay * (1.0f + position);
    fonepole(last_position_, left_time, 0.05f);

    // Read from delay lines
    sample_t out_l = state.delays[0]->ReadHermite(last_position_);
    sample_t out_r;

    if (params.is_unlinked)
    {
        out_r = ProcessRightChannel(state, position, params.right_delay);
    }
    else
    {
        out_r = state.delays[1]->ReadHermite(last_position_);
    }

    // Assign outputs and feedback
    for (size_t i = 0; i < kNumChannels; i += 2)
    {
        state.wet[i] = out_l;
        state.wet[i + 1] = out_r;
        state.whichout[i] = out_l;
        state.whichout[i + 1] = out_r;
    }
}

KaliPlayheadEngine::normalized_t KaliPlayheadEngine::NormalizePhasorOutput(sample_t phasor_out)
{
    return (phasor_out * 2.0f) - 1.0f;
}

KaliPlayheadEngine::sample_t KaliPlayheadEngine::ProcessRightChannel(
    const KaliInputState &state,
    normalized_t position,
    sample_t right_delay)
{
    sample_t right_time = right_delay * (1.0f + position);
    fonepole(last_position_r_, right_time, 0.05f);
    return state.delays[1]->ReadHermite(last_position_r_);
}