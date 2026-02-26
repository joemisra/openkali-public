#include "dpt/daisy_dpt.h"
#include "Kali.h"
#include "daisysp.h"
#include "sys/system.h"

#include <memory>

void KaliOscillator::Init(float sample_rate)
{
    Oscillator::Init(sample_rate);
    adsr.Init(sample_rate);
    adsr.SetAttackTime(0.00001f);
    adsr.SetDecayTime(0.2f);
    adsr.SetSustainLevel(0.5f);
    adsr.SetReleaseTime(1.0f);
    cn.Init(sample_rate);

    // Initialize visualization buffer
    for (int i = 0; i < 64; i++)
    {
        last32[i] = 0.0f;
    }
    last32pos = 0;
    last32posstepped = 0;
    last32possteppedmax = 24; // Set default decimation rate for visualization

    // Initialize wavetable with defaults
    for (int i = 0; i < 16; i++)
    {
        wavetable[i] = 128; // Mid-point value
    }
    sample_rate_ = sample_rate;
}

void KaliOscillator::UpdateGate(bool gate_)
{
    if (!gate && gate_)
    {
        last_gate = true;

        // Update ADSR parameters from preset
        /* FIXME:
        adsr.SetAttackTime(GetParam(LFOOptionsPages::Attack));
        adsr.SetDecayTime(GetParam(LFOOptionsPages::Decay));
        adsr.SetSustainLevel(GetParam(LFOOptionsPages::Sustain));
        adsr.SetReleaseTime(GetParam(LFOOptionsPages::Release));
        */
    }
    else
    {
        last_gate = false;
    }
    gate = gate_;
}

float KaliOscillator::MetaDividerToFloat()
{
    if (meta_divider < 65)
        return meta / (float)meta_divider;
    else
        return (float)(66 - (meta_divider - 64)) * meta;
}

void KaliOscillator::PokeWavetable(uint8_t idx, uint8_t val)
{
    if (idx < 16)
    {
        wavetable[idx] = val;
    }
}

void KaliOscillator::ReadWavetableLERP(float pos)
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

    // Scale to LFO range
    last = interpolated * 8192.0f - 4096.0f;
    last_unscaled = last;
}

void KaliOscillator::Flip()
{
    flip = 1 - flip;
}

void KaliOscillator::SetFreq(float f)
{
    frequency = f;
}

void KaliOscillator::SetRandomWaveform()
{
    float randwf = (uint8_t)fmap(Random::GetFloat(), 0, WAVE_LAST - 1);
    SetWaveform(randwf);
    resetonnext = true;
}

void KaliOscillator::SetWaveform(uint8_t wf)
{
    preset.SetOption(LFOOptionsPages::Waveform, wf);
    Oscillator::SetWaveform(wf);
    waveform = wf;
}

void KaliOscillator::SetMode(unsigned int mode_)
{
    if (mode_ != mode && (mode_ == Kali::LFOModes::Polythene || mode_ == Kali::LFOModes::UnlinkedStraight))
        resetonnext = true;

    mode = mode_;
}

void KaliOscillator::SetLFOAdjust(float amt)
{
    // Modern implementation - store in preset
    preset.SetOption(LFOOptionsPages::Adjust, amt);
}

void KaliOscillator::SetParent(std::shared_ptr<Kali> parent)
{
    kali_ = parent;
}

void KaliOscillator::UpdateFollow()
{
    int channel_idx[6] = {0, 0, 0, 1, 1, 1};
    if (Follower && index >= 0 && index < 6)
    {
        follow_level = Follower->m_env[channel_idx[index]] * (lfo_adjust * 16.f);
    }
    else
    {
        follow_level = 0.0f;
    }
}

void KaliOscillator::UpdateLocalParams()
{
    if (!is_clock)
    {
        mode = preset.GetOption(LFOOptionsPages::LFOMode);
        meta = preset.GetOption(LFOOptionsPages::Meta);
        meta_divider = preset.GetOption(LFOOptionsPages::MetaDivider);

        // Ensure minimum valid values
        meta = std::max(meta, 1.0f);
        meta_divider = std::max(meta_divider, 1);

        lfo_adjust = preset.GetOption(LFOOptionsPages::Adjust);
        waveform = preset.GetOption(LFOOptionsPages::Waveform);
        bipolar = preset.GetOption(LFOOptionsPages::Polarity);
    }
    offset = preset.GetOption(LFOOptionsPages::Offset);
    attenuate = preset.GetOption(LFOOptionsPages::Attenuate);
    offset_cal = preset.GetOption(LFOOptionsPages::OffsetCal);
    attenuate_cal = preset.GetOption(LFOOptionsPages::AttenuateCal);
}

float KaliOscillator::Process()
{
    return Process(nullptr);
}

float KaliOscillator::Process(KaliOscillator *myfriends)
{
    // Save current settings that might be temporarily modified
    float rememberwf = waveform;
    float rememberedfreq = frequency;
    float metadonk = 50.0f; // Default multiplier
    bool clockbang = false;

    // Update parameters from preset
    UpdateLocalParams();

    // Handle phase reset if needed
    if (resetonnext)
    {
        PhaseAdd(0.166666666f * index);
        resetonnext = false;
    }

    // Clock division logic
    if (clockdiv >= (meta * 2))
    {
        clockdiv = 0;
        clockbang = true;
    }

    // Handle clock trigger events
    if (clockbang)
    {
        if (mt.Process(lfo_adjust))
        {
            flipTrack = 1 - flipTrack;
        }

        if (flipTrack && mode == Kali::LFOModes::RandReset)
        {
            Reset();
        }
        clockdiv = 0;
    }

    // Calculate frequency multiplier based on mode
    switch (mode)
    {
    case Kali::LFOModes::UnlinkedStraight:
        metadonk = meta;
        frequency = 100.0f;
        break;
    case Kali::LFOModes::Polythene:
        SetWaveform(Oscillator::WAVE_SQUARE);
        metadonk = MetaDividerToFloat();
        bipolar = 0;
        SetPw(0.1);
        break;
    case Kali::LFOModes::Glacier:
        metadonk = MetaDividerToFloat() / 64.f;
        break;
    default:
        metadonk = MetaDividerToFloat();
    }

    // Set noise frequency for modes that use it
    cn.SetFreq(frequency * metadonk * 4.f);

    // Apply jitter if in jitter mode
    if (mode == Kali::LFOModes::Jitter)
    {
        frequency *= abs(cn.Process()) * lfo_adjust;
    }

    // Apply FM modulation
    frequency += oneover4096 * fm * preset.GetOption(LFOOptionsPages::FMSourceAmount);

    // Set the final oscillator frequency
    Oscillator::SetFreq(frequency * metadonk * global_lfo_rate);

    // Get the base oscillator output
    float parentsays;
    if (waveform != Oscillator::WAVE_NOISE)
    {
        parentsays = last = last_unscaled = Oscillator::Process();
    }
    else
    {
        // Calculate phase increment based on frequency with minimum update rate
        float base_rate = frequency * metadonk * 50.0f;

        // Ensure minimum update rate of 1Hz + lfo_adjust control
        float effective_rate = fmaxf(1.0f, base_rate) * (1.0f + 9.0f * lfo_adjust);

        // Advance noise sample & hold phase
        float phase_increment = effective_rate / sample_rate_;
        noise_phase += phase_increment;

        // Generate new sample when phase wraps
        if (noise_phase >= 1.0f)
        {
            noise_phase -= floorf(noise_phase); // Handle multiple cycles
            noise_value = Random::GetFloat(-1.0f, 1.0f);
        }

        // Different noise types based on bipolar setting
        if (bipolar == 0)
        {
            // White noise with sample & hold
            parentsays = last = last_unscaled = noise_value * 4096.0f;
        }
        else if (bipolar == 1)
        {
            // Pink-ish noise (more musical)
            noise_filter = 0.7f * noise_filter + 0.3f * noise_value;
            parentsays = last = last_unscaled = noise_filter * 4096.0f;
        }
        else
        {
            // Smoothly interpolated noise (voltage-like)
            float smooth_amount = lfo_adjust;
            float smooth_noise = noise_value * (1.0f - smooth_amount) +
                                 noise_filter * smooth_amount;
            noise_filter = smooth_noise;
            parentsays = last = last_unscaled = smooth_noise * 4096.0f;
        }
    }

    // Apply mode-specific processing
    switch (mode)
    {
    case Kali::LFOModes::SyncSH:
        last = (cn.Process() * 4096.f);
        break;
    case Kali::LFOModes::Sidechain:
        last *= 1.f - follow_level;
        break;
    case Kali::LFOModes::FeedbackFollower:
        last = follow_level * 4096.f;
        break;
    case Kali::LFOModes::SyncTH:
        last = th.Process(flipTrack, parentsays, SampleHold::Mode::MODE_TRACK_HOLD);
        break;
    case Kali::LFOModes::Wavetable:
        ReadWavetableLERP(parentsays);
        break;
    case Kali::LFOModes::Raw:
        if (lfo_adjust < 0.01f)
            last = 0.0f;
        else if (lfo_adjust > 0.98)
            last = 4095.f;
        else
            last = lfo_adjust * 4096.f;
        break;
    default:
        last = parentsays;
    }

    // Apply polarity processing
    if (mode != Kali::LFOModes::Raw)
    {
        switch (bipolar)
        {
        case 0: // + unipolar
            last = last * 0.25f + 2048.f;
            break;
        case 1: // bipolar
            last = last * 0.5f + 2048.f;
            break;
        case 2: // - unipolar
            last = last * -0.25f + 2048.f;
            break;
        case 3: // inverted bipolar
            last = last * -0.5f + 2048.f;
            break;
        }
    }

    // Restore original settings
    Oscillator::SetWaveform(waveform); // Use the current waveform, not the remembered one
    SetFreq(rememberedfreq);

    // Handle end-of-cycle events
    if (IsEOC() && mode == Kali::LFOModes::RandShape)
    {
        const int max_cycles = static_cast<int>(lfo_adjust * 6.0f) + 1; // Ensure at least 1 cycle
        if (++eoc_count >= max_cycles)
        {
            SetRandomWaveform();
            eoc_count = 0;
        }
    }

    // Handle end-of-cycle actions for other oscillators
    if (IsEOC() && preset.GetOption(LFOOptionsPages::Eschaton) > 0.0f && myfriends != nullptr)
    {
        int action = static_cast<int>(preset.GetOption(LFOOptionsPages::Eschaton));
        int target = static_cast<int>(preset.GetOption(LFOOptionsPages::Incandenza));

        switch (action)
        {
        case 1: // Reset target oscillator
            myfriends[target].Reset();
            break;
        case 2: // Add 90° phase
            myfriends[target].PhaseAdd(0.25f);
            break;
        case 3: // Add 180° phase
            myfriends[target].PhaseAdd(0.50f);
            break;
        case 4: // Add 270° phase
            myfriends[target].PhaseAdd(0.75f);
            break;
        }
    }

    // Apply final scaling
    last = DSY_CLAMP(((last + offset) * (attenuate * 0.01f)), 0, 4095);

    // Store value for visualization with anti-aliasing decimation
    if (last32possteppedmax <= 0)
        last32possteppedmax = 1; // Prevent division by zero

    if (++last32posstepped >= last32possteppedmax)
    {
        // Convert from 0-4095 range to centered bipolar range for visualization
        float visualValue = (last - 2048.f) * -1.f;

        // Apply some smoothing for better visualization (optional)
        // if (last32pos > 0)
        // {
        //     visualValue = 0.7f * visualValue + 0.3f * last32[(last32pos - 1) % 64];
        // }

        last32[last32pos] = visualValue;

        // Wrap around buffer position
        last32pos = (last32pos + 1) % 64;
        last32posstepped = 0;
    }

    displayfrequency = frequency * metadonk;
    return last;
}
