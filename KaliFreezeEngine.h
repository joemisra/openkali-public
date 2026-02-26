// Created with assistance from Claude 3.5 Sonnet
// https://claude.anthropic.com/chat/...

#pragma once
#ifndef KALI_FREEZE_ENGINE_H
#define KALI_FREEZE_ENGINE_H

#include "KaliDelayLine.h"
#include "Kali.h"
#include "daisysp.h"
#include <array>

using namespace daisysp;

class KaliFreezeEngine
{
public:
    static constexpr size_t MAX_FREEZE_GRAINS = 8;
    static constexpr float MIN_GRAIN_SIZE = 1000.0f;  // ~20ms at 48kHz
    static constexpr float MAX_GRAIN_SIZE = 24000.0f; // ~500ms at 48kHz

    struct FreezeGrain
    {
        bool active = false;
        float position = 0.0f;       // Current read position
        float increment = 1.0f;      // Speed of grain playback
        float amplitude = 0.0f;      // Current amplitude
        float size = MIN_GRAIN_SIZE; // Size in samples
        float crossfade_pos = 0.0f;  // Position in crossfade (0-1)
    };

    void Init(float sample_rate)
    {
        samplerate = sample_rate;
        freeze_active = false;
        current_grain = 0;
        grain_spacing = MIN_GRAIN_SIZE * 0.5f;

        // Initialize grains
        for (auto &grains : freeze_grains)
        {
            for (auto &grain : grains)
            {
                grain.active = false;
                grain.position = 0.0f;
                grain.increment = 1.0f;
                grain.amplitude = 0.0f;
                grain.size = MIN_GRAIN_SIZE;
                grain.crossfade_pos = 0.0f;
            }
        }
    }

    void SetFreezeEnabled(bool enabled)
    {
        if (enabled != freeze_active)
        {
            freeze_active = enabled;
            if (enabled)
            {
                // When freeze is activated, trigger initial grains
                for (size_t ch = 0; ch < 2; ch++)
                {
                    TriggerGrain(ch, buffer_position[ch]);
                }
            }
        }
    }

    void Process(KaliDelayLine<float, MAX_DELAY> *delays[4], float *wet, float meta1, float meta2)
    {
        if (!freeze_active)
        {
            return;
        }

        // Update parameters based on meta knobs
        float grain_size = fmap(meta1, MIN_GRAIN_SIZE, MAX_GRAIN_SIZE);
        grain_spacing = grain_size * fmap(meta2, 0.25f, 0.75f);

        // Process each stereo channel
        for (size_t ch = 0; ch < 2; ch++)
        {
            buffer_position[ch] = delays[ch]->GetWritePtr();

            // Check if we need to trigger new grain
            float elapsed = buffer_position[ch] - last_trigger_pos[ch];
            if (elapsed >= grain_spacing || elapsed < 0)
            {
                TriggerGrain(ch, buffer_position[ch]);
                last_trigger_pos[ch] = buffer_position[ch];
            }

            // Process all active grains
            float grain_mix = 0.0f;
            int active_count = 0;

            for (auto &grain : freeze_grains[ch])
            {
                if (grain.active)
                {
                    float grain_out = ProcessGrain(grain, delays[ch], buffer_position[ch], grain_size);
                    grain_mix += grain_out;
                    active_count++;
                }
            }

            // Normalize output
            if (active_count > 0)
            {
                wet[ch] = grain_mix / active_count;
                wet[ch + 2] = wet[ch]; // Copy to other channels
            }
        }
    }

private:
    float samplerate;
    bool freeze_active;
    std::array<std::array<FreezeGrain, MAX_FREEZE_GRAINS>, 2> freeze_grains;
    size_t current_grain;
    float grain_spacing;
    float buffer_position[2];
    float last_trigger_pos[2];

    void TriggerGrain(size_t channel, float delay_pos)
    {
        // Find next inactive grain or use round-robin
        size_t grain_idx = current_grain;
        bool found = false;

        for (size_t i = 0; i < MAX_FREEZE_GRAINS; i++)
        {
            if (!freeze_grains[channel][i].active)
            {
                grain_idx = i;
                found = true;
                break;
            }
        }

        if (!found)
        {
            grain_idx = current_grain;
            current_grain = (current_grain + 1) % MAX_FREEZE_GRAINS;
        }

        auto &grain = freeze_grains[channel][grain_idx];
        grain.active = true;
        grain.position = delay_pos;
        grain.increment = 1.0f;
        grain.amplitude = 1.0f;
        grain.crossfade_pos = 0.0f;
    }

    float ProcessGrain(FreezeGrain &grain, KaliDelayLine<float, MAX_DELAY> *delay,
                       float curr_pos, float grain_size)
    {
        if (!grain.active)
        {
            return 0.0f;
        }

        // Read from delay with interpolation
        float out = delay->ReadHermite(grain.position);

        // Update grain position with wrapping
        grain.position += grain.increment;
        if (grain.position >= curr_pos)
        {
            grain.position = curr_pos - grain_size;
        }

        // Update crossfade
        grain.crossfade_pos += 1.0f / grain_size;

        // Apply Hann window
        float window = 0.5f * (1.0f - cosf(M_PI * 2.0f * grain.crossfade_pos));
        out *= window;

        // Deactivate grain when crossfade complete
        if (grain.crossfade_pos >= 1.0f)
        {
            grain.active = false;
        }

        return out;
    }
};

#endif // KALI_FREEZE_ENGINE_H