// Created as part of conversation: https://claude.ai/chat/...

#pragma once
#ifndef KALI_PLAYHEAD_ENGINE_H
#define KALI_PLAYHEAD_ENGINE_H

#include <cmath>
#include <array>
#include "daisysp.h"
#include "KaliDelayLine.h"

using namespace daisysp;

// Forward declarations
class Kali;
struct KaliInputState;

class KaliPlayheadEngine
{
public:
    // Type aliases for clarity
    using sample_t = float;
    using normalized_t = float; // Range [-1.0, 1.0]

    // Constants
    static constexpr sample_t kMinDelayMs = 1.0f;   // 1ms minimum
    static constexpr sample_t kMaxDelayMs = 300.0f; // 300ms maximum
    static constexpr size_t kNumChannels = 4;

    struct PlayheadParameters
    {
        sample_t scan_rate;
        sample_t play_rate;
        sample_t base_delay;
        sample_t right_delay;
        bool is_unlinked;
    };

    KaliPlayheadEngine()
        : samplerate_{48000.0f},
          phase_amount_{0.0f},
          last_position_{0.0f},
          last_position_r_{0.0f},
          min_delay_samps_{kMinDelayMs * (samplerate_ / 1000.0f)},
          max_delay_samps_{kMaxDelayMs * (samplerate_ / 1000.0f)}
    {
    }

    void Init(sample_t samplerate);
    void SetPhaseAmount(normalized_t amount);
    void Process(KaliInputState &state);

private:
    sample_t samplerate_;
    Phasor scan_phasor_;
    Phasor play_phasor_;

    normalized_t phase_amount_;
    sample_t last_position_;
    sample_t last_position_r_;

    sample_t min_delay_samps_;
    sample_t max_delay_samps_;

    [[nodiscard]] PlayheadParameters CalculateParameters(const KaliInputState &state) const;
    [[nodiscard]] static normalized_t NormalizePhasorOutput(sample_t phasor_out);
    [[nodiscard]] sample_t ProcessRightChannel(const KaliInputState &state, normalized_t position, sample_t right_delay);
};

#endif // KALI_PLAYHEAD_ENGINE_H