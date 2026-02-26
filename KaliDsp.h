#pragma once
#ifndef KALIDSP_H
#define KALIDSP_H

// Feature flags - set to 0 to disable and save flash
#define ENABLE_FFT_BLUR 0 // Disables SpectralBlur DSP mode

#include "KaliOscillator.h"
#include "KaliInput.h"
#include "KaliInputState.h"
#include "KaliGrain.h"
#include "KaliDelayLine.h"
#include "daisy.h"
#include "daisysp.h"
#include "KaliPlayheadEngine.h"
#include "dpt/daisy_dpt.h"
#define MAX_DELAY 1920000
#define MIN_DELAY 4
#define MAX_TAPS 3

using namespace daisysp;

// Single unified DSP class
class KaliDSP
{
public:
    // DSP modes enum combining all previous submodes
    enum DSPMode
    {
        // Basic delay modes
        Basic,
        PingPong,
        Unlinked,

        // Granular/chorus modes
        Resonator,
        Chorus,
        Knuth,
        /*PingPong2,
        PlayheadMode,
        MultiTap,*/
        Granular,
        GranularOctave,
        GranularTexture,
        GranularShimmer,
        GranularCrystals,
#if ENABLE_FFT_BLUR
        SpectralBlur,
#endif
        Fluid,

        // Waveshaping modes
        /*WaveFolder,
        Saturator,
        Quantizer,

        // Special modes
        SimpleChorus,
        OnlyReverb,
        ShimmerGrain,
        //*/

        DSP_MODE_LAST
    };

    // Common DSP parameters
    float wet[4];
    float whichout[4];
    float samplerate;
    float sample_interval; // 1.0f / samplerate cached at Init
    float last_choppe = 1.0f;
    float oldy[2];
    float oldx[2];
    CrossFade cf;
    Phasor phs[4];
    KaliPlayheadEngine playhead_engine;
    // Granular parameters
    static constexpr int MAX_GRAINS = 4;
    static constexpr float MIN_GRAIN_SIZE = 400.0f;
    static constexpr float MAX_GRAIN_SIZE = 4800.0f;

    struct Grain
    {
        bool active;
        float pos;
        float increment;
        float amp;
        float grain_size;
        float crossfade_pos;
    };

    Grain grains[2][MAX_GRAINS];
    int current_grain[2];
    float grain_phase_accumulator[2]; // Phase accumulator for grain triggering

    // Chorus parameters
    float chorusConst = (480.f / 96.f);
    KaliOscillator drama;
    KaliOscillator chorus[2];
    float last32[4][64];
    int last32pos;
    float phsL, phsR, panl, panr;

    // slicey meta 2
    int quantized_curmet2;
    int last_quantized_curmet2;
    float fadecountdown;
    float choppe;

    float zerocool[4]{0.01f, 0.01f, 0.01f, 0.01f};
    int jump_grace[4]{0, 0, 0, 0}; // short window to use linear interp after big jumps

    // Seamless loop crossfade state (used in granular modes)
    int loop_xfade_remaining[4]{0, 0, 0, 0};
    float loop_xfade_prev_out[4]{0.f, 0.f, 0.f, 0.f};
    int loop_xfade_len = 24; // ~0.5 ms @ 48k
    float last_output_sample[4]{0.f, 0.f, 0.f, 0.f};

    // Lightweight fluid field state for Fluid mode
    float fluid_pos[4][2]{{0.25f, 0.25f}, {0.75f, 0.25f}, {0.25f, 0.75f}, {0.75f, 0.75f}};
    float fluid_vel[4][2]{{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
    float fluid_theta1 = 0.0f;
    float fluid_theta2 = 1.5707963f; // 90deg offset

    // Minimal FDN reverb state (removed - ProcessWeirdVerb inactive)
    // static constexpr int FDN_TAPS = 8;
    // static constexpr int FDN_SIZE_MAX = 12000;
    // int fdn_idx[FDN_TAPS]{0, 0, 0, 0};
    // int fdn_len[FDN_TAPS]{1201, 1301, 1409, 1511, 1613, 1723, 1831, 1949};
    // float fdn_lp[FDN_TAPS]{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    inline float mstocoeff(float ms)
    {
        if (ms <= 0.0f)
            ms = 10.0f;
        return 1.0 / ((ms * 0.01f) * samplerate);
    };

    float pos, posr;
    // Methods
    void Init(float samplerate);
    void Process(KaliInputState s);
    const char *GetCurrentModeName();
    // Per-mode P1..P4 metadata access
    const char *GetParamLabel(int pindex) const; // 0..3
    struct ParamSpec
    {
        float min;
        float max;
        uint8_t map; // 0 = linear, 1 = exp
        char unit;   // e.g., '%', 's', 'm' (ms), 'H' (Hz)
        float def;   // suggested default in real units
    };
    const ParamSpec &GetParamSpec(int pindex) const; // 0..3

    // Helper methods
    void ProcessBasicDelay(KaliInputState &s);
#if ENABLE_FFT_BLUR
    void ProcessSpectralBlur(KaliInputState &s);
#endif
    void ProcessResonator(KaliInputState &s);

    void ProcessWaveshaping(KaliInputState &s);
    void TriggerGrain(int channel, float base_delay_samples, float position_spread);
    float ProcessGrain(Grain &g, KaliDelayLine<float, MAX_DELAY> *delay, float base_delay_samples);
    void Allpass(float &wetl, float &wetr, float c);

    int MIDIDelayBufferLength(int midinote)
    {
        return static_cast<int>((samplerate / floorf(mtof(midinote))));
    }

    static inline float triangleFunc(float pos)
    {
        return (1.f - pos) * 0.5f + 0.25f;
    }

    // Wave shaping functions
    static float foldback(float in, float threshold);
    static float quantizer(float in, float A, float B);
    static float modulo(float x, float A, float B);
    static float diode(float in, float factor);

    static void kill_denormal_by_quantization(float &val);

    void SetMode(unsigned int mode_)
    {
        mode = DSY_CLAMP(mode_, 0, DSP_MODE_LAST - 1);
    }

    unsigned int GetMode() const
    {
        return mode;
    }

private:
    unsigned int mode;
    void ProcessPhasorPitch(KaliInputState &s);
    void ProcessPhasorPitchOctave(KaliInputState &s);
    void ProcessGranularTexture(KaliInputState &s);
    void ProcessGranularShimmer(KaliInputState &s);
    void ProcessGranularCrystals(KaliInputState &s);
    void ProcessFluid(KaliInputState &s);

public:
    // Debug helpers
    int CountActiveGrains(int ch) const
    {
        int c = 0;
        for (int i = 0; i < MAX_GRAINS; ++i)
        {
            if (grains[ch][i].active)
                ++c;
        }
        return c;
    }

    // Get granular debug info
    bool HasActiveGrains() const { return CountActiveGrains(0) > 0 || CountActiveGrains(1) > 0; }
};

#endif
