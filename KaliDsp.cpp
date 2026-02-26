#include "KaliDsp.h"
#include <cstdlib> // For rand() and RAND_MAX
#include <cmath>   // For cosf and M_PI
#include <cstring> // For strncpy
#include <algorithm>
// Using CMSIS headers available via libDaisy includes; declare minimal prototypes to avoid explicit link
#define ENABLE_FFT_BLUR 0

using namespace daisy;
using namespace daisysp;

daisysp::Wavefolder wf;

// Helper: map 0..2 param to 0..1 range expected by many mappers
static inline float p01(float v2) { return DSY_CLAMP(v2 * 0.5f, 0.0f, 1.0f); }

// Helper: normalize a real value to 0..1 given ParamSpec mapping
static inline float norm_from_spec(const KaliDSP::ParamSpec &spec, float v)
{
    if (spec.map == 1)
    {
        // Exponential mapping; assume positive range
        float mn = DSY_MAX(1e-12f, spec.min);
        float mx = DSY_MAX(mn * (1.0f + 1e-6f), spec.max);
        float ratio = v / mn;
        float range = mx / mn;
        float t = logf(DSY_MAX(1e-12f, ratio)) / logf(DSY_MAX(1.0f + 1e-6f, range));
        return DSY_CLAMP(t, 0.0f, 1.0f);
    }
    else
    {
        float mn = spec.min;
        float mx = spec.max;
        if (mx - mn == 0.0f)
            return 0.0f;
        float t = (v - mn) / (mx - mn);
        return DSY_CLAMP(t, 0.0f, 1.0f);
    }
}

// FDN buffers removed - ProcessWeirdVerb inactive
// static float DSY_SDRAM_BSS g_fdn_buf[KaliDSP::FDN_TAPS][KaliDSP::FDN_SIZE_MAX];
// Granular crystal state
static float crystal_hold_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float crystal_target_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float crystal_step_timer[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float crystal_lp_state[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static bool crystal_initialized[4] = {false, false, false, false};
static float crystal_block_base[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float crystal_block_size[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static int crystal_subdivisions_state[4] = {1, 1, 1, 1};
static int crystal_steps_done[4] = {0, 0, 0, 0};
static float crystal_pitch_state[4] = {0.0f, 0.0f, 0.0f, 0.0f};
void KaliDSP::Init(float samplerate_)
{
    srand(0); // Seed the random number generator with a fixed value
    samplerate = samplerate_;
    sample_interval = 1.0f / DSY_MAX(1.0f, samplerate);

    // Initialize phasors
    for (int i = 0; i < 4; i++)
    {
        phs[i].Init(samplerate);
        phs[i].SetFreq(1.f);
    }

    // Initialize chorus
    for (int i = 0; i < 2; i++)
    {
        chorus[i].Init(samplerate);
        chorus[i].SetWaveform(daisysp::Oscillator::WAVE_TRI);
        chorus[i].SetAmp(1.0f);
        chorus[i].SetFreq(2.0f);
        chorus[i].meta = 1;
        chorus[i].last32possteppedmax = 384;
    }

    // Initialize drama oscillator
    drama.Init(samplerate * 4);
    drama.SetFreq(2.4);
    drama.SetAmp(1.0);

    // Initialize crossfade
    cf.Init(CROSSFADE_LIN);

    // Initialize FDN reverb lengths based on samplerate (~25–55ms lines)
    // REMOVED: FDN reverb no longer used after ProcessWeirdVerb deletion
    // int bases[FDN_TAPS] = {
    //     std::max(149, (int)(samplerate * 0.029f)),
    //     std::max(173, (int)(samplerate * 0.033f)),
    //     std::max(199, (int)(samplerate * 0.037f)),
    //     std::max(227, (int)(samplerate * 0.041f)),
    //     std::max(251, (int)(samplerate * 0.044f)),
    //     std::max(281, (int)(samplerate * 0.047f)),
    //     std::max(311, (int)(samplerate * 0.051f)),
    //     std::max(347, (int)(samplerate * 0.055f)),
    // };
    // for (int i = 0; i < FDN_TAPS; ++i)
    //     fdn_len[i] = std::min(bases[i], FDN_SIZE_MAX - 1);
    // for (int i = 0; i < FDN_TAPS; ++i)
    // {
    //     fdn_idx[i] = 0;
    //     fdn_lp[i] = 0.0f;
    //     for (int n = 0; n < fdn_len[i]; ++n)
    //         g_fdn_buf[i][n] = 0.0f;
    // }

    // Initialize grains
    for (int ch = 0; ch < 2; ch++)
    {
        for (int i = 0; i < MAX_GRAINS; i++)
        {
            grains[ch][i].active = false;
            grains[ch][i].pos = 0.0f;
            grains[ch][i].increment = 1.0f;
            grains[ch][i].amp = 0.0f;
            grains[ch][i].grain_size = MIN_GRAIN_SIZE;
            grains[ch][i].crossfade_pos = 0.0f;
        }
        current_grain[ch] = 0;
        grain_phase_accumulator[ch] = 0.0f; // Initialize grain phase accumulator
    }

    // Initialize fluid state
    for (int j = 0; j < 4; ++j)
    {
        float r1 = (float)rand() / (float)RAND_MAX;
        float r2 = (float)rand() / (float)RAND_MAX;
        fluid_pos[j][0] = 0.1f + 0.8f * r1;
        fluid_pos[j][1] = 0.1f + 0.8f * r2;
        fluid_vel[j][0] = 0.0f;
        fluid_vel[j][1] = 0.0f;
    }
    fluid_theta1 = 0.0f;
    fluid_theta2 = 1.5707963f;
    for (int j = 0; j < 4; ++j)
        jump_grace[j] = 0;
    for (int j = 0; j < 4; ++j)
    {
        loop_xfade_remaining[j] = 0;
        loop_xfade_prev_out[j] = 0.0f;
        last_output_sample[j] = 0.0f;
    }

    for (int j = 0; j < 4; ++j)
    {
        crystal_hold_pos[j] = 0.0f;
        crystal_target_pos[j] = 0.0f;
        crystal_step_timer[j] = 0.0f;
        crystal_lp_state[j] = 0.0f;
        crystal_initialized[j] = false;
        crystal_block_base[j] = 0.0f;
        crystal_block_size[j] = 0.0f;
        crystal_subdivisions_state[j] = 1;
        crystal_steps_done[j] = 0;
        crystal_pitch_state[j] = 0.0f;
    }
}

void KaliDSP::Process(KaliInputState s)
{
    // Common processing setup
    fonepole(last_choppe, floorf(s.curmet * 16.f) + 1.f, 0.001f);

    // Update chorus parameters: make depth/rate audible across most of the range
    for (int i = 0; i < 2; i++)
    {
        float rate_hz = daisysp::fmap(s.curmet, 0.1f, 5.0f, daisysp::Mapping::EXP);    // 0.1..5 Hz
        float depth = daisysp::fmap(s.curmet2, 0.05f, 1.0f, daisysp::Mapping::LINEAR); // 5%..100%
        chorus[i].SetFreq(rate_hz);
        chorus[i].SetAmp(depth);
        chorus[i].Process();
    }

    // Clear outputs
    for (int i = 0; i < 4; i++)
    {
        wet[i] = 0.0f;
        whichout[i] = 0.0f;
    }

    // Process based on mode
    switch (mode)
    {
    case Basic:
    case PingPong:
    case Unlinked:
    /*case SimpleChorus:*/
    case Chorus:
    case Knuth:
    case Resonator:
        ProcessBasicDelay(s);
        break;

    /*case PlayheadMode:
    {
        // Process playhead engine with both wet signals
        playhead_engine.Process(s);
        for (int j = 0; j < 4; j++)
            wet[j] = s.wet[j];
    }
    break;

    case MultiTap:*/
    case Granular:
        ProcessPhasorPitch(s);
        break;

    case GranularOctave:
        ProcessPhasorPitchOctave(s);
        break;

    case GranularTexture:
        ProcessGranularTexture(s);
        break;

    case GranularShimmer:
        ProcessGranularShimmer(s);
        break;

    case GranularCrystals:
        ProcessGranularCrystals(s);
        break;

#if ENABLE_FFT_BLUR
    case SpectralBlur:
        ProcessSpectralBlur(s);
        break;
#endif

    case Fluid:
        ProcessFluid(s);
        break;

        /*case ShimmerGrain:
            ProcessShimmerGrain(s);
            break;

        case WaveFolder:
        case Saturator:
        case Quantizer:
            ProcessWaveshaping(s);
            break;*/

    default:
        ProcessBasicDelay(s);
        break;
    }

    // Apply allpass only when enabled by option.
    if (s.allpass)
    {
        // Protect against NaN inputs
        for (int j = 0; j < 4; j++)
        {
            if (!isfinite(wet[j]))
            {
                wet[j] = 0.0f;
            }
        }

        float c = daisysp::fmap(s.inp->Feedback, 0.001f, 0.08f);
        Allpass(wet[0], wet[1], c);
    }

    // Apply final gain and routing
    for (int j = 0; j < 4; j++)
    {
        whichout[j] = wet[j];
        // Track last output per channel for seam crossfades
        last_output_sample[j] = wet[j];
    }
}

void KaliDSP::ProcessBasicDelay(KaliInputState &s)
{
    float mix[4];
    bool isPingPong = (mode == PingPong);

    //
    // float submodeamt = abs(s.inp->DelayAdjust);

    float samps[4], frequencybase[4];
    float read_position[4], crossfade_position[4], last_output[4];
    float crossfade_duration = 96.f;
    bool crossfade_active[4];
    const int slices = 24;

    for (int i = 0; i < 2; i++)
    {
        if (mode == Chorus)
        {
            // Use per-mode real units for Chorus P1/P2
            chorus[i].SetFreq(DSY_CLAMP(s.config_new[0], 0.01f, 20.0f)); // safety clamp
            chorus[i].SetAmp(DSY_CLAMP(s.config_new[1], 0.0f, 1.0f));
        }
        // Always advance oscillator (may be used as a modulation source elsewhere)
        chorus[i].Process();
    }

    float chr[2]{chorus[0].last_unscaled, chorus[1].last_unscaled};

    if (mode == Resonator)
    {
        int midinote;
        // if(s.notes_active[0] != 0)
        // midinote = s.notes_active[0];
        // else
        midinote = abs((floor(s.inp->TimeL * 88.f)) - 88.88f);
        s.delaytimes[0] = MIDIDelayBufferLength(midinote);

        // if(s.notes_active[0] != 0)
        //    midinote = s.notes_active[0];
        // else
        midinote = abs(floor(s.inp->TimeR * 88) - 88);
        // if linked bnk[1] = delayBufferLength(midinote + abs(s.inp->TimeR * 36.f));
        s.delaytimes[1] = MIDIDelayBufferLength(midinote);
        s.delaytimes[2] = s.delaytimes[0];
        s.delaytimes[3] = s.delaytimes[1];
    }

    quantized_curmet2 = floor(s.curmet2 * slices);

    float tmpmeta2quant = quantized_curmet2 / slices;
    for (int i = 0; i < 4; i++)
    {
        samps[i] = (s.delaytimes[i] / 48000.f);
        frequencybase[i] = 1.f / (samps[i] != 0 ? samps[i] : 1.f);
        phs[i].SetFreq(frequencybase[i] * tmpmeta2quant);
    }

    // submode = daisysp::fmap(submodeamt + 0.01, 0, STRAIGHT_SUBMODE_LAST - 1);
    // submode_next = std::min(STRAIGHT_SUBMODE_LAST - 1, submode + 1);
    // float crossfade_pos = fmod(submodeamt * STRAIGHT_SUBMODE_LAST, 1.f);

    if (quantized_curmet2 != last_quantized_curmet2 && fadecountdown == 0.0f)
    {
        fadecountdown = 1.0;
        last_choppe = choppe;
    }

    last_quantized_curmet2 = quantized_curmet2;

    if (fadecountdown > 0.0f)
    {
        // Much faster crossfade to minimize click duration
        fonepole(fadecountdown, 0.f, 8.0f); // Was 2.0f, now much faster
        cf.SetPos(1.f - fadecountdown);
    }
    else
    {
        cf.SetPos(0);
    }

    // s.warbr = (drama[0].Process() + 1.0f) / 2.f;
    // s.warbl = (drama[1].Process() + 1.0f) / 2.f;

    float step = 10.f;

    // used for crossfading
    float first[4];

    float phsp[4];

    for (int j = 0; j < 4; j++)
    {
        phsp[j] = phs[j].Process();
    }

    // Make META2 slew time almost instant for all modes (IDM-style warp cut effects)
    // Use very fast slew to avoid clicks but still smooth out zipper noise
    float meta2_slew_rate = 0.999f; // Almost instant - just enough to prevent clicks
    fonepole(choppe, (int)(s.curmet2 * step) + 1.0f, meta2_slew_rate);

    if (s.freeze)
    {
        float quantoffL = phsp[0] + ((floor((s.curmet + 1.f) * step) + 1.f) / step);
        float quantoffR = phsp[1] + ((floor((s.curmet + 1.f) * step)) / step);

        // Bypass smoothing while looping/freezing to avoid artifacts
        float pos_direct = fmodf(fastmod1f(quantoffL) * s.delaytimes[0], s.delaytimes[0]) * (s.curmet * 8.f);
        float posr_direct = fmodf(fastmod1f(quantoffR) * s.delaytimes[1], s.delaytimes[1]) * (s.curmet * 8.f);

        wet[0] = s.delays[0]->ReadHermite(pos_direct);
        wet[1] = s.delays[1]->ReadHermite(posr_direct);
        wet[2] = wet[0];
        wet[3] = wet[1];
    }
    else
    {
        // fonepole(choppe, , 0.5f);
        float fineadj = (s.curmet * s.delaytimes[0] * 0.1); // max range is +10% of left

        /*
        if (i == Water)
        {
            const float scale = 6000.f;
            float p[4];

            for (int j = 0; j < 4; j++)
            {
                p[j] = s.delaytimes[j] + s.curmet * scale;
                float mod = (s.curmet * 16.f) * 0.0625;
                float w = s.delays[j]->ReadHermite(p[j], mod);
                w += s.delays[j]->ReadHermite(p[j] * s.warb[j] * s.curmet2, mod);
                w *= 0.5f;
                wet[j] = w;
            }

            // metacontext[i][0] = metacontext[i][1] = (100.f * s.curmet2);
        }
        */
        /*
        else if (i == Fire)
        {
            float leftboop = s.l->ReadHermite((s.delaytimes[0] + (s.curmet * 2.f * 3000.f)) / choppe);
            float leftkoko = s.l->ReadHermite((s.delaytimes[1] + (s.curmet * 2.f * 3000.f)) / choppe);

            float rightboop = s.r->ReadHermite((s.delaytimes[0] + (s.curmet2 * 2.f * 3000.f)) / choppe);
            float rightkoko = s.r->ReadHermite((s.delaytimes[1] + (s.curmet2 * 2.f * 3000.f)) / choppe);

            // m + s delayed by boop
            float Mboop = (leftboop + rightboop) / sqrt2; // obtain mid-signal from left and right
            float Sboop = (leftboop - rightboop) / sqrt2; // obtain side-signal from left and right

            // m + s delayed by koko
            float Mkoko = (leftkoko + rightkoko) / sqrt2; // obtain mid-signal from left and right
            float Skoko = (leftkoko - rightkoko) / sqrt2; // obtain side-signal from left and right

            float tmpcurmet = s.curmet;
            // amplify mid and side signal seperately:
            Mboop *= 2 * (1 - tmpcurmet);
            Sboop *= 2 * tmpcurmet;
            Mkoko *= 2 * (1 - tmpcurmet);
            Skoko *= 2 * tmpcurmet;

            // float width2 = curmet2;
            // float widel = ((Mboop + Skoko) / sqrt2) * width2; // obtain left signal from mid and side
            // float wider = ((Mboop - Skoko) / sqrt2) * width2; // obtain right signal from mid and side
            float width2 = daisysp::fmap(s.curmet, 0.5f, 1.0f);
            float widel = ((Mboop + Skoko) / sqrt2) * width2; // obtain left signal from mid and side
            float wider = ((Mboop - Skoko) / sqrt2) * width2; // obtain right signal from mid and side

            wets[i][0] = tanh(widel * (1.7f - width2));
            wets[i][1] = tanh(wider * (1.7f - width2));

            metacontext[i][0] = (width2 * 100.f);
            metacontext[i][1] = choppe;
        }
        */
        /*
        else if(i == LCR)
        {
            wets[i][0] = s.l->ReadHermite(s.delaytimes[0] + (s.curmet * 2.f * 3000.f));
            wets[i][1] = s.r->ReadHermite(s.delaytimes[1] + (s.curmet * 2.f * 3000.f));

            wets[i][0] += s.l->ReadHermite(s.delaytimes[0] + s.delaytimes[1] + (s.curmet * 2.f * 3000.f));
            wets[i][1] += s.r->ReadHermite(s.delaytimes[0] + s.delaytimes[1] + (s.curmet * 2.f * 3000.f));

            wets[i][0] *= 0.5f;
            wets[i][1] *= 0.5f;
        }
        */
        /*
        if (mode == SimpleChorus)
        {
            float paramstemp[4];
            paramstemp[0] = s.curmet2 * (s.config_new[0] + 1.f);
            paramstemp[1] = s.curmet * paramstemp[0];

            /*
            for (int j = 0; j < 4; j += 2)
            {
                float pos = (s.delaytimes[j] * 0.1f);
                float pos2 = (s.delaytimes[j + 1] * 0.1f);

                wet[j] = s.dry[j] + s.delays[j]->Read(pos);
                wet[j + 1] = s.dry[j + 1] + s.delays[j + 1]->Read(pos2);
                wet[j] += s.delays[j]->Read(DSY_CLAMP(pos + (phsp[j] * s.curmet * paramstemp[0]), 4.f, s.MAX_DELAY_WORKING);
                wet[j + 1] += s.delays[j + 1]->Read(DSY_CLAMP(pos2 + (phsp[j + 1] * s.curmet * paramstemp[0]), 4.f, s.MAX_DELAY_WORKING));
                wet[j] *= 0.5f;
                wet[j + 1] *= 0.5f;
            }
            */
        /*
      for (int j = 0; j < 4; j += 2)
      {
          float pos = s.delaytimes[j];
          float pos2 = s.delaytimes[j + 1];
          float spread = s.config_new[1] > 0.f ? 1.f + s.config_new[1] : 1.0f;

          float clampedPos = DSY_CLAMP(pos + phsp[j] * s.curmet * paramstemp[0], 4.f, s.MAX_DELAY_WORKING);
          float clampedPos2 = DSY_CLAMP((pos2 + spread) - phsp[j + 1] * s.curmet2 * paramstemp[0], 4.f, s.MAX_DELAY_WORKING);
          wet[j] = s.delays[j]->ReadHermite(pos) + s.delays[j]->ReadHermite(clampedPos);
          wet[j + 1] = s.delays[j + 1]->ReadHermite(pos2) + s.delays[j + 1]->ReadHermite(clampedPos2);

          wet[j] *= 0.56f;
          wet[j + 1] *= 0.56f;
      }
  }
  */
        if (mode == Chorus)
        {
            // for (int j = 0; j < 4; j++)
            // {
            //     s.dp[j]->SetDelaySamples(s.delaytimes[j]); // Delay time between 20ms to 30ms
            //     s.dp[j]->SetModFreqRatio(0.1f);            // Modulation frequency around 0.5Hz
            //     s.dp[j]->SetModDepth(0.01f);               // Modulation depth around 0.75
            //     s.dp[j]->SetWritePtr(s.delays[j]->GetWritePtr());
            //     s.dp[j]->Process();
            // }

            for (int j = 0; j < 4; j += 2)
            {
                float depthL = 0.02f + 0.18f * fabsf(chr[0]); // up to ~20% modulation on delay time
                float depthR = 0.02f + 0.18f * fabsf(chr[1]);
                float posL1 = s.delaytimes[j];
                float posL2 = s.delaytimes[j] * (1.f + depthL);
                float posR1 = s.delaytimes[j + 1];
                float posR2 = s.delaytimes[j + 1] * (1.f + depthR);
                wet[j] = 0.5f * (s.delays[j]->ReadHermite(posL1) + s.delays[j]->ReadHermite(DSY_CLAMP(posL2, 4.f, s.MAX_DELAY_WORKING)));
                wet[j + 1] = 0.5f * (s.delays[j + 1]->ReadHermite(posR1) + s.delays[j + 1]->ReadHermite(DSY_CLAMP(posR2, 4.f, s.MAX_DELAY_WORKING)));
            }
        }
        else if (mode == Knuth && false)
        {
            // this is sort of a crystals effect that pulls back to the beginning sharply
            for (int j = 0; j < 4; j++)
            {
                s.dp[j]->SetDelaySamples(s.delaytimes[j]);
                // Map P1/P2 via ParamSpec to normalized 0..1
                float p1v = norm_from_spec(GetParamSpec(0), s.config_new[0]);
                float p2v = norm_from_spec(GetParamSpec(1), s.config_new[1]);
                s.dp[j]->SetModFreqRatio(0.01f + 0.09f * p1v);
                s.dp[j]->SetModDepth(0.01f + 0.09f * p2v);
                s.dp[j]->SetWritePtr(s.delays[j]->GetWritePtr());
                s.dp[j]->Process();
            }

            for (int j = 0; j < 4; j++)
            {
                fonepole(zerocool[j], s.delaytimes[j] - s.dp[j]->GetDelayPosition(), mstocoeff(5));
                wet[j] = s.delays[j]->ReadHermite(zerocool[j]);
            }
        }
        else
        {
            for (int j = 0; j < 4; j++)
                wet[j] = s.delays[j]->ReadHermite(s.delaytimes[j]);
        }

        // Ping-pong cross-feedback handled in main audio callback.
        /*
        float in[2], z[2], y[2];
        in[0] = wets[i][0] * 5.f * s.inp.Feedback;
        in[1] = wets[i][1] * 5.f * s.inp.Feedback;
        z[0] = tanh(in[0]) - oldy[0] * 0.05f;//+ (tanh(oldy[0]) * 0.9f);
        z[1] = tanh(in[1]) - oldy[1] * 0.05f;// + (tanh(oldy[1]) * 0.9f);
        y[0] = (z[0] + -0.05f * sin(in[0] * TWOPI_F * 24000.f / 48000.f));
        y[1] = (z[1] + -0.05f * sin(in[1] * TWOPI_F * 24000.f / 48000.f));

        wets[i][0] = (y[0] * (1.f - s.inp.Feedback / 2.5f)) * 0.5f;
        wets[i][1] = (y[1] * (1.f - s.inp.Feedback / 2.5f)) * 0.5f;
        oldy[0] = y[0];
        oldy[1] = y[1];
        */
    }

    // at one point modes could be crossfaded during changes
    // for(int j=0;j<4;j++)
    //    wet[j] = wets[submode][j];

    // for(int j=0;j<4;j++)
    //     wet[j] = mix[j];
    //  float widel, wider;
}

void KaliDSP::ProcessWaveshaping(KaliInputState &s)
{
    float in[2], out[2];

    // Get input
    in[0] = s.dry[0];
    in[1] = s.dry[1];

    switch (mode)
    {
        /*
    case WaveFolder:
        for (int ch = 0; ch < 2; ch++)
        {
            float gain = fmap(s.curmet2, 1.f, 64.f, daisysp::Mapping::LOG);
            float threshold = fmap(s.curmet, 8.0f, 1.0f);
            out[ch] = foldback(in[ch] * gain, threshold) / threshold;
        }
        break;

    case Saturator:
        for (int ch = 0; ch < 2; ch++)
        {
            float gain = fmap(s.curmet2, 1.f, 64.f, daisysp::Mapping::LOG);
            out[ch] = tanhf(in[ch] * gain);
        }
        break;

    case Quantizer:
        for (int ch = 0; ch < 2; ch++)
        {
            out[ch] = quantizer(in[ch], s.curmet, s.curmet2);
        }
        break;
    }
    */

        // Apply to output
        wet[0] = out[0];
        wet[1] = out[1];
        wet[2] = out[0];
        wet[3] = out[1];
    }
}

#if ENABLE_FFT_BLUR
// Prototype FFT-based spectral blur (small frame, low overlap)
void KaliDSP::ProcessSpectralBlur(KaliInputState &s)
{
    // Ultra-conservative time-domain blur to prevent buffer glitching
    float temporal = DSY_CLAMP(s.curmet, 0.0f, 1.0f);
    // Very limited taps: 1..3 max to prevent CPU overload
    int taps = 1 + (int)(temporal * 6.0f);
    taps = DSY_CLAMP(taps, 1, 3);

    // Meta2 controls spacing (1..3 samples max)
    int spacing = 1 + (int)(s.curmet2 * 5.0f);
    spacing = DSY_CLAMP(spacing, 1, 3);

    for (int ch = 0; ch < 4; ch++)
    {
        float acc = 0.0f;
        float norm = 0.0f;

        for (int t = 0; t < taps; t++)
        {
            float p = s.delaytimes[ch] - (float)(t * spacing);
            // Extra safety margins on position bounds
            p = DSY_CLAMP(p, 16.0f, s.MAX_DELAY_WORKING - 16.0f);
            float w = (float)(taps - t) / (float)taps; // Triangular weight
            float x = s.delays[ch]->ReadHermite(p);

            // Safety check for invalid samples
            if (!isfinite(x))
                x = 0.0f;
            if (fabsf(x) > 10.0f)
                x = 0.0f; // Clamp extreme values

            acc += w * x;
            norm += w;
        }

        // Safety checks on accumulator
        if (!isfinite(acc) || !isfinite(norm) || norm < 1e-6f)
        {
            wet[ch] = 0.0f;
            whichout[ch] = 0.0f;
            continue;
        }

        float out = acc / norm;

        // Denormal guard
        out += 1e-9f;
        out -= 1e-9f;

        // Very aggressive headroom and limiting
        out *= 0.5f;             // Much more headroom
        out = tanhf(out * 0.7f); // Double soft limiting

        wet[ch] = out;
        whichout[ch] = wet[ch];
    }
}
#endif // ENABLE_FFT_BLUR

static char DSY_SDRAM_BSS dsp_mode_names[13][16]; // modes, max 16 chars each
static bool dsp_names_initialized = false;
// Per-mode parameter metadata (labels + specs) - Shortened to save flash
static const char *k_param_labels[KaliDSP::DSP_MODE_LAST][4] = {
    // Basic
    {"P1", "P2", "P3", "P4"},
    // PingPong
    {"P1", "P2", "P3", "P4"},
    // Unlinked
    {"P1", "P2", "P3", "P4"},
    // Resonator
    {"Rate", "Dpth", "Damp", "Mix"},
    // Chorus
    {"Rate", "Dpth", "Stro", "Colr"},
    // Knuth
    {"MRate", "MDpth", "Colr", "Smth"},
    // Granular
    {"MRate", "MDpth", "Ptch", "Smth"},
    // GranOctave
    {"MRate", "MDpth", "Ptch", "Smth"},
    // GranTexture
    {"MRate", "MDpth", "Colr", "Blnd"},
    // GranShimmer
    {"MRate", "MDpth", "Colr", "Blnd"},
    // GranCrystals
    {"Rate", "Size", "Colr", "Edge"},
#if ENABLE_FFT_BLUR
    // SpectralBlur
    {"Tmpl", "Spc", "Colr", "Blnd"},
#endif
    // Fluid
    {"Flow", "Visc", "Coup", "Turb"},
};

static const KaliDSP::ParamSpec k_param_specs[KaliDSP::DSP_MODE_LAST][4] = {
    // Basic
    {{0, 2, 0, '%', 1}, {0, 2, 0, '%', 1}, {0, 2, 0, '%', 1}, {0, 2, 0, '%', 1}},
    // PingPong
    {{0, 2, 0, '%', 1}, {0, 2, 0, '%', 1}, {0, 2, 0, '%', 1}, {0, 2, 0, '%', 1}},
    // Unlinked
    {{0, 2, 0, '%', 1}, {0, 2, 0, '%', 1}, {0, 2, 0, '%', 1}, {0, 2, 0, '%', 1}},
    // Resonator
    {{0.1f, 5.f, 1, 'H', 1.0f}, {0.0f, 1.0f, 0, '%', 0.25f}, {0.0f, 1.0f, 0, '%', 0.25f}, {0.0f, 1.0f, 0, '%', 0.5f}},
    // Chorus
    {{0.1f, 5.f, 1, 'H', 0.5f}, {0.05f, 1.0f, 0, '%', 0.25f}, {0.0f, 1.0f, 0, '%', 0.5f}, {0.0f, 1.0f, 0, '%', 0.5f}},
    // Knuth
    {{0.01f, 0.1f, 1, 'H', 0.02f}, {0.01f, 0.1f, 1, '%', 0.02f}, {0.0f, 1.0f, 0, '%', 0.25f}, {0.0f, 1.0f, 0, '%', 0.25f}},
    // Granular
    {{0.0f, 0.1f, 1, 'H', 0.02f}, {0.0f, 1.0f, 0, '%', 0.2f}, {-12.f, 12.f, 0, 's', 0.0f}, {0.0f, 1.0f, 0, '%', 0.1f}},
    // GranOctave
    {{0.0f, 0.1f, 1, 'H', 0.02f}, {0.0f, 1.0f, 0, '%', 0.2f}, {-2.f, 2.f, 0, 's', 0.f}, {0.0f, 1.0f, 0, '%', 0.1f}},
    // GranTexture
    {{0.0f, 0.05f, 1, 'H', 0.01f}, {0.1f, 0.4f, 0, '%', 0.2f}, {0.0f, 1.0f, 0, '%', 0.2f}, {0.0f, 0.2f, 0, '%', 0.1f}},
    // GranShimmer
    {{0.05f, 0.2f, 1, 'H', 0.08f}, {0.3f, 0.8f, 0, '%', 0.45f}, {0.0f, 1.0f, 0, '%', 0.2f}, {0.1f, 0.3f, 0, '%', 0.2f}},
    // GranCrystals
    {{1.0f, 32.0f, 1, 'H', 2.0f}, {4.0f, 64.0f, 1, 'm', 12.0f}, {0.0f, 1.0f, 0, '%', 0.1f}, {0.0f, 0.1f, 0, '%', 0.09f}},
#if ENABLE_FFT_BLUR
    // SpectralBlur
    {{0.0f, 1.0f, 0, '%', 0.25f}, {1.0f, 3.0f, 0, ' ', 1.0f}, {0.0f, 1.0f, 0, '%', 0.0f}, {0.0f, 1.0f, 0, '%', 0.0f}},
#endif
    // Fluid
    {{0.02f, 2.5f, 1, 'H', 0.2f}, {10.0f, 2000.0f, 1, 'm', 100.0f}, {0.0f, 1.0f, 0, '%', 0.2f}, {0.0f, 1.0f, 0, '%', 0.2f}},
};

const char *KaliDSP::GetCurrentModeName()
{
    if (!dsp_names_initialized)
    {
        // Initialize mode names in SDRAM (must match DSPMode order)
        const char *source_names[] = {
            "Basic", "PingPong", "Unlinked",
            "Resonator", "Chorus", "Knuth", "Granular",
            "GranOctave", "GranTexture", "GranShimmer", "GranCrystals",
            "SpBlur", "Fluid"};

        for (int i = 0; i < 13; i++)
        {
            strncpy(dsp_mode_names[i], source_names[i], 15);
            dsp_mode_names[i][15] = '\0';
        }
        dsp_names_initialized = true;
    }

    return dsp_mode_names[mode];
}

const char *KaliDSP::GetParamLabel(int pindex) const
{
    if (pindex < 0 || pindex > 3)
        return "P?";
    unsigned int m = GetMode();
    m = DSY_CLAMP(m, 0u, (unsigned int)DSP_MODE_LAST - 1u);
    return k_param_labels[m][pindex];
}

const KaliDSP::ParamSpec &KaliDSP::GetParamSpec(int pindex) const
{
    static const ParamSpec fallback{0.0f, 2.0f, 0, '%', 1.0f};
    if (pindex < 0 || pindex > 3)
        return fallback;
    unsigned int m = GetMode();
    m = DSY_CLAMP(m, 0u, (unsigned int)DSP_MODE_LAST - 1u);
    return k_param_specs[m][pindex];
}

// Navier–Stokes-inspired fluid modulation over read positions
void KaliDSP::ProcessFluid(KaliInputState &s)
{
    // Controls
    // P1..P4 are real units per ParamSpec
    float p1_hz = DSY_MAX(0.0f, s.config_new[0]);
    float p2_ms = DSY_MAX(0.0001f, s.config_new[1]);
    float p3_couple = DSY_CLAMP(s.config_new[2], 0.0f, 1.0f);
    float p4_turb = DSY_CLAMP(s.config_new[3], 0.0f, 1.0f);

    // Sample interval (cached)
    const float dt = sample_interval;

    // P1: flow rate in Hz (already mapped)
    float step = 2.0f * M_PI * p1_hz * dt;

    // P2: viscosity via time-constant in ms -> per-sample smoothing factor
    float tau = DSY_MAX(1e-6f, p2_ms * 0.001f);
    float visc = 1.0f - expf(-dt / tau);

    // P3: coupling depth with more low-end resolution
    float couple = 0.0005f + 0.10f * (p3_couple * p3_couple);

    // P4: turbulence amount with squared response for subtle control
    float turb_amt = 0.12f * (p4_turb * p4_turb);

    // Meta1: vorticity sign/intensity, Meta2: weight of 2nd vortex
    float vort = (s.curmet - 0.5f) * 2.0f; // -1..1
    float w2 = s.curmet2;                  // 0..1

    // Update two vortex centers along simple orbits
    fluid_theta1 += step * (0.8f + 0.2f * (vort >= 0 ? 1.0f : -1.0f));
    fluid_theta2 -= step * (0.6f + 0.2f * (vort >= 0 ? 1.0f : -1.0f));
    if (fluid_theta1 > 6.28318f)
        fluid_theta1 -= 6.28318f;
    if (fluid_theta2 < 0.0f)
        fluid_theta2 += 6.28318f;
    float c1x = 0.5f + 0.25f * cosf(fluid_theta1);
    float c1y = 0.5f + 0.25f * sinf(fluid_theta1);
    float c2x = 0.5f + 0.33f * cosf(fluid_theta2 + 1.3f);
    float c2y = 0.5f + 0.33f * sinf(fluid_theta2 + 0.7f);

    // Smoothed noise for turbulence (per channel)
    static float nx[4] = {0}, ny[4] = {0};
    float noise_tc_ms = daisysp::fmap(p4_turb, 10.0f, 200.0f); // 10..200 ms
    float noise_a = mstocoeff(noise_tc_ms);

    for (int j = 0; j < 4; ++j)
    {
        // Position and velocity in [0,1]^2
        float px = fluid_pos[j][0];
        float py = fluid_pos[j][1];

        // Vortex 1 contribution
        float r1x = px - c1x;
        float r1y = py - c1y;
        float d1 = r1x * r1x + r1y * r1y + 1e-4f;
        float p1x = -r1y;
        float p1y = r1x;
        float v1x = (p1x / d1);
        float v1y = (p1y / d1);

        // Vortex 2 contribution (weighted by Meta2)
        float r2x = px - c2x;
        float r2y = py - c2y;
        float d2 = r2x * r2x + r2y * r2y + 1e-4f;
        float p2x = -r2y;
        float p2y = r2x;
        float v2x = (p2x / d2);
        float v2y = (p2y / d2);

        // Base velocity field
        float vx = vort * (v1x * (1.0f - w2) + v2x * w2);
        float vy = vort * (v1y * (1.0f - w2) + v2y * w2);

        // Add small turbulence (smoothed random to avoid zippering)
        float tgtx = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
        float tgty = 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
        fonepole(nx[j], tgtx, noise_a);
        fonepole(ny[j], tgty, noise_a);
        vx += turb_amt * nx[j];
        vy += turb_amt * ny[j];

        // Semi-implicit Euler with viscosity damping
        fluid_vel[j][0] = (1.0f - visc) * (fluid_vel[j][0] + step * vx);
        fluid_vel[j][1] = (1.0f - visc) * (fluid_vel[j][1] + step * vy);

        px += fluid_vel[j][0];
        py += fluid_vel[j][1];

        // Wrap to [0,1]
        if (px < 0.0f)
            px += 1.0f;
        if (px > 1.0f)
            px -= 1.0f;
        if (py < 0.0f)
            py += 1.0f;
        if (py > 1.0f)
            py -= 1.0f;

        fluid_pos[j][0] = px;
        fluid_pos[j][1] = py;

        // Map (px,py) to an offset around the current delay time
        float center = 0.5f; // center of domain
        float dx = px - center;
        float dy = py - center;
        float scalar = (dx + dy) * 0.5f; // -0.5..0.5-ish
        float offset = scalar * couple * s.delaytimes[j];

        float read_pos = s.delaytimes[j] + offset;
        read_pos = DSY_CLAMP(read_pos, 4.0f, s.MAX_DELAY_WORKING - 4.0f);

        // Slight smoothing to avoid zippering
        fonepole(zerocool[j], read_pos, mstocoeff(6.0f));
        float x = s.delays[j]->ReadHermite(zerocool[j]);
        // Gentle soft clip
        wet[j] = tanhf(x * 0.98f);
    }
}

// Helper functions implementation
void KaliDSP::TriggerGrain(int channel, float base_delay_samples, float position_spread)
{
    Grain &g = grains[channel][current_grain[channel]];
    g.active = true;
    // Compute spread window relative to base delay (allow up to 50% of base when position_spread=1)
    float max_spread = base_delay_samples * (0.05f + position_spread * 0.45f);
    float r = (static_cast<float>(rand()) / RAND_MAX) * 2.f - 1.f; // -1..1
    float offset = r * max_spread;
    float target = base_delay_samples + offset;
    float upper_limit = DSY_CLAMP(base_delay_samples * 2.0f, 2000.f, (float)MAX_DELAY - 8.f);
    g.pos = DSY_CLAMP(target, 480.f, upper_limit);

    // Grain size (shorter at low spread, longer at high)
    float min_g = MIN_GRAIN_SIZE; // 400
    float max_g = MAX_GRAIN_SIZE; // 4800
    g.grain_size = min_g + (max_g - min_g) * position_spread;

    // Slight playback rate jitter for texture
    float rate_jitter = 0.02f * position_spread; // up to ±2%
    float rate_r = (static_cast<float>(rand()) / RAND_MAX) * 2.f - 1.f;
    g.increment = 1.0f + rate_r * rate_jitter;
    g.amp = 1.0f;
    g.crossfade_pos = 0.0f;
    current_grain[channel] = (current_grain[channel] + 1) % MAX_GRAINS;
}

float KaliDSP::ProcessGrain(Grain &g, KaliDelayLine<float, MAX_DELAY> *delay, float base_delay_samples)
{
    if (!g.active)
        return 0.0f;

    // Read from delay line using the grain's current delay time
    float out = delay->ReadHermite(g.pos);

    // Advance position for pitch (increment usually ~1.0)
    g.pos += g.increment;

    // Apply grain envelope (Hanning window)
    g.crossfade_pos += 1.0f / DSY_MAX(1.0f, g.grain_size);
    float window = 0.5f * (1.0f - cosf(2.0f * M_PI * g.crossfade_pos));
    out *= window;

    // Check if grain is finished
    if (g.crossfade_pos >= 1.0f)
    {
        g.active = false;
    }

    return out;
}

// Wave shaping functions implementation
float KaliDSP::foldback(float in, float threshold)
{
    if (in > threshold || in < -threshold)
    {
        return fabs(fabs(fmodf(in - threshold, threshold * 4)) - threshold * 2) - threshold;
    }
    return in;
}

float KaliDSP::quantizer(float in, float A, float B)
{
    int maxres = 4096;
    int choppa = maxres * A;
    return ((in * choppa) / choppa) * B * 2.0f;
}

float KaliDSP::modulo(float x, float A, float B)
{
    float y = tanh(fmod(x * A * 10.f, 1.0f) * B * 2.0f);
    return y;
}

float KaliDSP::diode(float in, float factor)
{
    return in / (factor + abs(in));
}

void KaliDSP::Allpass(float &wetl, float &wetr, float c)
{
    kill_denormal_by_quantization(wetl);
    kill_denormal_by_quantization(wetr);

    oldx[0] = wetl;
    wetl = -c * wetl + oldx[0] + c * oldy[0];
    oldx[1] = wetr;
    wetr = -c * wetr + oldx[1] + c * oldy[1];
    oldy[0] = wetl;
    oldy[1] = wetr;
    wetl += oldx[0];
    wetr += oldx[1];
    wetl *= 0.5f;
    wetr *= 0.5f;
}

void KaliDSP::kill_denormal_by_quantization(float &val)
{
    static const float anti_denormal = 1e-18;
    val += anti_denormal;
    val -= anti_denormal;
}

void KaliDSP::ProcessPhasorPitch(KaliInputState &s)
{
    // Clear output buffers
    for (int i = 0; i < 4; i++)
    {
        wet[i] = 0.0f;
    }

    // Parameters:
    // Meta1 (s.curmet): Pitch/playback rate - 0.5 = normal speed (1.0x), with deadband
    // Meta2 (s.curmet2): Time division/quantization
    // P1 (s.config_new[0]): Modulation frequency/rate (0 = no modulation)
    // P2 (s.config_new[1]): Modulation depth/intensity (0 = no modulation)
    // P3 (s.config_new[2]): Fine pitch adjustment
    // P4 (s.config_new[3]): Crossfade/smoothing amount

    // Map Meta1 to playback rate with proper center and deadband
    float playback_rate = 1.0f;   // Default to normal speed
    const float deadband = 0.05f; // 5% deadband around center

    if (s.curmet < (0.5f - deadband))
    {
        // Below center: map 0.0-0.45 to 0.25x-1.0x
        float normalized = s.curmet / (0.5f - deadband);
        playback_rate = daisysp::fmap(normalized, 0.25f, 1.0f, daisysp::Mapping::EXP);
    }
    else if (s.curmet > (0.5f + deadband))
    {
        // Above center: map 0.55-1.0 to 1.0x-4.0x
        float normalized = (s.curmet - (0.5f + deadband)) / (0.5f - deadband);
        playback_rate = daisysp::fmap(normalized, 1.0f, 4.0f, daisysp::Mapping::EXP);
    }
    // else: within deadband, keep playback_rate = 1.0f

    // Fine pitch adjustment from P3 (ParamSpec is semitones)
    float fine_multiplier = 1.0f;
    {
        // Use real semitone value directly
        float fine_pitch = s.config_new[2];
        fine_multiplier = powf(2.0f, fine_pitch / 12.0f);
    }
    playback_rate *= fine_multiplier;

    // Modulation parameters - allow complete disable
    float mod_freq_ratio = 0.0f;
    float mod_depth = 0.0f;

    {
        float p1v = norm_from_spec(GetParamSpec(0), s.config_new[0]);
        float p2v = norm_from_spec(GetParamSpec(1), s.config_new[1]);
        if (p1v > 0.01f)
            mod_freq_ratio = p1v * 0.1f;
        if (p2v > 0.01f)
            mod_depth = p2v;
    }

    // Time division from Meta2 with deadband
    float time_division = 1.0f; // Default no division
    if (s.curmet2 > 0.05f)      // Only apply time division if Meta2 > 5%
    {
        time_division = daisysp::fmap(s.curmet2, 1.0f, 16.0f, daisysp::Mapping::EXP);
    }

    // Crossfade amount for smoothing
    float crossfade_amount = norm_from_spec(GetParamSpec(3), s.config_new[3]) * 0.1f;

    // Process each channel
    for (int j = 0; j < 4; j++)
    {
        float read_pos = s.delaytimes[j]; // Start with base delay time

        // Only apply DelayPhasor modulation if parameters are non-zero
        if (mod_freq_ratio > 0.0f || mod_depth > 0.0f)
        {
            s.dp[j]->SetDelaySamples(s.delaytimes[j]);
            s.dp[j]->SetModFreqRatio(mod_freq_ratio);
            s.dp[j]->SetModDepth(mod_depth * 0.5f);
            s.dp[j]->SetWritePtr(s.delays[j]->GetWritePtr());
            s.dp[j]->Process();

            // Apply modulation to read position
            read_pos = s.delaytimes[j] - s.dp[j]->GetDelayPosition();
        }

        // Only apply playback rate changes if rate != 1.0
        if (fabsf(playback_rate - 1.0f) > 0.01f)
        {
            // Accumulate rate offset over time for pitch shifting
            static float rate_accumulators[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            rate_accumulators[j] += (playback_rate - 1.0f) * 0.1f;

            // Wrap accumulator to prevent overflow
            if (rate_accumulators[j] > s.delaytimes[j] * 0.5f)
                rate_accumulators[j] -= s.delaytimes[j];
            if (rate_accumulators[j] < -s.delaytimes[j] * 0.5f)
                rate_accumulators[j] += s.delaytimes[j];

            read_pos += rate_accumulators[j];
        }

        // Apply time division/quantization if enabled
        if (time_division > 1.01f)
        {
            float quantized_pos = floorf(read_pos / time_division) * time_division;

            // Crossfade between smooth and quantized positions
            read_pos = read_pos + crossfade_amount * (quantized_pos - read_pos);
        }

        // Ensure position stays within valid range
        read_pos = DSY_CLAMP(read_pos, 4.0f, s.MAX_DELAY_WORKING - 4.0f);

        // Detect loop seam jump and do a tiny crossfade instead of smoothing
        float prev = zerocool[j];
        float delta = fabsf(read_pos - prev);
        float jump_thresh = DSY_MAX(64.0f, 0.45f * s.delaytimes[j]);
        bool is_jump = (delta > jump_thresh);

        // Snap position (no smoothing at seam)
        zerocool[j] = read_pos;
        float new_sample = s.delays[j]->ReadHermite(zerocool[j]);

        if (is_jump)
        {
            loop_xfade_prev_out[j] = last_output_sample[j];
            loop_xfade_remaining[j] = loop_xfade_len;
        }

        if (loop_xfade_remaining[j] > 0)
        {
            float t = 1.0f - (loop_xfade_remaining[j] / (float)loop_xfade_len);
            wet[j] = loop_xfade_prev_out[j] * (1.0f - t) + new_sample * t;
            loop_xfade_remaining[j]--;
        }
        else
        {
            wet[j] = new_sample;
        }

        // Apply gentle level compensation only when pitch shifting
        if (fabsf(playback_rate - 1.0f) > 0.01f)
        {
            wet[j] *= 0.95f; // Slight level compensation
        }
    }
}

void KaliDSP::ProcessPhasorPitchOctave(KaliInputState &s)
{
    // Same as ProcessPhasorPitch but with 12x wider pitch range on Meta1
    for (int i = 0; i < 4; i++)
    {
        wet[i] = 0.0f;
    }

    // Map Meta1 linearly to ±12 semitones (full octave) with no deadband
    float pitch_semitones = (s.curmet - 0.5f) * 24.0f;
    float playback_rate = powf(2.0f, pitch_semitones / 12.0f);

    // Fine pitch from P3 (ParamSpec provides real semitone value, typically ±2)
    playback_rate *= powf(2.0f, s.config_new[2] / 12.0f);

    // Modulation parameters
    float mod_freq_ratio = 0.0f;
    float mod_depth = 0.0f;
    {
        float p1v = norm_from_spec(GetParamSpec(0), s.config_new[0]);
        float p2v = norm_from_spec(GetParamSpec(1), s.config_new[1]);
        mod_freq_ratio = (p1v > 0.01f) ? p1v * 0.1f : 0.0f;
        mod_depth = (p2v > 0.01f) ? p2v : 0.0f;
    }

    // Time division from Meta2
    float time_division = (s.curmet2 > 0.05f) ? daisysp::fmap(s.curmet2, 1.0f, 16.0f, daisysp::Mapping::EXP) : 1.0f;

    float crossfade_amount = norm_from_spec(GetParamSpec(3), s.config_new[3]) * 0.1f;

    // Process each channel (same logic as original)
    for (int j = 0; j < 4; j++)
    {
        float read_pos = s.delaytimes[j];

        if (mod_freq_ratio > 0.0f || mod_depth > 0.0f)
        {
            s.dp[j]->SetDelaySamples(s.delaytimes[j]);
            s.dp[j]->SetModFreqRatio(mod_freq_ratio);
            s.dp[j]->SetModDepth(mod_depth * 0.5f);
            s.dp[j]->SetWritePtr(s.delays[j]->GetWritePtr());
            s.dp[j]->Process();
            read_pos = s.delaytimes[j] - s.dp[j]->GetDelayPosition();
        }

        if (fabsf(playback_rate - 1.0f) > 0.0001f)
        {
            static float rate_accumulators[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            // Accumulate a position offset proportional to playback rate difference
            // Scale by base delay so an octave shift is clearly audible across ranges
            // Limit scaling on large delay ranges to avoid runaway offsets
            float base_scale = s.delaytimes[j] * 0.002f;       // 0.2% of base
            base_scale = DSY_CLAMP(base_scale, 12.0f, 512.0f); // 0.25..10.7 ms at 48k
            rate_accumulators[j] += (playback_rate - 1.0f) * base_scale;

            if (rate_accumulators[j] > s.delaytimes[j])
                rate_accumulators[j] -= s.delaytimes[j];
            if (rate_accumulators[j] < -s.delaytimes[j])
                rate_accumulators[j] += s.delaytimes[j];

            read_pos += rate_accumulators[j];
        }

        if (time_division > 1.01f)
        {
            float quantized_pos = floorf(read_pos / time_division) * time_division;
            read_pos = read_pos + crossfade_amount * (quantized_pos - read_pos);
        }

        read_pos = DSY_CLAMP(read_pos, 4.0f, s.MAX_DELAY_WORKING - 4.0f);

        // Seam handling: no smoothing, use tiny crossfade on wrap
        float prev = zerocool[j];
        float delta = fabsf(read_pos - prev);
        float jump_thresh = DSY_MAX(64.0f, 0.45f * s.delaytimes[j]);
        bool is_jump = (delta > jump_thresh);

        zerocool[j] = read_pos;
        float new_sample = s.delays[j]->ReadHermite(zerocool[j]);

        if (is_jump)
        {
            loop_xfade_prev_out[j] = last_output_sample[j];
            loop_xfade_remaining[j] = loop_xfade_len;
        }

        if (loop_xfade_remaining[j] > 0)
        {
            float t = 1.0f - (loop_xfade_remaining[j] / (float)loop_xfade_len);
            wet[j] = loop_xfade_prev_out[j] * (1.0f - t) + new_sample * t;
            loop_xfade_remaining[j]--;
        }
        else
        {
            wet[j] = new_sample;
        }

        if (fabsf(playback_rate - 1.0f) > 0.01f)
        {
            wet[j] *= 0.95f;
        }
    }
}

void KaliDSP::ProcessGranularTexture(KaliInputState &s)
{
    // Preset: Slow modulation, medium grain size, subtle effects
    // Meta1 = pitch (±1 semitone range), Meta2 = texture density

    for (int i = 0; i < 4; i++)
    {
        wet[i] = 0.0f;
    }

    // Subtle pitch range like original granular
    float playback_rate = 1.0f;
    const float deadband = 0.05f;

    if (s.curmet < (0.5f - deadband))
    {
        float normalized = s.curmet / (0.5f - deadband);
        playback_rate = daisysp::fmap(normalized, 0.94f, 1.0f, daisysp::Mapping::EXP); // ~1 semitone down
    }
    else if (s.curmet > (0.5f + deadband))
    {
        float normalized = (s.curmet - (0.5f + deadband)) / (0.5f - deadband);
        playback_rate = daisysp::fmap(normalized, 1.0f, 1.06f, daisysp::Mapping::EXP); // ~1 semitone up
    }

    // Preset modulation: slow and gentle
    float mod_freq_ratio = 0.005f + norm_from_spec(GetParamSpec(0), s.config_new[0]) * 0.02f; // 0.5%..2.5%
    float mod_depth = 0.1f + norm_from_spec(GetParamSpec(1), s.config_new[1]) * 0.3f;         // 10%..40%

    // Meta2 controls texture density (time division)
    float time_division = (s.curmet2 > 0.05f) ? daisysp::fmap(s.curmet2, 1.0f, 8.0f, daisysp::Mapping::EXP) : 1.0f;

    // Smooth crossfading for textures
    float crossfade_amount = 0.05f + norm_from_spec(GetParamSpec(3), s.config_new[3]) * 0.15f;

    for (int j = 0; j < 4; j++)
    {
        float read_pos = s.delaytimes[j];

        // Always apply gentle modulation for texture
        s.dp[j]->SetDelaySamples(s.delaytimes[j]);
        s.dp[j]->SetModFreqRatio(mod_freq_ratio);
        s.dp[j]->SetModDepth(mod_depth * 0.5f);
        s.dp[j]->SetWritePtr(s.delays[j]->GetWritePtr());
        s.dp[j]->Process();
        read_pos = s.delaytimes[j] - s.dp[j]->GetDelayPosition();

        // Apply subtle pitch changes
        if (fabsf(playback_rate - 1.0f) > 0.005f)
        {
            static float texture_accumulators[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            texture_accumulators[j] += (playback_rate - 1.0f) * 0.05f; // Slower rate changes

            if (texture_accumulators[j] > s.delaytimes[j] * 0.3f)
                texture_accumulators[j] -= s.delaytimes[j] * 0.6f;
            if (texture_accumulators[j] < -s.delaytimes[j] * 0.3f)
                texture_accumulators[j] += s.delaytimes[j] * 0.6f;

            read_pos += texture_accumulators[j];
        }

        // Apply time division for texture
        if (time_division > 1.01f)
        {
            float quantized_pos = floorf(read_pos / time_division) * time_division;
            read_pos = read_pos + crossfade_amount * (quantized_pos - read_pos);
        }

        read_pos = DSY_CLAMP(read_pos, 4.0f, s.MAX_DELAY_WORKING - 4.0f);

        // Seam handling: detect wrap and crossfade instead of smoothing
        float prev = zerocool[j];
        float delta = fabsf(read_pos - prev);
        float jump_thresh = DSY_MAX(64.0f, 0.45f * s.delaytimes[j]);
        bool is_jump = (delta > jump_thresh);

        zerocool[j] = read_pos;
        float new_sample = s.delays[j]->ReadHermite(zerocool[j]);

        if (is_jump)
        {
            loop_xfade_prev_out[j] = last_output_sample[j];
            loop_xfade_remaining[j] = loop_xfade_len;
        }

        if (loop_xfade_remaining[j] > 0)
        {
            float t = 1.0f - (loop_xfade_remaining[j] / (float)loop_xfade_len);
            wet[j] = loop_xfade_prev_out[j] * (1.0f - t) + new_sample * t;
            loop_xfade_remaining[j]--;
        }
        else
        {
            wet[j] = new_sample;
        }
        wet[j] *= 0.98f; // Very gentle level compensation
    }
}

void KaliDSP::ProcessGranularShimmer(KaliInputState &s)
{
    // Preset: Fast modulation, small grains, pitch up, ethereal sounds
    // Meta1 = shimmer intensity, Meta2 = shimmer rate

    for (int i = 0; i < 4; i++)
    {
        wet[i] = 0.0f;
    }

    // Pitch bias upward for shimmer effect
    float shimmer_intensity = s.curmet;
    float base_pitch = 0.0f + shimmer_intensity * 7.0f; // 0 to +7 semitones bias
    float playback_rate = powf(2.0f, base_pitch / 12.0f);

    // Add shimmer modulation on top (use chorus output directly)
    float shimmer_mod = 0.02f * chorus[0].last_unscaled; // Use the chorus output as modulation source
    playback_rate *= (1.0f + shimmer_mod * 0.01f);       // Scale down the modulation

    // Fast modulation for sparkle
    float mod_freq_ratio = 0.05f + norm_from_spec(GetParamSpec(0), s.config_new[0]) * 0.15f; // 5%..20%
    float mod_depth = 0.3f + norm_from_spec(GetParamSpec(1), s.config_new[1]) * 0.5f;        // 30%..80%

    // Meta2 controls shimmer rate (fast time divisions)
    float time_division = (s.curmet2 > 0.05f) ? daisysp::fmap(s.curmet2, 2.0f, 32.0f, daisysp::Mapping::EXP) : 1.0f;

    // Fast crossfading for sparkles
    float crossfade_amount = 0.1f + norm_from_spec(GetParamSpec(3), s.config_new[3]) * 0.2f;

    for (int j = 0; j < 4; j++)
    {
        float read_pos = s.delaytimes[j];

        // Apply fast modulation
        s.dp[j]->SetDelaySamples(s.delaytimes[j]);
        s.dp[j]->SetModFreqRatio(mod_freq_ratio);
        s.dp[j]->SetModDepth(mod_depth * 0.7f); // Deeper modulation
        s.dp[j]->SetWritePtr(s.delays[j]->GetWritePtr());
        s.dp[j]->Process();
        read_pos = s.delaytimes[j] - s.dp[j]->GetDelayPosition();

        // Apply shimmer pitch changes
        static float shimmer_accumulators[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        shimmer_accumulators[j] += (playback_rate - 1.0f) * 0.08f; // Faster pitch changes

        if (shimmer_accumulators[j] > s.delaytimes[j] * 0.4f)
            shimmer_accumulators[j] -= s.delaytimes[j] * 0.8f;
        if (shimmer_accumulators[j] < -s.delaytimes[j] * 0.4f)
            shimmer_accumulators[j] += s.delaytimes[j] * 0.8f;

        read_pos += shimmer_accumulators[j];

        // Apply fast time division for sparkles
        if (time_division > 1.01f)
        {
            float quantized_pos = floorf(read_pos / time_division) * time_division;
            read_pos = read_pos + crossfade_amount * (quantized_pos - read_pos);
        }

        read_pos = DSY_CLAMP(read_pos, 4.0f, s.MAX_DELAY_WORKING - 4.0f);

        // Seam handling: crossfade on loop instead of smoothing
        float prev = zerocool[j];
        float delta = fabsf(read_pos - prev);
        float jump_thresh = DSY_MAX(64.0f, 0.45f * s.delaytimes[j]);
        bool is_jump = (delta > jump_thresh);

        zerocool[j] = read_pos;
        float new_sample = s.delays[j]->ReadHermite(zerocool[j]);

        if (is_jump)
        {
            loop_xfade_prev_out[j] = last_output_sample[j];
            loop_xfade_remaining[j] = loop_xfade_len;
        }

        if (loop_xfade_remaining[j] > 0)
        {
            float t = 1.0f - (loop_xfade_remaining[j] / (float)loop_xfade_len);
            wet[j] = loop_xfade_prev_out[j] * (1.0f - t) + new_sample * t;
            loop_xfade_remaining[j]--;
        }
        else
        {
            wet[j] = new_sample;
        }

        // Add harmonic sparkle
        wet[j] *= (0.9f + 0.1f * fabsf(shimmer_mod * 0.1f));
    }
}

void KaliDSP::ProcessGranularCrystals(KaliInputState &s)
{
    // Meta1: semitone pitch bias, Meta2: subdivisions per crystal
    // P1: step rate in Hz, P2: crystal size in ms, P3: tone tilt, P4: edge sharpness
    // TODO: pitch bias is good but also wanted an actual granular shift, it may not matter

    for (int i = 0; i < 4; i++)
        wet[i] = 0.0f;

    float step_rate_hz = s.config_new[0];
    if (!(step_rate_hz > 0.0f))
        step_rate_hz = 1.0f;
    step_rate_hz = DSY_CLAMP(step_rate_hz, 1.0f, 128.0f);
    float step_period_samples = DSY_MAX(1.0f, samplerate / step_rate_hz);

    float crystal_size_ms = s.config_new[1];
    if (!(crystal_size_ms > 0.0f))
        crystal_size_ms = 12.0f;
    crystal_size_ms = DSY_CLAMP(crystal_size_ms, 2.0f, 1024.0f);
    float crystal_unit_samples = DSY_MAX(1.0f, crystal_size_ms * 0.001f * samplerate);

    float color = norm_from_spec(GetParamSpec(2), s.config_new[2]);
    float edge = norm_from_spec(GetParamSpec(3), s.config_new[3]);
    float crossfade_amount = 0.01f + (1.0f - edge) * 0.09f;
    crossfade_amount = DSY_CLAMP(crossfade_amount, 0.005f, 0.1f);
    float tone_pole = expf(-2.0f * static_cast<float>(M_PI) * 2000.0f * sample_interval);
    tone_pole = DSY_CLAMP(tone_pole, 0.0f, 0.9999f);

    float pitch_norm = DSY_CLAMP(s.curmet, 0.0f, 1.0f);
    float pitch_semitones = roundf(daisysp::fmap(pitch_norm, -12.0f, 12.0f, daisysp::Mapping::LINEAR));
    float playback_rate = powf(2.0f, pitch_semitones / 12.0f);

    float subdiv_norm = DSY_CLAMP(s.curmet2, 0.0f, 1.0f);
    int desired_subdiv = 1 + static_cast<int>(roundf(subdiv_norm * 7.0f));
    desired_subdiv = DSY_CLAMP(desired_subdiv, 1, 8);

    for (int j = 0; j < 4; j++)
    {
        float base_delay = DSY_CLAMP(s.delaytimes[j], 4.0f, s.MAX_DELAY_WORKING - 4.0f);

        s.dp[j]->SetDelaySamples(base_delay);
        s.dp[j]->SetModFreqRatio(0.0f);
        s.dp[j]->SetModDepth(0.0f);
        s.dp[j]->SetWritePtr(s.delays[j]->GetWritePtr());
        s.dp[j]->Process();
        float base_pos = s.dp[j]->GetDelayPosition();

        float max_quant = DSY_MAX(4.0f, base_delay - 4.0f);
        float quant_unit = DSY_CLAMP(crystal_unit_samples, 4.0f, max_quant);
        if (base_delay <= 8.0f)
            quant_unit = DSY_MAX(4.0f, base_delay * 0.5f);

        crystal_step_timer[j] -= 1.0f;
        if (crystal_initialized[j] && desired_subdiv != crystal_subdivisions_state[j] && crystal_steps_done[j] < crystal_subdivisions_state[j])
            crystal_steps_done[j] = crystal_subdivisions_state[j];

        bool trigger = (crystal_step_timer[j] <= 0.0f) || !crystal_initialized[j];
        if (trigger)
        {
            bool need_new_block = (!crystal_initialized[j]) || (crystal_steps_done[j] >= crystal_subdivisions_state[j]);
            if (need_new_block)
            {
                float quantized = floorf(base_pos / quant_unit) * quant_unit;
                float block_span = DSY_MAX(1.0f, quant_unit);
                float max_base = DSY_MAX(0.0f, base_delay - block_span);
                float block_base = DSY_CLAMP(quantized, 0.0f, max_base);
                crystal_block_base[j] = block_base;
                crystal_block_size[j] = block_span;
                crystal_subdivisions_state[j] = desired_subdiv;
                crystal_steps_done[j] = 0;
                crystal_pitch_state[j] = 0.0f;
            }

            int active_subdiv = DSY_MAX(1, crystal_subdivisions_state[j]);
            float sub_size = DSY_MAX(1.0f, crystal_block_size[j] / static_cast<float>(active_subdiv));
            float sub_center = (static_cast<float>(crystal_steps_done[j]) + 0.5f) * sub_size;
            float block_target = crystal_block_base[j] + sub_center;
            block_target = DSY_CLAMP(block_target, 0.0f, base_delay - 4.0f);
            float target = base_delay - block_target;
            target = DSY_CLAMP(target, 4.0f, s.MAX_DELAY_WORKING - 4.0f);
            crystal_target_pos[j] = target;

            crystal_steps_done[j] += 1;
            crystal_step_timer[j] += DSY_MAX(1.0f, step_period_samples / static_cast<float>(DSY_MAX(1, crystal_subdivisions_state[j])));
            crystal_initialized[j] = true;
        }

        crystal_hold_pos[j] += crossfade_amount * (crystal_target_pos[j] - crystal_hold_pos[j]);
        float base_pos_smoothed = DSY_CLAMP(crystal_hold_pos[j], 4.0f, s.MAX_DELAY_WORKING - 4.0f);

        int active_subdiv = DSY_MAX(1, crystal_subdivisions_state[j]);
        float active_sub_size = DSY_MAX(1.0f, (crystal_block_size[j] > 0.0f) ? (crystal_block_size[j] / static_cast<float>(active_subdiv)) : quant_unit);
        float pitch_delta = playback_rate - 1.0f;
        if (fabsf(pitch_delta) > 1e-5f)
        {
            crystal_pitch_state[j] += pitch_delta;
            float pitch_limit = active_sub_size * 0.5f;
            crystal_pitch_state[j] = DSY_CLAMP(crystal_pitch_state[j], -pitch_limit, pitch_limit);
        }
        else
        {
            crystal_pitch_state[j] = 0.0f;
        }

        float edge_slew_ms = 1.0f + (1.0f - edge) * 6.0f; // fast smoothing by default
        float prev = zerocool[j];
        float smoothed = prev;
        fonepole(smoothed, base_pos_smoothed, mstocoeff(edge_slew_ms));

        float pitched_pos = smoothed - crystal_pitch_state[j];
        pitched_pos = DSY_CLAMP(pitched_pos, 4.0f, s.MAX_DELAY_WORKING - 4.0f);

        float delta = fabsf(pitched_pos - prev);
        float jump_thresh = DSY_MAX(64.0f, 0.45f * s.delaytimes[j]);
        bool is_jump = (delta > jump_thresh);

        zerocool[j] = pitched_pos;
        float sample = s.delays[j]->ReadHermite(pitched_pos);

        fonepole(crystal_lp_state[j], sample, tone_pole);
        float colored = crystal_lp_state[j] + (sample - crystal_lp_state[j]) * color;

        if (is_jump)
        {
            loop_xfade_prev_out[j] = last_output_sample[j];
            loop_xfade_remaining[j] = loop_xfade_len;
        }

        if (loop_xfade_remaining[j] > 0)
        {
            float t = 1.0f - (loop_xfade_remaining[j] / (float)loop_xfade_len);
            wet[j] = loop_xfade_prev_out[j] * (1.0f - t) + colored * t;
            loop_xfade_remaining[j]--;
        }
        else
        {
            wet[j] = colored;
        }
    }
}
