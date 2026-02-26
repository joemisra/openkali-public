#include "daisysp.h"
#include "dpt/daisy_dpt.h"
#include "KaliDelayLine.h"
#include "ParameterInterpolator.h"
#include "dev/oled_ssd130x.h"
#include "util/oled_fonts.h"
#include "util/hal_map.h"
#include "dev/leddriver.h"
#include "sys/system.h"
#include "Kali.h"
#include "KaliFIR.h"
#include "stringtables.h"
#include "PresetNameGenerator.h"
#include "stdio.h"
#include <memory>
#include <algorithm>
#include <unordered_map>

// VERSION is defined by Makefile from git tag
// Commented string arrays moved to stringtables.cpp to save flash
static KaliDelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delayl;
static KaliDelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delayr;
static KaliDelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delayx;
static KaliDelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delayy;

// extern "C" void initialise_monitor_handles(void);

using namespace daisy;
using namespace daisysp;
using namespace dpt;

const float cvfraction = 1.f / 4096.f;

#define MIDI_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP 0xFC
// #define ENABLE_UNLINKED 0

using MyOledDisplay = OledDisplay<SSD130xI2c128x32Driver>;
static MyOledDisplay display;

static Kali kali;

// static FIR<FIRFILTER_USER_MEMORY> flt[4];

static OptimizedFIR flt[4];

static constexpr size_t flt_size = 17;    /*< FIR filter length */
static constexpr int upd_rate_hz = 48000; /*< FIR recalculation rate */

// static float ir_front[4][flt_size] = {0};    /*< Active FIR coefficients */
// static float ir_back[4][flt_size + 1] = {0}; /*< Updating FIR coefficients */
// static float wnd[flt_size / 2] = {0};        /*< Windowing function */
// static bool ir_update_pending = false;       /*< ir_back update ready */
// static float flt_state[4][flt_size + 1];     /*< Impl-specific storage */
// static int updatecount = 0;

#ifndef PI
#define PI 3.14159265358979323846264338328
#endif

#if ENABLE_WAVETABLE_EDITOR
#include "WavetableStepEditor.h"
#endif

bool oledyes = false;
int oledtick = 0;
int oledtick_max = 4;
// OLED contrast tracking (needs to be visible before updateoled())
static int mincontrast = 0x20;
static int lastcontrast = 0xFF;

// forward declaration
void handlar(void *data);

void Kali::Init(float samplerate)
{
    // Initialize string tables in SDRAM
    InitStringTables();

    // Initialize Font_10x10 from Font_5x5 (saves ~1.9KB flash)
    InitFont10x10();

#if ENABLE_REVERB_RETURN
    reverb.Init(samplerate);
#endif
    patch.ProcessAllControls();

    for (int i = 0; i < 4; i++)
    {
        acidburn[i].Init(samplerate);
    }

    // check if both buttons are held down, FIXME: currently unused
    // bool reset_options = patch.gate_in_1.State() && patch.gate_in_2.State();

    // OPTIONS MANAGER TODO: options.Init(patch.qspi, reset_options);

    delays[0] = &delayl;
    delays[1] = &delayr;
    delays[2] = &delayx;
    delays[3] = &delayy;

    // Global envelope follower
    Follower.Setup(samplerate, 50, 250);
#if ENABLE_WAVETABLE_EDITOR
    if (!wavetable_editor_ptr)
    {
        // Lazy init; editor owns no memory, it writes into knob0 table
        wavetable_editor_ptr = new WavetableStepEditor();
        wavetable_editor_ptr->Init(inp.Knobs[0].wavetable.wavetable);
    }
#endif

    InitStateMap();

    float warbsr = (samplerate / 96) * 64;

    for (int j = 0; j < OptionsPages::KALI_OPTIONS_LAST; j++)
    {
        OptionRules[BankType::Global][j]->Reset();
    }

    for (int i = 0; i < 9; i++)
    {
        warble[i].Init(warbsr);

        /*
        for(int j=0; j < LFOOptionsPages::KALI_LFO_OPTIONS_LAST; j++) {
            warble[i].preset.SetOption(j, OptionRules[BankType::LFO][j]->DefaultValue);
        }
        */

        warble[i].SetFreq(20 * (i + 1)); // not in options array
        warble[i].SetAmp(4096.);         // not in options array
        warble[i].SetMode(0);
        warble[i].SetWaveform(Oscillator::WAVE_SIN);
        warble[i].index = i;

        warble[i].offset = 0;
        warble[i].attenuate = 100.;
        warble[i].offset_cal = 0;
        warble[i].attenuate_cal = 100.;

        warble[i].Follower = &Follower;
    }

    for (int i = 6; i < 8; i++)
    {
        warble[i].SetWaveform(daisysp::Oscillator::WAVE_TRI);
        warble[i].SetMode(Kali::LFOModes::SyncStraight);
        warble[i].SetAmp(1900.);
        warble[i].index = 0;
        warble[i].meta = 4;
        warble[i].bipolar = true;
    }

    warble[8].SetWaveform(daisysp::Oscillator::WAVE_SIN);
    warble[8].SetAmp(1.);
    warble[8].SetFreq(0.001);

    // Clock oscillators removed - using PLL approach instead

    // stereo related stuff

    for (int i = 0; i < 4; i++)
    {
        delays[i]->Init();
        delays[i]->SetDelay(48000.f); // 1 second

        dc[i].Init(samplerate);
    }

    // Make Meta1/Meta2 analog smoothing fast
    patch.controls[0].SetCoeff(0.8f); // META1 faster response
    patch.controls[1].SetCoeff(0.8f); // META2 already fast
    patch.controls[3].SetCoeff(0.8f);
    patch.controls[4].SetCoeff(0.8f);
    patch.controls[6].SetCoeff(0.01f);

    dsp.Init(samplerate);

    cf.Init(CROSSFADE_CPOW);

    for (int i = 0; i < 4; i++)
    {
        flt[i].Init();
    }

    // Make Meta1/Meta2 interpolation respond faster
    // Reduce internal interpolation window (size) so Next() converges quicker
    inp.Knobs[Kali::CV::META1].size = 24; // was 96
    inp.Knobs[Kali::CV::META1].slew = 1;
    inp.Knobs[Kali::CV::META1].sizetimesslewrecip = 1.0f / (float)(inp.Knobs[Kali::CV::META1].size * inp.Knobs[Kali::CV::META1].slew);
    inp.Knobs[Kali::CV::META2].size = 24; // was 96
    inp.Knobs[Kali::CV::META2].slew = 1;
    inp.Knobs[Kali::CV::META2].sizetimesslewrecip = 1.0f / (float)(inp.Knobs[Kali::CV::META2].size * inp.Knobs[Kali::CV::META2].slew);

    // Initialize state
    currentState = &KaliConfigOscState::getInstance();
    editstate.major_mode = KaliEditState::LFO_EDIT;

    // Load last selected preset slot
    editstate.SelectedPresetIndex = LoadLastSelectedPresetSlot();

    // Initialize master clock with loaded options
    masterclock.midi_ppqn = 24;    // Always 24 for MIDI
    masterclock.internal_ppqn = 4; // Default internal PPQN
    masterclock.external_ppqn = GetValue(OptionsPages::ExternalCvClockPPQN);
    masterclock.Mode = (KaliClock::KaliClockMode)GetValue(OptionsPages::SyncEngine);

    // Initialize with proper mode and timing settings
    masterclock.Init(samplerate, size, masterclock.internal_ppqn, 144, masterclock.Mode);

    // TODO: Precompute a range for UpdateFilter?
    // InitWindow();

    // UpdateFilter(0.8f);
}

void Kali::HandleMIDI()
{
    // Poll hardware
    patch.midi.Listen();
    midi_activity = patch.midi.HasEvents();

    // Coalesce CC floods this cycle
    static uint8_t last_cc_value[16][128];
    static bool cc_seen[16][128];
    for (int ch = 0; ch < 16; ++ch)
        for (int c = 0; c < 128; ++c)
            cc_seen[ch][c] = false;

    const int maxDrain = 128; // drain up to 128 MIDI events per call
    int drained = 0;
    while (patch.midi.HasEvents() && drained < maxDrain)
    {
        auto event = patch.midi.PopEvent();
        drained++;
        AddMidiEvent(event);

        if (event.type == MidiMessageType::SystemRealTime)
        {
            switch (event.srt_type)
            {
            case SystemRealTimeType::TimingClock:
                patch.MIDISendClock();
                kali.TriggerReceived = true;
                midi_clock_flag = true;
                break;
            case SystemRealTimeType::Continue:
                patch.MIDISendClockContinue();
                ResetClocks();
                kali.TriggerReceived = true;
                break;
            case SystemRealTimeType::Start:
                patch.MIDISendClockStart();
                ResetAllThings();
                gate_count = 0;
                kali.TriggerReceived = true;
                break;
            case SystemRealTimeType::Stop:
                patch.MIDISendClockStop();
                ResetAllThings();
                gate_count = 0;
                kali.TriggerReceived = false;
                break;
            default:
                break;
            }
        }
        else if (event.type == MidiMessageType::ControlChange)
        {
            auto cc = event.AsControlChange();
            int ch = cc.channel & 0x0F;
            if (cc.control_number < 128 && ch >= 0 && ch < 16)
            {
                last_cc_value[ch][cc.control_number] = cc.value;
                cc_seen[ch][cc.control_number] = true;
            }
#if ENABLE_WAVETABLE_EDITOR
            const int startcc = 64;
            if (cc.control_number >= startcc && cc.control_number <= startcc + 16)
            {
                int step = cc.control_number - startcc;
                if (step >= 0 && step < 16)
                    wavetable_editor_ptr->ProcessMidiCC(step, cc.value);
            }
#endif
        }
        else if (event.type == MidiMessageType::NoteOn)
        {
            auto note = event.AsNoteOn();
            midi.receiveNoteOn(note);
        }
        else if (event.type == MidiMessageType::NoteOff)
        {
            auto note = event.AsNoteOff();
            midi.receiveNoteOff(note);
        }
    }

    // Apply coalesced CC updates
    for (int ch = 0; ch < 16; ++ch)
    {
        for (int c = 0; c < 128; ++c)
        {
            if (cc_seen[ch][c])
            {
                ControlChangeEvent e;
                e.channel = ch;
                e.control_number = (uint8_t)c;
                e.value = last_cc_value[ch][c];
                midi.receiveControlChange(e, this);
            }
        }
    }
}

void Kali::HitTick(size_t blocksize)
{
    tick = (tick + 1) % blocksize * 96;
}

float Kali::GateSpaceTime(size_t blocksize)
{
    if (last_gate < tick)
    {
        return tick - last_gate;
    }
    else
    {
        return tick + (blocksize * 96) - last_gate;
    }
}

/// @brief This is called from within AudioCallback, it handles a countdown which we use to detect when there is no longer sync present.
void Kali::HandleCallbackSync()
{
    // NOTE: I think we have moved this over into masterclock
    //
    // Decrement gate count used for clock detection
    // gate_count -= size * 1;
    // if (gate_count < 0)
    // {
    //     gate_count = 0;
    //     has_clock = KaliClock::KaliClockMode::None;
    // }
}

void Kali::HandleSyncTrigger()
{
    // If sync button mode is set to trigger (0), handle as clock signal
    // If set to reset (1), reset LFO phases
    if (!CheckFlag(OptionsPages::SyncButtonMode))
    {
        // Set higher gate count to maintain clock detection
        // gate_count += size * 2048;
        // if (gate_count >= (size * 4096))
        // {
        //     gate_count = (size * 2048);
        //     last_gate = 0;
        // }
        TriggerReceived = true;
    }
    else
    {
        ResetAllThings();
    }
}

void Kali::HandleMIDIClock()
{
    // Make MIDI clock handling consistent with other clock types
    midi_clock_flag = true;
    masterclock.SetMode(KaliClock::KaliClockMode::MidiClock);
    TriggerReceived = true;
    // Reset gate_count correctly
    gate_count = 0;
}

void kill_denormal_by_quantization(float &val)
{
    static const float anti_denormal = 1e-18;
    val += anti_denormal;
    val -= anti_denormal;
}

/**
 * Processes the controls for KaliInput.
 *
 * @param patch The DPT object to process controls for.
 */
void KaliInput::ProcessControls(DPT *patch)
{
    patch->ProcessAnalogControls();
    patch->ProcessDigitalControls();

    patch->e1.Debounce();
    patch->e2.Debounce();

    for (int i = 0; i < 8; i++)
    {
        // get actual current knob position
        // abs that sucker unless it's lfo rate
        // float current_value = (i != 2) ? abs(patch->controls[i].Value()) : patch->controls[i].Value();
        float current_value = abs((int)(patch->controls[i].Value() * 1000.f) * 0.001f); // truncate

        kill_denormal_by_quantization(current_value);

        Knobs[i].UpdateValue(current_value);
        // kali.KnobState[i].UpdateValue(current_value);
    }

    /*
    TODO: FM internal
    for(int i=0;i<6;i++) {
        int target = kali.warble[i].preset.GetOption(LFOOptionsPages::FMTarget);
        float amt = kali.warble[i].last_unscaled * abs(kali.warble[i].preset.GetOption(LFOOptionsPages::FMTargetAmount) * 0.01f);
        if(amt > 0.0f)
            Knobs[target]._Value += amt;
    }
    */

    Gate[0] = patch->gate_in_1.State();
    Gate[1] = patch->gate_in_2.State();

    LFOAdjust = patch->GetAdcValue(ADC_9);
    Knobs[8].Hysteresis = 0.01f;
    Knobs[8].UpdateValue(LFOAdjust);
    // kali.KnobState[8].UpdateValue(LFOAdjust);
    DelayAdjust = patch->GetAdcValue(ADC_10);
    Knobs[9].Hysteresis = 0.01f;
    Knobs[9].UpdateValue(DelayAdjust);
    // kali.KnobState[9].UpdateValue(DelayAdjust);
    patch->controls[0].GetRawFloat();

    Meta = fmap(patch->controls[CV_1].Value(), 0.0f, 1.0f, Mapping::LINEAR);
    MetaMapped = abs(Meta);
    Meta2 = fmap(patch->controls[CV_2].Value(), 0.0f, 1.0f, Mapping::EXP);
    Meta2Mapped = abs(Meta2);

    LFORate = Knobs[2].Value();
    // fonepole(LFORateMapped, abs(LFORate), 0.1f);
    LFORateMapped = abs(LFORate);
    TimeL = Knobs[3].Value();
    TimeLMapped = abs(TimeL);
    TimeR = Knobs[4].Value();
    TimeRMapped = abs(TimeR);
    Cutoff = Knobs[5].Value();
    CutoffMapped = abs(Cutoff);
    Feedback = Knobs[6].Value();
    FeedbackMapped = abs(Feedback);

    // Feedback = FeedbackMapped = 0.9f;
    Mix = Knobs[7].Value();
    MixMapped = Knobs[7].Value();
}

// FIXME: Why are these global?

float newboop, newkoko;

float lopspeed = 1. / (0.001f * 192000.f);

/**
 * @brief This fun is the callback for audio processing,
 * also reads controls and updates a lot of things related to those..
 *
 * @param in The input buffer for audio data.
 * @param out The output buffer for processed audio data.
 * @param size The size of the audio buffers.
 */
void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size)
{

    // all of this KaliInputState stuff feels weird
    KaliInputState s;
    s.k = &kali;

    float frequencybase[4], samps[4];

    float dry[4]{0.0f, 0.0f, 0.0f, 0.0f};
    float wet[4]{0.0f, 0.0f, 0.0f, 0.0f};
    float filtered[4]{0.0f, 0.0f, 0.0f, 0.0f};
    float fbbuffer[4][size * 2];

    float clockL = 1.0f, clockR = 1.0f;

    // Get delay range preset and set working min/max accordingly
    int rangePreset = (int)kali.GetValue(DSPOptionsPages::DelayRangePreset, BankType::DSP);

    // Set delay range based on preset
    switch (rangePreset)
    {
    case RANGE_PRECISION:                  // 1-500ms - Fine detail work
        kali.MIN_DELAY_WORKING = 48.0f;    // 1ms
        kali.MAX_DELAY_WORKING = 24000.0f; // 500ms
        break;
    case RANGE_STUDIO:                     // 10ms-2s - Standard studio delays
        kali.MIN_DELAY_WORKING = 480.0f;   // 10ms
        kali.MAX_DELAY_WORKING = 96000.0f; // 2s
        break;
    case RANGE_AMBIENT:                     // 50ms-8s - Ambient textures
        kali.MIN_DELAY_WORKING = 2400.0f;   // 50ms
        kali.MAX_DELAY_WORKING = 384000.0f; // 8s
        break;
    case RANGE_LOOPER:                      // 100ms-20s - Looping and long delays
        kali.MIN_DELAY_WORKING = 4800.0f;   // 100ms
        kali.MAX_DELAY_WORKING = 960000.0f; // 20s
        break;
    case RANGE_EXPERIMENTAL: // 1ms-30s - Full range madness
    default:
        kali.MIN_DELAY_WORKING = 48.0f;      // 1ms
        kali.MAX_DELAY_WORKING = 1440000.0f; // 30s
        break;
    }

    // Clamp to hardware limits
    kali.MIN_DELAY_WORKING = DSY_CLAMP(kali.MIN_DELAY_WORKING, 12.0f, MAX_DELAY);
    kali.MAX_DELAY_WORKING = DSY_CLAMP(kali.MAX_DELAY_WORKING, kali.MIN_DELAY_WORKING, MAX_DELAY);

    // Update delay range for external sync based on current tempo
    kali.UpdateDelayRangeForExternalSync();

    KaliInput *inp = &kali.inp;

    kali.size = size;

    inp->ProcessControls(&kali.patch);

    float send_level = abs(inp->Mix);
    inp->MixMapped = send_level;

    float in_level = inp->MetaMapped;
    float meta = abs(fmap(in_level, 0.f, 1.f));

    // Comment out encoder events for LFO/Delay adjust since they now control P1/P2
    // if (inp->Knobs[CV::LFO_ADJUST].Changed)
    // {
    //     handle_event(KaliEvent::LFOADJTURN);
    // }
    // if (inp->Knobs[CV::DELAY_ADJUST].Changed)
    // {
    //     handle_event(KaliEvent::DELAYADJTURN);
    // }

    if (inp->Knobs[Kali::CV::LFO_ADJUST].Changed)
    {
        kali.warble[0].SetLFOAdjust(inp->Knobs[Kali::CV::LFO_ADJUST].Value());
    }

    if (inp->Knobs[Kali::CV::DELAY_ADJUST].Changed)
    {
        // kali.warble[1].SetLFOAdjust(inp->Knobs[9].Value());
        // li::banktype::] dspoptionspages::distortionamount, 0, inp->knobs[9].value(), kali::banktype::dsp);
        // KaliOption *o = OptionRules[Kali::BankType::DSP][kali.editstate.SelectedOptionIndex];
        // kali.SetOptionValue(kali.editstate.SelectedOptionIndex, inp->Knobs[9].Value() * o->Max, Kali::BankType::DSP);
        kali.SetOptionValue(DSPOptionsPages::DistortionAmount, inp->Knobs[9].Value(), BankType::DSP);
    }

    if (ENABLE_FIR_FILTER)
    {
        float cutoff = fmap(inp->Cutoff, 20.0f, 48000.f * 0.45f, Mapping::LINEAR);
        for (int j = 0; j < 4; j++)
        {
            flt[j].UpdateFilter(cutoff, 48000.f);
        }
    }

    float feedback;

    // Feedback can go out of control in the granular modes so we lower the max.
    feedback = fmap(inp->Feedback, 0.0f, 1.01f);

    float lfospeedstep = 12.f;

    float modmult_knob = inp->LFORateMapped; // + 1.0f) / 2;

    float modmultnew = (int)fmap(inp->LFORateMapped, 0.0f, lfospeedstep, Mapping::LOG);

    float splatyea = fmap(meta, 0.0f, 3.0f);

    ParameterInterpolator splatslide(&kali.splat, splatyea, size * 8);

    for (int i = 0; i < 6; i++)
    {
        kali.warble[i].global_lfo_rate = ceil(modmult_knob * kali.GetValue(OptionsPages::LfoRateMultiplier));
        // kali.warble[i].preset.SetOption(LFOOptionsPages::Meta, kali.new_normal);
        //  kali.warble[i].flip = inp->Knobs[Kali::CV::LFO_RATE].Value() < 0.f; // thru zero
    }

    // kali.masterclock.midi_ppqn = kali.GetValue(OptionsPages::LeftClockRateMultiplier);
    // kali.masterclock.internal_ppqn = 4;
    kali.masterclock.internal_ppqn = 4;
    kali.masterclock.external_ppqn = kali.GetValue(OptionsPages::ExternalCvClockPPQN);
    kali.masterclock.Mode = (KaliClock::KaliClockMode)kali.GetValue(OptionsPages::SyncEngine); // actually sync mode, TODO: rename

    // PLL clock outputs - calculate phase-locked timing
    // No need to set frequencies - we'll calculate pulses directly from master timing

    bool trigger_event = false;

    switch (kali.masterclock.Mode)
    {
    case KaliClock::KaliClockMode::ClockIn:
        // External CV clock handling
        if (inp->Gate[0] && !kali.prevgate1)
        {
            trigger_event = true;
        }
        break;

    case KaliClock::KaliClockMode::MidiClock:
        // MIDI clock handling
        if (kali.midi_clock_flag)
        {
            kali.midi_clock_flag = false;
            trigger_event = true;
        }
        break;

    case KaliClock::KaliClockMode::Internal:
    default:
        // Internal clock handling - no external triggers needed
        break;
    }

    /* Internal Sync */

    kali.HitTick(size);

    // Gate 2 supports two modes:
    // Freeze mode (option = 0): hold gate to freeze delay write/mix behavior.
    // Reset mode  (option = 1): rising edge performs reset only.
    bool freeze_gate = inp->Gate[1];
    bool freeze_mode = !kali.CheckFlag(OptionsPages::FreezeButtonMode);
    bool freeze_rising_edge = freeze_gate && !kali.prevgate2;

    if (freeze_mode)
    {
        kali.isfrozen = freeze_gate;
    }
    else
    {
        if (freeze_rising_edge)
        {
            kali.ResetAllThings();
        }
        kali.isfrozen = false;
    }

    kali.HandleEncoders(inp);

    kali.prevgate1 = inp->Gate[0];
    kali.prevgate2 = inp->Gate[1];

    // Replace lines ~625-659 with this unified solution:

    // Calculate delay times based on clock mode
    // Since range is now dynamically set for external sync, both modes can use simple linear mapping
    if (kali.masterclock.Mode == KaliClock::KaliClockMode::Internal)
    {
        // Internal mode: use direct range mapping for precise control
        newboop = fmap(inp->Knobs[Kali::CV::L_TIME].Value(),
                       kali.MIN_DELAY_WORKING,
                       kali.MAX_DELAY_WORKING,
                       Mapping::LINEAR);
        kali.masterclock.SetSamples(newboop);
    }
    else
    {
        // External clock modes: range is already set to musical divisions, so use linear mapping
        newboop = fmap(inp->Knobs[Kali::CV::L_TIME].Value(),
                       kali.MIN_DELAY_WORKING,
                       kali.MAX_DELAY_WORKING,
                       Mapping::LINEAR);
    }

    // Handle linked/unlinked modes for right channel
    if (!kali.IsUnlinked())
    {
        // In linked mode, right time is a multiplier of left time
        // We want the knob to cover a useful range while staying within bounds
        float knobValue = inp->Knobs[Kali::CV::R_TIME].Value();

        // Calculate the maximum and minimum possible ratios that keep us in bounds
        float maxPossibleRatio = kali.MAX_DELAY_WORKING / newboop;
        float minPossibleRatio = kali.MIN_DELAY_WORKING / newboop;

        // Clamp these to reasonable musical ratios (0.5x to 1.5x)
        float actualMaxRatio = DSY_MIN(maxPossibleRatio, 1.5f);
        float actualMinRatio = DSY_MAX(minPossibleRatio, 0.5f);

        // Map the knob to this constrained ratio range
        float ratio = fmap(knobValue, actualMinRatio, actualMaxRatio, Mapping::LINEAR);
        newkoko = newboop * ratio;
    }
    else
    {
        // In unlinked mode, right time is independent (same logic as left)
        newkoko = fmap(inp->Knobs[Kali::CV::R_TIME].Value(),
                       kali.MIN_DELAY_WORKING,
                       kali.MAX_DELAY_WORKING,
                       Mapping::LINEAR);
    }

    // Apply range constraints
    newboop = DSY_CLAMP(newboop, kali.MIN_DELAY_WORKING, kali.MAX_DELAY_WORKING);
    newkoko = DSY_CLAMP(newkoko, kali.MIN_DELAY_WORKING, kali.MAX_DELAY_WORKING);

    // these hold the caluclated delay time w/ just straight Time L and Time R (no slewing or multipliers etc)
    kali.delaytargets[0] = newboop;
    kali.delaytargets[1] = newkoko;
    kali.delaytargets[2] = newboop;
    kali.delaytargets[3] = newkoko;

    ParameterInterpolator ampslide(&kali.warbamp, inp->Knobs[Kali::CV::META2].Value(), size * 2);
    ParameterInterpolator modmultslide(&kali.modmult, modmultnew, size * 2);

    // TODO: This is a sort of jitter effect, may need updates
    // if (kali.warble[6].mt.Process(0.05))
    //{
    // for (int i = 6; i < 8; i++)
    //    kali.warble[i].DoRand(0.01f, 2.0f, 0.01f, 2048.f);

    // kali.pdsp.drama.SetWaveform((int)kali.patch.GetRandomFloat(0.0, 5.0));
    // kali.pdsp.drama.SetFreq(kali.warble[6].frequency);
    // kali.pdsp.drama.last32possteppedmax = 384;
    //`}

    // ParameterInterpolator warbslide(&kali.warbfreq, newwarbfreq, size * 4);

    // FIXME: Could definitely be improved, also probably move this.
    auto mode = (DSPModes)(kali.GetValue(DSPOptionsPages::Mode, BankType::DSP));

    // Map UI DSP mode to internal DSP engine mode
    auto mapUiToDsp = [](DSPModes ui) -> unsigned int
    {
        switch (ui)
        {
        case DSPModes::StraightLinked:
            return KaliDSP::DSPMode::Basic;
        case DSPModes::PingPongLinked:
            return KaliDSP::DSPMode::PingPong;
        case DSPModes::StraightUnlinked:
            return KaliDSP::DSPMode::Unlinked;
        case DSPModes::Reverse:
            return KaliDSP::DSPMode::Basic;
        case DSPModes::Resonate:
            return KaliDSP::DSPMode::Resonator;
        case DSPModes::Chorus:
            return KaliDSP::DSPMode::Chorus;
        case DSPModes::Knuth:
            return KaliDSP::DSPMode::Knuth;
        case DSPModes::Granular:
            return KaliDSP::DSPMode::Granular;
        case DSPModes::GranularOctave:
            return KaliDSP::DSPMode::GranularOctave;
        case DSPModes::GranularTexture:
            return KaliDSP::DSPMode::GranularTexture;
        case DSPModes::GranularShimmer:
            return KaliDSP::DSPMode::GranularShimmer;
        case DSPModes::GranularCrystals:
            return KaliDSP::DSPMode::GranularCrystals;
#if ENABLE_FFT_BLUR
        case DSPModes::SpectralBlur:
            return KaliDSP::DSPMode::SpectralBlur;
#endif
        case DSPModes::Fluid:
            return KaliDSP::DSPMode::Fluid;
        /*case DSPModes::ShimmerGrain:      return KaliDSP::DSPMode::ShimmerGrain;
        case DSPModes::SimpleChorus:      return KaliDSP::DSPMode::SimpleChorus;
        case DSPModes::Saturate:          return KaliDSP::DSPMode::Saturator;
        case DSPModes::Quantize:          return KaliDSP::DSPMode::Quantizer;
        case DSPModes::OnlyReverb:        return KaliDSP::DSPMode::OnlyReverb;*/
        default:
            return KaliDSP::DSPMode::Basic;
        }
    };

    // Store UI mode for UI logic, set mapped mode for DSP engine
    kali.mode = mode;
    kali.dsp.SetMode(mapUiToDsp(mode));

    // Map P1–P4 UI values (0..100) to per-mode real units using ParamSpec
    for (int p = 0; p < 4; ++p)
    {
        float ui = kali.GetValue(DSPOptionsPages::P1 + p, BankType::DSP); // 0..100
        float t = DSY_CLAMP(ui * 0.01f, 0.0f, 1.0f);                      // 0..1
        const KaliDSP::ParamSpec &spec = kali.dsp.GetParamSpec(p);
        float real = (spec.map == 1)
                         ? daisysp::fmap(t, spec.min, spec.max, daisysp::Mapping::EXP)
                         : daisysp::fmap(t, spec.min, spec.max, daisysp::Mapping::LINEAR);
        s.config_new[p] = real;
    }

    // Reverb return path is compile-time gated by ENABLE_REVERB_RETURN.
    // Keep disabled during licensing/compliance hold for closed-source builds.
#if ENABLE_REVERB_RETURN
    kali.reverb.SetFeedback(kali.GetValue(OptionsPages::ReverbFeedback) * 0.01f);
    kali.reverb.SetLpFreq(kali.GetValue(OptionsPages::ReverbDamp));
    float wetsendlevel = kali.GetValue(OptionsPages::ReverbWetSend) * 0.01f;
    float drysendlevel = kali.GetValue(OptionsPages::ReverbDrySend) * 0.01f;
#endif
    float tmpl = 0.0f, tmpr = 0.0f;

    // kali.in_audio_callback = true;

    // TODO: Delay adj
    float knob8 = inp->Knobs[8].Value();
    float knob9 = inp->Knobs[9].Value();

    // this needs to move to dsp class
    float fineadj = inp->Knobs[Kali::CV::META1].Value() * newboop * 0.3f;
    float choppe = static_cast<int>(fmap(inp->Knobs[Kali::CV::META2].Value(), 1.0f, 12.0f));
    int choppe_int = static_cast<int>(inp->Knobs[Kali::CV::META2].Value() * 16.f);

    // choppe = kali.midi.CC[22 + choppe_int] / 16.f;
    // if (choppe <= 0)
    //     choppe = 1;

    // Slower slewing for delay time parameters (restored from previous behavior)
    // Use extra smoothing for external sync to reduce jitter
    size_t slew_multiplier = (kali.masterclock.Mode != KaliClock::KaliClockMode::Internal) ? 128 : 64;
    ParameterInterpolator boopslide(&kali.delaytimes[0], (newboop / choppe) + fineadj, size * slew_multiplier);
    ParameterInterpolator kokoslide(&kali.delaytimes[1], (newkoko / choppe) + fineadj, size * slew_multiplier);

    /*
    ****** DSP LOOP *****************************************************
    *********************************************************************
    *********************************************************************
    */

    for (size_t i = 0; i < size; i++) // MAIN DSP LOOP
    {
        // has_clock here represents true/false
        // based on receiving a sync input trigger

        // For external clock modes, only pass the trigger on the first sample
        // to avoid double-triggering, but ensure accurate timing
        bool current_trigger = (i == 0) ? trigger_event : false;

        kali.TriggerReceived =
            kali.masterclock.Tick(current_trigger);

        // Reset PLL phase accumulators when we receive a trigger to keep clocks in sync
        if (kali.TriggerReceived && i == 0)
        {
            kali.left_clock_phase_accumulator = 0;
            kali.right_clock_phase_accumulator = 0;
            kali.left_clock_state = true; // Start with a pulse
            kali.right_clock_state = true;
            kali.left_gate_timer = (int)(kali.masterclock.one_ms * 5); // 5ms pulse
            kali.right_gate_timer = (int)(kali.masterclock.one_ms * 5);
        }

        kali.size = size;

        kali.delaytimes[0] = kali.isfrozen ? kali.delaytargets[0] : boopslide.Next();
        kali.delaytimes[1] = kali.isfrozen ? kali.delaytargets[1] : kokoslide.Next();
        kali.delaytimes[2] = kali.delaytimes[0] * 0.5f;
        kali.delaytimes[3] = kali.delaytimes[1] * 0.5f;

        // FIXME: Actual stereo width setting.
        if (!kali.CheckFlag(OptionsPages::InputWidth))
        {
            dry[0] = (IN_L[i] + IN_R[i]) * 0.5f;
            dry[1] = dry[0];
            dry[2] = dry[0];
            dry[3] = dry[0];
        }
        else
        {
            dry[0] = IN_L[i];
            dry[1] = IN_R[i];
            dry[2] = IN_L[i];
            dry[3] = IN_R[i];
        }

        // for (int j = 0; j < 4; j++)
        //     dry[j] = kali.dc[j].Process(dry[j]);

        // where miss piggy at
        float curmet = inp->Knobs[Kali::CV::META1].Next();
        float curmet2 = inp->Knobs[Kali::CV::META2].Next();

        // dont go outof bounds
        if (curmet < 0.00000001f)
        {
            curmet = 0.00000001f;
        }
        if (curmet2 < 0.00000001f)
        {
            curmet2 = 0.00000001f;
        }

        // update clock stuff
        float cursplat = splatslide.Next();
        float freezesource = curmet;

        // frequency base for lfo
        for (int j = 0; j < 4; j++)
        {
            samps[j] = kali.delaytimes[j] / 48000.f;
            frequencybase[j] = 48000.f / kali.delaytimes[j];
        }

        // PLL clock output generation
        // Calculate phase-locked clock outputs based on master clock timing
        if (kali.masterclock.spqn > 0)
        {
            // Get PPQN multipliers from options
            int left_ppqn = (int)kali.GetValue(OptionsPages::LeftClockRateMultiplier);
            int right_ppqn = (int)kali.GetValue(OptionsPages::RightClockRateMultiplier);

            // Calculate samples per pulse for each output
            // Higher PPQN = more pulses per quarter note = fewer samples between pulses
            int left_spp = (left_ppqn > 0) ? kali.masterclock.spqn / left_ppqn : kali.masterclock.spqn;
            int right_spp = (right_ppqn > 0) ? kali.masterclock.spqn / right_ppqn : kali.masterclock.spqn;

            // Ensure minimum pulse interval (prevent too-fast clocking)
            left_spp = DSY_MAX(left_spp, 48); // Minimum ~1ms at 48kHz
            right_spp = DSY_MAX(right_spp, 48);

            // Update phase accumulators
            kali.left_clock_phase_accumulator++;
            kali.right_clock_phase_accumulator++;

            // Generate left clock pulses
            if (kali.left_clock_phase_accumulator >= left_spp)
            {
                kali.left_clock_phase_accumulator = 0;
                kali.left_clock_state = true;
                kali.left_gate_timer = (int)(kali.masterclock.one_ms * 5); // 5ms pulse
            }

            // Generate right clock pulses
            if (kali.right_clock_phase_accumulator >= right_spp)
            {
                kali.right_clock_phase_accumulator = 0;
                kali.right_clock_state = true;
                kali.right_gate_timer = (int)(kali.masterclock.one_ms * 5); // 5ms pulse
            }

            // Handle gate pulse duration
            if (kali.left_gate_timer > 0)
            {
                kali.left_gate_timer--;
            }
            else
            {
                kali.left_clock_state = false;
            }

            if (kali.right_gate_timer > 0)
            {
                kali.right_gate_timer--;
            }
            else
            {
                kali.right_clock_state = false;
            }
        }

        // Output the phase-locked clocks to GPIO pins
        dsy_gpio_write(&kali.patch.gate_out_1, kali.left_clock_state);
        dsy_gpio_write(&kali.patch.gate_out_2, kali.right_clock_state);

        // this is a sin wave used w/ chorus modes
        float frequencybase_in;

        for (int j = 0; j < 4; j++)
        {
            samps[j] = (kali.delaytimes[j] / 192000.f);
            frequencybase_in = (1.f / (samps[j] != 0 ? samps[j] : 1.f)) * freezesource * 2.f;
            // kali.pdsp.pansine[j].SetFreq(frequencybase_in * 2.f);
        }

        if (kali.mode == Kali::Modes::Parvati && false)
        {
            /*
                In the granular modes, kali manually scans delay buffers,
                so we set these to the maximum working delay (which is 1/100
                of the largest possible delay size (which can technically
                be increased a little bit, currently set to use 50mb of
                SDRAM of a possible 64mb or so).
            */
            for (int j = 0; j < 4; j++)
                kali.delays[j]->SetDelay((float)kali.MAX_DELAY_WORKING);
        }
        else
        {
            for (int j = 0; j < 4; j++)
            {
                kali.delays[j]->SetDelay(kali.delaytimes[j]);
            }
        }

        float whichout[4];

        bool ispingpong = (kali.mode == DSPModes::PingPongLinked);

        // DISTORTION FIXME: MOVE TO KALIDSP OR SOMETHING
        int algo = kali.GetValue(DSPOptionsPages::Distortion, BankType::DSP);
        int amount = kali.GetValue(DSPOptionsPages::DistortionAmount, BankType::DSP);
        // float amount = inp->DelayAdjust * 100.f;
        int distortion_target = kali.GetValue(DSPOptionsPages::DistortionTarget, BankType::DSP);

        // 1 = DRY 3 = BOTH
        if (distortion_target == 1 || distortion_target == 3)
            kali.ApplyDistortion(dry, knob8, knob9, algo, amount);

        // set up array to send to dsp algos
        s.inp = inp;

        float warbl = (kali.warble[6].last + 2048.0f) / 2048.f;
        float warbr = (kali.warble[7].last + 2048.0f) / 2048.f;

        for (int j = 0; j < 4; j++)
        {
            s.delaytimes[j] = kali.delaytimes[j];
            s.delays[j] = kali.delays[j];
            s.dry[j] = dry[j];
        }

        s.override = kali.MAX_DELAY_WORKING;
        s.curmet = curmet;
        s.curmet2 = curmet2;
        s.warb[0] = warbl; // TODO: this nonsense will end up somewhere else
        s.warb[1] = warbr;
        s.warb[2] = warbl;
        s.warb[3] = warbr;
        s.freeze = kali.isfrozen;
        s.cursplat = cursplat;

        // OLED updates are handled in main loop to avoid I2C in audio thread
        s.allpass = kali.CheckFlag(OptionsPages::UseAllpass);
        s.size = size;

        // These are DelayPhasor objects.
        for (int k = 0; k < 4; k++)
        {
            s.dp[k] = &kali.acidburn[k];
        }

        s.MAX_DELAY_WORKING = kali.MAX_DELAY_WORKING;

        // TODO: Process function that routes to the different algorithms
        // dsp.Process
        kali.dsp.Process(s);

        for (int j = 0; j < 4; j++)
        {
            wet[j] = kali.dsp.wet[j];
            whichout[j] = kali.dsp.whichout[j]; // sometimes we want a different level feeding back than we put out
        }

        if (distortion_target == 2 || distortion_target == 3)
            kali.ApplyDistortion(wet, knob8, knob9, algo, amount);

        kali.cf.SetPos(inp->MixMapped);

        // Blend between dry/wet, with optional reverb return when enabled.
#if ENABLE_REVERB_RETURN
        float rvb_l = 0.0f;
        float rvb_r = 0.0f;
        kali.reverb.Process(wet[0] * wetsendlevel + dry[0] * drysendlevel,
                            wet[1] * wetsendlevel + dry[1] * drysendlevel,
                            &rvb_l,
                            &rvb_r);
        tmpl = wet[0] + rvb_l;
        tmpr = wet[1] + rvb_r;
#else
        tmpl = wet[0];
        tmpr = wet[1];
#endif

        if (kali.mode == Kali::Modes::ExtLoop)
        {
            OUT_L[i] = whichout[0];
            OUT_R[i] = kali.cf.Process(dry[0], dry[1]);
        }
        else if (kali.isfrozen)
        {
            // mute wet
            tmpl = 0.0f;
            tmpr = 0.0f;
            OUT_L[i] = kali.cf.Process(dry[0], tmpl);
            OUT_R[i] = kali.cf.Process(dry[1], tmpr);
        }
        else
        {
            OUT_L[i] = kali.cf.Process(dry[0], tmpl);
            OUT_R[i] = kali.cf.Process(dry[1], tmpr);
        }

        float fb[4];

        if (ENABLE_FILTERZ)
        {
            if (kali.filtmode == Kali::FilterModes::FIR && ENABLE_FIR_FILTER && inp->Cutoff < 0.98f) // if cutoff knob is all the way up, skip filter
            {
                // if (!ispingpong)
                //{
                // for (int j = 0; j < 4; j++)
                //  filtered[j] = flt[j].Process(&wet[j], 1); // Note: You might need to adjust this depending on your buffer sizes
                // filtered[j] = flt[j].ProcessSample(dry[j] + wet[j] * feedback * kali.feedback_toggle[j]);
                // for (int j = 0; j < 4; j++)
                //}
                if (ispingpong)
                {
                    float mono01 = (dry[0] + dry[1]) * 0.5f;
                    filtered[0] = flt[0].ProcessSample(mono01 + (wet[1] * feedback * kali.feedback_toggle[0]));
                    filtered[1] = flt[1].ProcessSample((wet[0] * feedback * kali.feedback_toggle[1]));
                    filtered[2] = filtered[0];
                    filtered[3] = filtered[1];
                }
                else
                {
                    for (int j = 0; j < 4; j++)
                        filtered[j] = flt[j].ProcessSample(dry[j] + wet[j] * feedback * kali.feedback_toggle[j]);
                }
            }
            else
            {
                if (ispingpong)
                {
                    float mono01 = (dry[0] + dry[1]) * 0.5f;
                    filtered[0] = mono01 + (wet[1] * feedback * kali.feedback_toggle[0]);
                    filtered[1] = (wet[0] * feedback * kali.feedback_toggle[1]);
                    filtered[2] = filtered[0];
                    filtered[3] = filtered[1];
                }
                else
                {
                    for (int j = 0; j < 4; j++)
                        filtered[j] = dry[j] + wet[j] * feedback * kali.feedback_toggle[j];
                }
            }
        }
        // TODO: ExtLoop
        if (kali.mode == Kali::Modes::ExtLoop)
        {
            filtered[0] = dry[1] * feedback;
            filtered[1] = dry[1] * feedback;
            filtered[2] = filtered[0];
            filtered[3] = filtered[1];
        }

        for (int j = 0; j < 4; j++)
            fbbuffer[j][i] = dry[j];

        if (kali.mode == Kali::Modes::ExtLoop)
        {
            // FIXME: Reimplement this
            fb[0] = dry[0] + filtered[1];
            fb[1] = dry[1] + filtered[1];
            fb[2] = dry[2] + filtered[1];
            fb[3] = dry[3] + filtered[1];
        }
        else
        {
            for (int j = 0; j < 4; j++)
                fb[j] = filtered[j];
        }

        for (int j = 0; j < 4; j++)
            kill_denormal_by_quantization(fb[j]);

        for (int j = 0; j < 4; j++)
        {
            if (!kali.isfrozen)
                kali.delays[j]->Write(fb[j]);
        }
    }
    // kali.in_audio_callback = false;

    /*
        MIDI clock output - use left clock state for MIDI timing
    */
    if (kali.left_clock_state && !kali.masterclock.gate)
    {
        kali.queue_midi_clock = true;
    }

    // For VU meter

    kali.last_sample_in[0] = IN_L[0];
    kali.last_sample_in[1] = IN_R[0];
    kali.last_sample_in[2] = IN_L[0];
    kali.last_sample_in[3] = IN_R[0];

    kali.last_sample_out[0] = OUT_L[0];
    kali.last_sample_out[1] = OUT_R[0];
    kali.last_sample_out[2] = OUT_L[0];
    kali.last_sample_out[3] = OUT_R[0];

    // follow buffer
    float *fbout[2] = {fbbuffer[0], fbbuffer[1]};

    kali.Follower.Process(size, fbout);
    for (int j = 0; j < 6; j++)
    {
        kali.warble[j].UpdateFollow();
    }

    for (int j = 0; j < 6; j++)
    {
        int fmsource = (int)kali.warble[j].preset.GetOption(LFOOptionsPages::FMSource);
        int amsource = (int)kali.warble[j].preset.GetOption(LFOOptionsPages::AMSource);
        kali.warble[j].fm = kali.warble[fmsource].last;
        kali.warble[j].am = kali.warble[amsource].last;
    }

    bool clock1, clock2;

    clock1 = kali.left_clock_state;
    clock2 = kali.right_clock_state;

    // Update clockL and clockR frequencies for LFO synchronization
    if (kali.masterclock.spqn > 0)
    {
        // Get base quarter note frequency from master clock
        float base_freq = kali.masterclock.GetFreq();

        // Apply PPQN multipliers for left and right clocks
        float left_multiplier = kali.GetValue(OptionsPages::LeftClockRateMultiplier);
        float right_multiplier = kali.GetValue(OptionsPages::RightClockRateMultiplier);

        // Calculate actual frequencies for LFO sync
        clockL = base_freq * left_multiplier;
        clockR = base_freq * right_multiplier;
    }

    // i don't want meta1 or meta2 messing up the base frequency used by clocks and oscillators
    kali.warble[0].SetFreq(clockL);
    kali.warble[1].SetFreq(clockL);
    kali.warble[2].SetFreq(clockL);
    kali.warble[3].SetFreq(clockL);
    kali.warble[4].SetFreq(clockL);
    kali.warble[5].SetFreq(clockL);
    // eastside

    /* FIXME: A lot of these are set where left delay and right delay knobs set the base frequency of l and r lfos respectively,
    this is a little awkward in practice, or at least hard to follow as I'm looking at it now. Assume it would be difficult for a user to remember. */

    /*
    for (int i = 0; i < 2; i++)
    {
        kali.warble[i].PokeWithStick(clock1);
    }

    for (int i = 2; i < 4; i++)
    {
        kali.warble[i].PokeWithStick(clock2);
    }
    kali.warble[4].PokeWithStick(clock1);
    kal*i.warble[5].PokeWithStick(clock2);
    */

    for (int i = 0; i < 8; i++)
    {
        kali.warble[i].Process(kali.warble);
        // TODO:: Eschaton thing
        // kali.warble[i].eschatonsource = &kali.warble[(int)(kali.warble[i]).preset.GetOption(LFOOptionsPages::FMSource)];
    }

    kali.patch.WriteCvOutExp(
        kali.warble[0].GetScaled(),
        kali.warble[1].GetScaled(),
        kali.warble[2].GetScaled(),
        kali.warble[3].GetScaled(),
        // kali.acidburn[0].GetModPhase() * 4096.f,
        true);

    kali.patch.WriteCvOut(1, kali.warble[4].GetScaled(), true);
    kali.patch.WriteCvOut(2, kali.warble[5].GetScaled(), true);
}

void Kali::UpdateDelayRangeForExternalSync()
{
    // Only update range in external sync modes
    if (masterclock.Mode == KaliClock::KaliClockMode::Internal)
        return;

    // Get the current tempo from external sync
    float rawSamplesPerBeat = masterclock.GetSamplesPerBeat();

    // If no valid sync yet, use a default tempo (120 BPM) for range calculation
    if (rawSamplesPerBeat <= 0)
    {
        // 120 BPM = 2 beats per second, at 48kHz = 24000 samples per beat
        rawSamplesPerBeat = 24000.0f;
    }

    // Apply heavy smoothing to prevent jitter from tempo detection fluctuations
    // Use a very slow filter coefficient for stable tempo tracking
    float smoothing_coeff = 0.002f; // Very slow smoothing (500 samples to reach ~63% of target)
    smoothed_samples_per_beat += (rawSamplesPerBeat - smoothed_samples_per_beat) * smoothing_coeff;

    // Get the current range preset
    int rangePreset = (int)GetValue(DSPOptionsPages::DelayRangePreset, BankType::DSP);

    // Define musical subdivision ranges for each preset
    float minDivision, maxDivision;

    switch (rangePreset)
    {
    case RANGE_PRECISION:
        // 1/32 triplet to 1/4 note (very short, precise timing)
        minDivision = 1.0f / 48.0f; // 1/32 triplet
        maxDivision = 1.0f / 4.0f;  // 1/4 note
        break;

    case RANGE_STUDIO:
        // 1/16 triplet to 1 bar (standard studio delays)
        minDivision = 1.0f / 24.0f; // 1/16 triplet
        maxDivision = 4.0f;         // 1 bar (4 beats)
        break;

    case RANGE_AMBIENT:
        // 1/8 triplet to 4 bars (ambient textures)
        minDivision = 1.0f / 12.0f; // 1/8 triplet
        maxDivision = 16.0f;        // 4 bars
        break;

    case RANGE_LOOPER:
        // 1/4 note to 8 bars (looping)
        minDivision = 1.0f / 4.0f; // 1/4 note
        maxDivision = 32.0f;       // 8 bars
        break;

    case RANGE_EXPERIMENTAL:
    default:
        // 1/64 triplet to 16 bars (full madness)
        minDivision = 1.0f / 96.0f; // 1/64 triplet
        maxDivision = 64.0f;        // 16 bars
        break;
    }

    // Calculate the delay range in samples using smoothed tempo
    float calculatedMin = smoothed_samples_per_beat * minDivision;
    float calculatedMax = smoothed_samples_per_beat * maxDivision;

    // Apply hardware limits
    calculatedMin = DSY_CLAMP(calculatedMin, 12.0f, MAX_DELAY);
    calculatedMax = DSY_CLAMP(calculatedMax, calculatedMin, MAX_DELAY);

    // Apply additional smoothing to the final range values to further reduce jitter
    float range_smoothing_coeff = 0.005f; // Slightly faster than tempo smoothing
    last_calculated_min += (calculatedMin - last_calculated_min) * range_smoothing_coeff;
    last_calculated_max += (calculatedMax - last_calculated_max) * range_smoothing_coeff;

    // Update the working range with smoothed values
    MIN_DELAY_WORKING = last_calculated_min;
    MAX_DELAY_WORKING = last_calculated_max;
}

bool Kali::IsUnlinked()
{
    return (kali.mode == DSPModes::StraightUnlinked) || (kali.mode == DSPModes::Resonate);
}
/*
    OLED oled drawing stuff
*/
void Kali::updateoled(void *data)
{
    // Apply contrast based on countdown (do this outside ISR)
    if (displaycountdown > 0)
    {
        if (lastcontrast != 0xFF)
            display.driver_.SetContrast(0xFF);
        lastcontrast = 0xFF;
    }
    else
    {
        if (lastcontrast != mincontrast)
            display.driver_.SetContrast(mincontrast);
        lastcontrast = mincontrast;
    }

    display.Fill(false);
    daisy::Rectangle screen(0, 0, 128, 32);

    UpdateVuMeters();
    DrawActivityIndicators();

    switch (editstate.major_mode)
    {
    case KaliEditState::LFO_EDIT:
        UpdateLfoEditScreen(screen);
        break;
    case KaliEditState::OPTIONS:
        UpdateOptionsScreen(screen);
        break;
    case KaliEditState::DSP_EDIT:
        UpdateDspEditScreen(screen);
        break;
    case KaliEditState::PRESETS:
        UpdatePresetsScreen(screen);
        break;
#ifndef NDEBUG
    case KaliEditState::MIDI_MODE:
        UpdateMIDIScreen(screen);
        break;
    case KaliEditState::DEBUG_MODE:
        UpdateDebugScreen(screen);
        break;
    case KaliEditState::CLOCK_DEBUG_MODE:
        UpdateClockDebugScreen(screen);
        break;
    default:
        UpdateDebugScreen(screen);
        break;
#endif
    }

    display.Update();
}

void Kali::UpdateVuMeters()
{
    static const float scale = 64.f;
    static const int center = 64;
    static const int yOffsetStart = 12;
    static const int yOffsetIncrement = 2;

    for (int i = 0; i < 4; ++i)
    {
        float sample = (i % 2 == 0) ? last_sample_in[i / 2] : last_sample_out[i / 2];
        int width = DSY_CLAMP(static_cast<int>(sample * scale), -center, center);
        display.DrawLine(center, yOffsetStart + i * yOffsetIncrement, center + width, yOffsetStart + i * yOffsetIncrement, true);
    }
}

void Kali::DrawActivityIndicators()
{
    if (midi_clock_flag)
        display.DrawRect(28, 28, 30, 32, true, true);

    if (inp.Gate[0])
        display.DrawRect(0, 26, 2, 28, true, true);

    if (inp.Gate[1])
        display.DrawRect(2, 26, 6, 28, true, true);

    if (kali.TriggerReceived)
        display.DrawRect(0, 28, 4, 32, true, true);

    if (midi_activity)
        display.DrawRect(4, 28, 8, 32, true, true);
}

void Kali::UpdateLfoEditScreen(const daisy::Rectangle &screen)
{
    KaliOscillator *curr_lfo = &warble[editstate.SelectedIndex];
    if (!curr_lfo)
        return;

    KaliOption *option = OptionRules[BankType::LFO][editstate.SelectedOptionIndex];
    PrintToScreen(screen, Alignment::topLeft, Font_5x5, true, "6xOSC");
    PrintLfoInfo(screen, curr_lfo, option);
    DrawLfoWaveform(curr_lfo);
}

void Kali::PrintLfoInfo(const daisy::Rectangle &screen, KaliOscillator *curr_lfo, KaliOption *option)
{
    // LFO output selection shown larger at bottom-left
    PrintToScreen(screen, Alignment::bottomLeft, Font_10x10, !editstate.iseditingsubject, "%s", string_tables[StringTableType::STLFOOutputNames][editstate.SelectedIndex]);
    PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "%s", option->ShortDescription);

    // Display scaled Meta2 knob value and effective LFO rate (knob * multiplier)
    int meta2_scaled = static_cast<int>(fmap(inp.Knobs[Kali::CV::META2].Value(), 1.0f, 12.0f));
    int effective_lfo_rate = static_cast<int>(curr_lfo->global_lfo_rate);

    PrintToScreen(screen, Alignment::bottomCentered, Font_5x5, !editstate.isediting, "M2x%2dLFOx%2d", meta2_scaled, effective_lfo_rate);

    float value = curr_lfo->preset.Options[editstate.SelectedOptionIndex];
    // Editable value always bottom-right in big font
    PrintEditableValueBig(screen, value, option->STType, option->IsFloat);
}

void Kali::DrawLfoWaveform(KaliOscillator *curr_lfo)
{
    int waveformLength = 64;
    int maxPixelHeight = 31;
    int yOffset = 16;
    float scale = 16.f;
    int curry = yOffset;

    for (int i = 0; i < waveformLength; ++i)
    {
        int posWrapped = (i + curr_lfo->last32pos) % waveformLength;
        float level = curr_lfo->last32[posWrapped] / 2048.f;
        int nexty = DSY_CLAMP(static_cast<int>(yOffset + level * scale), 0, maxPixelHeight);
        display.DrawPixel(i * 2, nexty, true);
        curry = nexty;
    }
}

void Kali::UpdateOptionsScreen(const daisy::Rectangle &screen)
{
    KaliOption *option = OptionRules[BankType::Global][editstate.SelectedOptionIndex];
    PrintToScreen(screen, Alignment::topLeft, Font_5x5, true, "OPT");
    PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "%s", option->ShortDescription);
    PrintEditableValueBig(screen, option->Value, option->STType, option->IsFloat);
}

void Kali::UpdateDspEditScreen(const daisy::Rectangle &screen)
{
    KaliOption *option = OptionRules[BankType::DSP][editstate.SelectedOptionIndex];
    float ms[2], curry;
    ms[0] = (delaytimes[0] / 48000.f) * 1000.0f;
    ms[1] = (delaytimes[1] / 48000.f) * 1000.0f;
    PrintToScreen(screen, Alignment::topLeft, Font_5x5, true, "DSP");
    PrintToScreen(screen, Alignment::centeredLeft, Font_5x5, true, "%dms", static_cast<int>(ms[0]));
    PrintToScreen(screen, Alignment::bottomLeft, Font_5x5, true, "%dms", static_cast<int>(ms[1]));
    // Show per-mode parameter label for P1..P4, otherwise generic option label
    if (editstate.SelectedOptionIndex >= DSPOptionsPages::P1 && editstate.SelectedOptionIndex <= DSPOptionsPages::P4)
    {
        const char *plabel = kali.dsp.GetParamLabel(editstate.SelectedOptionIndex - DSPOptionsPages::P1);
        PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "%s", plabel);
    }
    else
    {
        PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "%s", option->ShortDescription);
    }
    // For P1..P4, render the value using per-mode ParamSpec (range/map/units). Otherwise use default printing.
    if (editstate.SelectedOptionIndex >= DSPOptionsPages::P1 && editstate.SelectedOptionIndex <= DSPOptionsPages::P4)
    {
        int pidx = editstate.SelectedOptionIndex - DSPOptionsPages::P1; // 0..3
        const KaliDSP::ParamSpec &spec = kali.dsp.GetParamSpec(pidx);
        // Map 0..100 UI to 0..1
        float t = DSY_CLAMP(option->Value * 0.01f, 0.0f, 1.0f);
        // Apply linear/exp mapping to real units
        float mapped = (spec.map == 1)
                           ? daisysp::fmap(t, spec.min, spec.max, daisysp::Mapping::EXP)
                           : daisysp::fmap(t, spec.min, spec.max, daisysp::Mapping::LINEAR);

        // Format with units; keep it concise to fit OLED
        // Avoid %f (not supported/expensive on embedded). Use integer formatting.
        auto format_float = [](char *buf, size_t n, float v, int decimals, bool show_plus)
        {
            if (!buf || n == 0)
                return;
            char signch = 0;
            if (v < 0.0f)
            {
                signch = '-';
                v = -v;
            }
            else if (show_plus)
            {
                signch = '+';
            }

            int scale = 1;
            switch (decimals)
            {
            case 0:
                scale = 1;
                break;
            case 1:
                scale = 10;
                break;
            case 2:
                scale = 100;
                break;
            case 3:
                scale = 1000;
                break;
            default:
                scale = 10000; // cap at 4 decimals
                decimals = 4;
                break;
            }

            int ipart = (int)floorf(v);
            int fpart = (int)floorf((v - (float)ipart) * (float)scale + 0.5f);
            if (fpart >= scale)
            {
                ipart += 1;
                fpart -= scale;
            }

            if (decimals <= 0)
            {
                if (signch)
                    snprintf(buf, n, "%c%d", signch, ipart);
                else
                    snprintf(buf, n, "%d", ipart);
            }
            else
            {
                if (signch)
                    snprintf(buf, n, "%c%d.%0*d", signch, ipart, decimals, fpart);
                else
                    snprintf(buf, n, "%d.%0*d", ipart, decimals, fpart);
            }
        };

        char num[24];
        switch (spec.unit)
        {
        case 'H': // Hertz
        {
            // Show decimals for low Hz ranges, int otherwise
            if (spec.max <= 10.0f || mapped < 10.0f)
            {
                format_float(num, sizeof(num), mapped, 2, false);
                PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%sHz", num);
            }
            else
            {
                format_float(num, sizeof(num), mapped, 0, false);
                PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%sHz", num);
            }
            break;
        }
        case 'm': // milliseconds
            format_float(num, sizeof(num), mapped, 0, false);
            PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%sms", num);
            break;
        case '%': // percent
            format_float(num, sizeof(num), mapped * 100.0f, 0, false);
            PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%s%%", num);
            break;
        case 's': // semitones
            format_float(num, sizeof(num), mapped, 1, true /*show + for positive*/);
            PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%ss", num);
            break;
        default:
            // Generic float with two decimals
            format_float(num, sizeof(num), mapped, 2, false);
            PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%s", num);
            break;
        }
    }
    else
    {
        PrintEditableValueBig(screen, option->Value, option->STType, option->IsFloat);
    }
    // TODO: For testing/monitoring
    curry = acidburn[0].GetModPhase() * 32.0f;
    /* code */
    display.DrawPixel(64, curry, true);
}

/*
    Short variadic function to write formatted strings to oled.
*/
void Kali::PrintToScreen(const daisy::Rectangle screen, const Alignment alignment, FontDef f, bool on, const char *format, ...)
{
    char toscreen[32];
    va_list args;
    va_start(args, format);
    vsnprintf(toscreen, sizeof(toscreen), format, args);
    va_end(args);

    display.WriteStringAligned(toscreen, f, screen, alignment, on);
}

#ifndef NDEBUG
void Kali::UpdateDebugScreen(const daisy::Rectangle &screen)
{
    PrintToScreen(screen, Alignment::topLeft, Font_5x5, true, "Debug");

    kali.editstate.SelectedIndex = DSY_CLAMP(kali.editstate.SelectedIndex, 0, 5);

    // Addit0ional debug information can be printed here as needed.
    //         for (int i = 0; i < 6; i++)
    //{
    //    kali.lasttbl[i] = (kali.warble[i].last / 4096.f);
    //}

    switch (kali.editstate.SelectedIndex)
    {

    case 0:
        PrintToScreen(screen, Alignment::centeredLeft, Font_5x5, true, "%d", (int)(kali.patch.controls[3].GetRawFloat() * 1000.f));
        PrintToScreen(screen, Alignment::bottomLeft, Font_5x5, true, "%d", (int)(kali.inp.Knobs[3].Value() * 1000.f));
        PrintToScreen(screen, Alignment::topCentered, Font_5x5, true, "ta-%5d", (int)(kali.masterclock.TriggerAccumulator));
        PrintToScreen(screen, Alignment::centered, Font_5x5, true, "sa-%5d", (int)(kali.masterclock.SampleAccumulator));
        PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "%d", (int)kali.masterclock.GetBPM());
        PrintToScreen(screen, Alignment::bottomRight, Font_5x5, true, "%d", (int)(kali.masterclock.gate_timer ? 1 : 0));
        if (kali.TriggerReceived)
        {
            PrintToScreen(screen, Alignment::bottomCentered, Font_5x5, true, "RECV SYNC");
        }
        break;
    case 1:
        PrintToScreen(screen, Alignment::centeredLeft, Font_5x5, true, "%d  %d  %d  %d",
                      (int)(kali.patch.controls[0].GetRawFloat() * 10000.f),
                      (int)(kali.patch.controls[1].GetRawFloat() * 10000.f),
                      (int)(kali.patch.controls[2].GetRawFloat() * 10000.f),
                      (int)(kali.patch.controls[3].GetRawFloat() * 10000.f));
        PrintToScreen(screen, Alignment::bottomLeft, Font_5x5, true, "%d  %d  %d  %d",
                      (int)(kali.patch.controls[4].GetRawFloat() * 10000.f),
                      (int)(kali.patch.controls[5].GetRawFloat() * 10000.f),
                      (int)(kali.patch.controls[6].GetRawFloat() * 10000.f),
                      (int)(kali.patch.controls[7].GetRawFloat() * 10000.f));
        PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "%d %d", kali.delays[0]->GetWritePtr(), kali.delays[1]->GetWritePtr());
        break;
    case 2:
        /*
        PrintToScreen(screen, Alignment::bottomCentered, Font_5x5, true, "%d %d %d %d",
                      (int)(kali.delaytimes[0]),
                      (int)(kali.delaytimes[1]),
                      (int)(kali.delaytimes[2]),
                      (int)(kali.delaytimes[3]));
                      */
        // Add debug information for modPhase_, modDepth_, and delaySamples_
        PrintToScreen(screen, Alignment::topCentered, Font_5x5, true, "%d", static_cast<int>(acidburn[0].GetModPhase() * 100.0f));
        PrintToScreen(screen, Alignment::centeredLeft, Font_5x5, true, "%d", static_cast<int>(acidburn[0].GetModDepth() * 100.0f));
        PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "%d", static_cast<int>(acidburn->GetDelayPosition()));
        PrintToScreen(screen, Alignment::centeredRight, Font_5x5, true, "%d", static_cast<int>(kali.acidburn->GetModulatedPosition(true)));
        PrintToScreen(screen, Alignment::bottomRight, Font_5x5, true, "%d", static_cast<int>(acidburn[0].GetDelaySamples()));
        break;
    case 3:
        PrintToScreen(screen, Alignment::centeredLeft, Font_5x5, true, "%d %d %d %d",
                      (int)(kali.patch.controls[0].GetRawValue()),
                      (int)(kali.patch.controls[1].GetRawValue()),
                      (int)(kali.patch.controls[2].GetRawValue()),
                      (int)(kali.patch.controls[3].GetRawValue()));
        PrintToScreen(screen, Alignment::bottomLeft, Font_5x5, true, "%d %d %d %d",
                      (int)(kali.patch.controls[4].GetRawValue()),
                      (int)(kali.patch.controls[5].GetRawValue()),
                      (int)(kali.patch.controls[6].GetRawValue()),
                      (int)(kali.patch.controls[7].GetRawValue()));
        break;
    case 4:
#ifdef ENABLE_WAVETABLE_EDITOR
        // draw kali knob 1's wavetable (optional editor preview)
        {
            PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "Wavetable");

            const int tableSize = 16;
            const int width = 120;
            const int startX = 4;
            const int startY = 16;
            const int height = 24;
            const int stepWidth = width / tableSize;
            const int screenW = display.Width();
            const int screenH = display.Height();

            // Draw wavetable grid background
            display.DrawRect(startX, startY - 12, startX + width, startY + height - 12, true, false);

            // Calculate vertical center position (zero point)
            const int verticalCenter = startY - 12 + (height / 2);

            // Draw center line (zero line)
            display.DrawLine(startX, verticalCenter, startX + width, verticalCenter, true);

            // Calculate y-positions for all points first
            int yPositions[tableSize];
            for (int i = 0; i < tableSize; i++)
            {
                // Get wavetable value (0-127) and map to bipolar range
                // 63.5 should be center, 0 at bottom, 127 at top
                // knob wt
                // knob wt
                // Use 0..255 range so center is 127.5 and output is roughly -0.5..+0.5
                float wtVal = static_cast<float>(inp.Knobs[0].wavetable.Peek(i));
                float normValue = (wtVal - 127.5f) / 255.0f; // -0.5 to +0.5

                // Calculate y position centered around middle
                // Multiply by height and invert (negative values go up from center)
                int y = verticalCenter - static_cast<int>(normValue * height) - 4;
                // Clamp to screen to avoid out-of-bounds drawing
                yPositions[i] = DSY_CLAMP(y, 0, screenH - 1);
            }

            // Draw connected line segments
            for (int i = 0; i < tableSize - 1; i++)
            {
                int x1 = startX + (i * stepWidth);
                int x2 = startX + ((i + 1) * stepWidth);
                display.DrawLine(x1, yPositions[i], x2, yPositions[i + 1], true);
            }

            // Use Knobs[0] position to determine current position in wavetable
            float normalized = (inp.Knobs[0].Value() * tableSize);
            int currentPos = static_cast<int>(normalized) % tableSize;
            currentPos = DSY_CLAMP(currentPos, 0, tableSize - 1);
            int x = startX + (currentPos * stepWidth);

            // Draw position indicator as small vertical line
            display.DrawLine(x, startY - 14, x, startY - 12, true);

            // Add small circle at current position point
            display.DrawCircle(x, yPositions[currentPos], 2, true);
        }
#else
    {
        PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "DSPDbg");
        // Show mode and active grains
        const char *name = kali.dsp.GetCurrentModeName();
        PrintToScreen(screen, Alignment::topCentered, Font_5x5, true, "%s", name);
        int gL = kali.dsp.CountActiveGrains(0);
        int gR = kali.dsp.CountActiveGrains(1);
        PrintToScreen(screen, Alignment::centeredLeft, Font_5x5, true, "GL:%d GR:%d", gL, gR);
        // Show key params
        PrintToScreen(screen, Alignment::bottomLeft, Font_5x5, true, "M1:%d M2:%d", (int)(inp.Knobs[Kali::CV::META1].Value() * 100), (int)(inp.Knobs[Kali::CV::META2].Value() * 100));
        PrintToScreen(screen, Alignment::bottomRight, Font_5x5, true, "P1:%d P2:%d", (int)(GetValue(DSPOptionsPages::P1, BankType::DSP) * 100), (int)(GetValue(DSPOptionsPages::P2, BankType::DSP) * 100));
    }
#endif

    default:
        break;
    }
}

void Kali::UpdateMIDIScreen(const daisy::Rectangle &screen)
{
    // KaliOption *option = OptionRules[Kali::BankType::LFO][editstate.SelectedOptionIndex];
    PrintToScreen(screen, Alignment::topLeft, Font_5x5, true, "MIDI");
    // last 5 events?
    MidiEvent e = GetLastMIDIEvent();
    if (e.type == MidiMessageType::ControlChange)
    {
        auto eventinfo = e.AsControlChange();
        PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "%d/%d/%d", eventinfo.control_number, eventinfo.value, eventinfo.channel);
    }
    else if (e.type == MidiMessageType::NoteOn)
    {
        auto eventinfo = e.AsNoteOn();
        PrintToScreen(screen, Alignment::topRight, Font_5x5, true, "%d/%d", eventinfo.note, eventinfo.velocity);
    }

    PrintToScreen(screen, Alignment::centeredLeft, Font_5x5, true, "TA%3d", (int)(kali.masterclock.TriggerAccumulator));
    PrintToScreen(screen, Alignment::centeredRight, Font_5x5, true, "SA%3d", (int)(kali.masterclock.SampleAccumulator));
    PrintToScreen(screen, Alignment::bottomLeft, Font_5x5, true, "BPM%d", (int)kali.masterclock.GetBPM());
    PrintToScreen(screen, Alignment::bottomRight, Font_5x5, true, "SPQN%4d/PPQN%3d", kali.masterclock.spqn, kali.masterclock.internal_ppqn);
}

void Kali::UpdateClockDebugScreen(const daisy::Rectangle &screen)
{
    PrintToScreen(screen, Alignment::topLeft, Font_5x5, true, "Clock Monitor");

    // Basic timing info with more precision
    // Avoid float formatting to keep printf footprint minimal
    PrintToScreen(screen, Alignment::topLeft, Font_5x5, true,
                  "BPM:%d SPQN:%d", static_cast<int>(masterclock.GetBPM()), masterclock.spqn);

    // Mode and status
    const char *modeNames[] = {"Int", "Ext", "MIDI", "None"};
    PrintToScreen(screen, Alignment::topLeft, Font_5x5, true,
                  "Mode:%s Active:%s",
                  modeNames[masterclock.Mode],
                  masterclock.IsExternalClockPresent() ? "Yes" : "No");

    // PPQN settings and timeout counter
    PrintToScreen(screen, Alignment::centeredRight, Font_5x5, true,
                  "PPQN:%d Timeout:%d",
                  masterclock.GetCurrentPPQN(),
                  masterclock.clock_timeout_counter);

    // Gate input states
    PrintToScreen(screen, Alignment::centeredLeft, Font_5x5, true,
                  "G1:%s G2:%s",
                  inp.Gate[0] ? "HI" : "lo",
                  inp.Gate[1] ? "HI" : "lo");

    // Counter values - show both accumulators for debugging
    PrintToScreen(screen, Alignment::bottomLeft, Font_5x5, true,
                  "TA:%d SA:%d TM:%d",
                  masterclock.TriggerAccumulator,
                  masterclock.SampleAccumulator,
                  masterclock.TimingAccumulator);

    // Show averaging buffer contents to debug timing detection
    if (masterclock.full > 0)
    {
        PrintToScreen(screen, Alignment::bottomRight, Font_5x5, true,
                      "Buf:%d,%d,%d F:%d",
                      (int)masterclock.foravg[0],
                      masterclock.full > 1 ? (int)masterclock.foravg[1] : 0,
                      masterclock.full > 2 ? (int)masterclock.foravg[2] : 0,
                      masterclock.full);
    }
    else
    {
        PrintToScreen(screen, Alignment::bottomRight, Font_5x5, true,
                      "No timing data");
    }
}
#endif

void Kali::PrintFormattedValue(const daisy::Rectangle &screen, float value, StringTableType stType, bool is_float = false)
{
    unsigned int val = static_cast<unsigned int>(value);
    const char *sign = (value < 0) ? "-" : "";
    value = std::abs(value);
    int intValue = static_cast<int>(value);
    float fracValue = value - intValue;
    int fracInt = static_cast<int>(fracValue * 10000);

    if (stType == StringTableType::None)
    {
        if (is_float)
            PrintToScreen(screen, Alignment::bottomRight, Font_5x5, !editstate.isediting, "%s%d.%04d", sign, intValue, fracInt);
        else
            PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%d", intValue);
    }
    else
    {
        PrintToScreen(screen, Alignment::bottomRight, Font_5x5, !editstate.isediting, "%s\0", string_tables[stType][val]);
    }
}

// Always print editable values using 10x10 at bottom-right (ints, floats, strings)
void Kali::PrintEditableValueBig(const daisy::Rectangle &screen, float value, StringTableType stType, bool is_float)
{
    unsigned int val = static_cast<unsigned int>(value);
    const char *sign = (value < 0) ? "-" : "";
    value = std::abs(value);
    int intValue = static_cast<int>(value);
    float fracValue = value - intValue;
    int fracInt = static_cast<int>(fracValue * 10000);

    if (stType == StringTableType::None)
    {
        if (is_float)
            PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%s%d.%04d", sign, intValue, fracInt);
        else
            PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%d", intValue);
    }
    else
    {
        PrintToScreen(screen, Alignment::bottomRight, Font_10x10, !editstate.isediting, "%s\0", string_tables[stType][val]);
    }
}

// ---- Presets (6 LFOs + 1 DSP) ----
namespace
{
    static constexpr uint32_t MAIN_PRESET_MAGIC = 0x4B414C50; // 'KALP'
    static constexpr uint32_t MAIN_PRESET_VERSION = 1;
    static constexpr uint32_t MAIN_PRESET_BASE = 0x0010000; // 64KB into QSPI
    static constexpr uint32_t MAIN_PRESET_SECTOR = 0x1000;  // 4KB per slot

    struct MainPresetEntry
    {
        uint32_t magic;
        uint32_t version;
        char name[16];
        float lfo[6][LFOOptionsPages::KALI_LFO_OPTIONS_LAST];
        float dsp[DSPOptionsPages::KALI_DSP_OPTIONS_LAST];
        float knob_snapshot[8];     // Knob positions at save time (for soft takeover)
        uint8_t last_selected_slot; // Remember last selection
        uint8_t reserved[3];        // Padding for alignment
    };

    // Persistent config location (separate from presets)
    static const uint32_t PERSISTENT_CONFIG_ADDR = MAIN_PRESET_BASE + (MAX_PRESETS * MAIN_PRESET_SECTOR);

    struct PersistentConfig
    {
        uint32_t magic;
        uint8_t last_preset_slot;
        uint8_t reserved[251]; // Pad to 256 bytes
    };
    static const uint32_t PERSISTENT_CONFIG_MAGIC = 0xCA11C0FF; // CALLCOFF (Kali Config)

    static inline uint32_t preset_slot_addr(int slot)
    {
        if (slot < 0)
            slot = 0;
        if (slot >= MAX_PRESETS)
            slot = MAX_PRESETS - 1;
        return MAIN_PRESET_BASE + static_cast<uint32_t>(slot) * MAIN_PRESET_SECTOR;
    }
}

bool Kali::ReadMainPresetMeta(int slot, char *name_out, bool &valid)
{
    valid = false;
    if (!name_out)
        return false;
    uint32_t addr = preset_slot_addr(slot);
    auto *entry = reinterpret_cast<const MainPresetEntry *>(patch.qspi.GetData(addr));
    if (entry && entry->magic == MAIN_PRESET_MAGIC && entry->version == MAIN_PRESET_VERSION)
    {
        // Copy name (ensure null-termination)
        for (int i = 0; i < 15; ++i)
            name_out[i] = entry->name[i];
        name_out[15] = '\0';
        valid = true;
        return true;
    }
    name_out[0] = '\0';
    return false;
}

void Kali::SaveMainPreset(int slot)
{
    MainPresetEntry e{};
    e.magic = MAIN_PRESET_MAGIC;
    e.version = MAIN_PRESET_VERSION;

    // Generate random name (4-8 characters)
    PresetNameGenerator::generateName(&patch, e.name, sizeof(e.name), 4, 8);

    // Capture current knob positions for soft takeover
    for (int i = 0; i < 8; ++i)
    {
        e.knob_snapshot[i] = inp.Knobs[i].Value();
    }

    // Store this as last selected slot
    e.last_selected_slot = static_cast<uint8_t>(slot);

    // Copy 6 LFOs (0..5)
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < LFOOptionsPages::KALI_LFO_OPTIONS_LAST; ++j)
        {
            e.lfo[i][j] = warble[i].preset.Options[j];
        }
    }
    // Copy DSP options
    for (int j = 0; j < DSPOptionsPages::KALI_DSP_OPTIONS_LAST; ++j)
    {
        e.dsp[j] = OptionRules[BankType::DSP][j]->Value;
    }

    uint32_t addr = preset_slot_addr(slot);
    // Erase sector, then write
    patch.qspi.Erase(addr, addr + MAIN_PRESET_SECTOR);
    patch.qspi.Write(addr, sizeof(MainPresetEntry), reinterpret_cast<uint8_t *>(&e));

    // Cache last name for UI popup
    for (int i = 0; i < 15; ++i)
        last_preset_name[i] = e.name[i];
    last_preset_name[15] = '\0';
}

bool Kali::LoadMainPreset(int slot)
{
    uint32_t addr = preset_slot_addr(slot);
    auto *e = reinterpret_cast<const MainPresetEntry *>(patch.qspi.GetData(addr));
    if (!e || e->magic != MAIN_PRESET_MAGIC || e->version != MAIN_PRESET_VERSION)
        return false;

    // Apply to runtime
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < LFOOptionsPages::KALI_LFO_OPTIONS_LAST; ++j)
        {
            warble[i].preset.Options[j] = e->lfo[i][j];
        }
    }
    for (int j = 0; j < DSPOptionsPages::KALI_DSP_OPTIONS_LAST; ++j)
    {
        OptionRules[BankType::DSP][j]->Value = e->dsp[j];
    }

    // Enable soft takeover for the 8 main knobs using saved snapshot
    for (int i = 0; i < 8; ++i)
    {
        inp.Knobs[i].EnableSoftTakeover(e->knob_snapshot[i]);
    }

    // Reset some engines to ensure clean state after large changes
    ResetAllThings();

    // Cache last name for UI
    for (int i = 0; i < 15; ++i)
        last_preset_name[i] = e->name[i];
    last_preset_name[15] = '\0';

    // Save this as last selected slot
    SaveLastSelectedPresetSlot(slot);

    return true;
}

void Kali::SaveLastSelectedPresetSlot(int slot)
{
    PersistentConfig cfg{};
    cfg.magic = PERSISTENT_CONFIG_MAGIC;
    cfg.last_preset_slot = static_cast<uint8_t>(slot);

    // Write to QSPI
    patch.qspi.Erase(PERSISTENT_CONFIG_ADDR, PERSISTENT_CONFIG_ADDR + 4096);
    patch.qspi.Write(PERSISTENT_CONFIG_ADDR, sizeof(PersistentConfig),
                     reinterpret_cast<uint8_t *>(&cfg));
}

int Kali::LoadLastSelectedPresetSlot()
{
    auto *cfg = reinterpret_cast<const PersistentConfig *>(
        patch.qspi.GetData(PERSISTENT_CONFIG_ADDR));

    if (cfg && cfg->magic == PERSISTENT_CONFIG_MAGIC)
    {
        return static_cast<int>(cfg->last_preset_slot);
    }

    return 0; // Default to slot 0
}

void Kali::UpdatePresetsScreen(const daisy::Rectangle &screen)
{
    // Title
    PrintToScreen(screen, Alignment::topLeft, Font_5x5, true, "PRESETS");

    // Action indicator - inverted style when in save mode
    const char *act = preset_save_mode ? "Save" : "Load";
    PrintToScreen(screen, Alignment::topRight, Font_5x5, !preset_save_mode, "%s", act);

    // Slot number on left, centered vertically
    int slot = editstate.SelectedPresetIndex;
    PrintToScreen(screen, Alignment::centeredLeft, Font_5x5, true, "SLOT %02d", slot);

    // Preset name: right-aligned, centered vertically, using 5x5 font
    bool valid = false;
    char namebuf[16];
    ReadMainPresetMeta(slot, namebuf, valid);
    if (!valid)
        snprintf(namebuf, sizeof(namebuf), "(empty)");

    PrintToScreen(screen, Alignment::centeredRight, Font_5x5, !preset_save_mode, "%s", namebuf);

    // Bottom hints - simplified
    if (preset_save_mode)
    {
        PrintToScreen(screen, Alignment::bottomLeft, Font_5x5, true, "RClk:Save  LClk:Cancel");
    }
    else
    {
        PrintToScreen(screen, Alignment::bottomLeft, Font_5x5, true, "RClk:Load  RHold:Save");
    }
}

int lastadcatten = -1;
int lastdacatten = -1;
/*
 * TIM2 IRQ HANDLER
 */
void handlar(void *data)
{
    // Timer ISR: Avoid heavy/blocking I2C OLED operations here.
    // Keep this ISR minimal: just maintain lightweight counters/flags.

    if (kali.displaycountdown > 0)
    {
        kali.displaycountdown--;
    }

    /*
    TODO: This is for midi clock out but it was interrupting other time
    sensitive things, very jittery. Leaving here just in case.

    if(!kali.midi_clock_flag && kali.queue_midi_clock) {
        patch.MIDISendClock();
        kali.queue_midi_clock = false;
    }
    */
}

void Kali::HandleEncoders(KaliInput *inp)
{
    e1inc = patch.e1.Increment();
    e2inc = patch.e2.Increment();

    if (CheckFlag(OptionsPages::InvertEncoders))
    {
        e1inc *= -1;
        e2inc *= -1;
    }

    // New unified event handling
    if (e1inc != 0 && patch.e1.Pressed())
    {
        handle_event(KaliEvent::RIGHTHOLDANDTURN);
    }
    else if (e1inc != 0)
    {
        handle_event(KaliEvent::RIGHTTURN);
    }

    if (e2inc != 0 && patch.e2.Pressed())
    {
        handle_event(KaliEvent::LEFTHOLDANDTURN);
    }
    else if (e2inc != 0)
    {
        handle_event(KaliEvent::LEFTTURN);
    }

    // Check for hold while button is pressed
    if (patch.e1.Pressed() && patch.e1.TimeHeldMs() > 400 && !e1_was_held)
    {
        handle_event(KaliEvent::RIGHTHOLD);
        e1_was_held = true; // Mark that this button was held
    }

    // Check for hold while button is pressed
    if (patch.e2.Pressed() && patch.e2.TimeHeldMs() > 400 && !e2_was_held)
    {
        handle_event(KaliEvent::LEFTHOLD);
        e2_was_held = true; // Mark that this button was held
    }

    // Fire click on release, only if button wasn't held
    if (patch.e2.FallingEdge())
    {
        if (!e2_was_held)
        {
            handle_event(KaliEvent::LEFTCLICK);
        }
        editstate.encoder_timers[1] = 0;
        e2_was_held = false; // Reset hold flag on release
    }
    if (patch.e1.FallingEdge())
    {
        if (!e1_was_held)
        {
            handle_event(KaliEvent::RIGHTCLICK);
        }
        editstate.encoder_timers[0] = 0;
        e1_was_held = false; // Reset hold flag on release
    }

    // Update button states
    prevbut1 = patch.e1.Pressed();
    prevbut2 = patch.e2.Pressed();

    if (prevbut2 && prevbut1)
    {
        saveSettings = true;
    }
    if (prevbut1 && isfrozen)
    {
        ResetAllThings();
    }
}

// makes shit blink when you turn kali on
void toglights(bool tog)
{
    kali.patch.WriteCvOut(1, tog * 4095, true);
    kali.patch.WriteCvOut(2, tog * 4095, true);
    kali.patch.WriteCvOutExp((1 - tog) * 4095, (1 - tog) * 4095, (1 - tog) * 4095, (1 - tog) * 4095, true);

    dsy_gpio_write(&kali.patch.gate_out_1, tog);
    dsy_gpio_write(&kali.patch.gate_out_2, 1 - tog);

    kali.patch.Delay(20);
}

void Kali::LoadLFOPreset(int SelectedIndex, int SelectedPresetIndex)
{
    uint32_t address_offset_ = 0;
    address_offset_ = 0x00 & (uint32_t)(~0xff);

    for (int j = 0; j < LFOOptionsPages::KALI_LFO_OPTIONS_LAST; j++)
    {
        warble[SelectedIndex].preset.SetOption(j, options.LFOPresets[SelectedPresetIndex].Options[j]);
        editstate.SelectedOptionIndex = 0;
    }
    warble[SelectedIndex].preset.Options[LFOOptionsPages::LFOPresetSave] = SelectedPresetIndex;
    warble[SelectedIndex].preset.Options[LFOOptionsPages::LFOPresetLoad] = 0;
    editstate.SelectedOptionIndex = 0; // jump back to first option
}

void Kali::LoadOptions(bool init)
{
    bool wasInitialized;

    patch.ProcessAllControls();
    patch.e1.Debounce();
    patch.e2.Debounce();

    OptimizedKaliConfig config;
    if (init)
    {
        KaliVersionManager::initializeConfig(config);
    }
    else
    {
        wasInitialized = KaliVersionManager::readConfig(patch, config);
    }

    // Transfer config data to actual objects (only first 6 LFOs are stored in config)
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < LFOOptionsPages::KALI_LFO_OPTIONS_LAST; j++)
        {
            warble[i].preset.SetOption(j, config.LFOOptions[i][j]);
        }
    }

    // Set global options
    for (int k = 0; k < OptionsPages::KALI_OPTIONS_LAST; k++)
    {
        SetOptionValue(k, config.GlobalOptions[k]);
    }

    // Set DSP options
    for (int k = 0; k < DSPOptionsPages::KALI_DSP_OPTIONS_LAST; k++)
    {
        SetOptionValue(k, config.DSPOptions[k], BankType::DSP);
    }
}

void Kali::SaveOptions()
{
    OptimizedKaliConfig config;

    // Copy current state to config
    for (int i = 0; i < OptionsPages::KALI_OPTIONS_LAST; i++)
    {
        config.GlobalOptions[i] = OptionRules[BankType::Global][i]->Value;
    }

    for (int i = 0; i < MAX_LFOS; i++)
    {
        for (int j = 0; j < LFOOptionsPages::KALI_LFO_OPTIONS_LAST; j++)
        {
            config.LFOOptions[i][j] = warble[i].preset.Options[j];
        }
    }

    for (int i = 0; i < DSPOptionsPages::KALI_DSP_OPTIONS_LAST; i++)
    {
        config.DSPOptions[i] = OptionRules[BankType::DSP][i]->Value;
    }

    KaliVersionManager::writeConfig(patch, config);
}

int main(void)
{
    int samplerate = 48000;
    // printf("Yes I am cooking for my son and his wife.\n");
    kali.patch.Init();

    kali.Init(samplerate);

    // Initialize MIDI CC mappings after kali is initialized
    kali.midi.initializeCCMappings(&kali);

    // Check if both gate inputs are held down for factory reset
    // This allows users to start with clean defaults if saved state is corrupted
    bool factory_reset = kali.patch.gate_in_1.State() && kali.patch.gate_in_2.State();

    if (!factory_reset)
    {
        kali.LoadOptions(false);
    }
    // If factory_reset is true, skip loading - use defaults

    kali.patch.Delay(10);

    kali.patch.SetAudioSampleRate(samplerate);
    kali.patch.SetAudioBlockSize(96);

    kali.patch.Delay(20);

    // initialise_monitor_handles();

    kali.patch.WriteCvOut(1, 4095, true);
    kali.patch.WriteCvOut(2, 4095, true);
    kali.patch.WriteCvOutExp(0, 0, 0, 0, true);

    dsy_gpio_write(&kali.patch.gate_out_1, 1);
    dsy_gpio_write(&kali.patch.gate_out_2, 1);

    kali.patch.Delay(50);

    MyOledDisplay::Config disp_cfg;
    disp_cfg.driver_config.transport_config.i2c_config.periph = I2CHandle::Config::Peripheral::I2C_1;
    disp_cfg.driver_config.transport_config.i2c_config.speed = I2CHandle::Config::Speed::I2C_1MHZ;
    disp_cfg.driver_config.transport_config.i2c_config.mode = I2CHandle::Config::Mode::I2C_MASTER;
    disp_cfg.driver_config.transport_config.i2c_config.pin_config.scl = DPT::B7;
    disp_cfg.driver_config.transport_config.i2c_config.pin_config.sda = DPT::B8;
    disp_cfg.driver_config.transport_config.i2c_address = 0x3c;

    I2CHandle i2c_oled;
    i2c_oled.Init(disp_cfg.driver_config.transport_config.i2c_config);

    int tog = 0;

    // keep trying to write to the screen until it is on for sure
    while (i2c_oled.TransmitBlocking(0x3C, nullptr, 0, 10) != I2CHandle::Result::OK)
    {
        toglights(tog);
        tog = 1 - tog;

        kali.patch.Delay(20);
    };

    kali.patch.Delay(250);

    display.Init(disp_cfg);

    int frame = 0;
    int frame_count = 1;

    daisy::Rectangle screen(0, 0, 128, 32);

    display.Fill(false);

    // Display factory reset message if triggered
    if (factory_reset)
    {
        kali.PrintToScreen(screen, daisy::Alignment::centeredLeft, Font_5x5, true, "FACTORY RESET");
        kali.PrintToScreen(screen, daisy::Alignment::bottomLeft, Font_5x5, true, "Loading defaults...");
        display.Update();
        kali.patch.Delay(2000); // Show message for 2 seconds
        display.Fill(false);
    }

    /*
    // display.driver_.SetContrast(0xi0);
    for (int blinking = 0; blinking < 3; blinking++)
    {
        for (frame = 0; frame < frame_count; frame++)
        {
            for (int y = 0; y < 32; y++)
            {
                for (int x = 0; x < 16; x++)
                {
                    for (int bit = 0; bit < 8; bit++)
                    {
                        // display.DrawPixel((x * 8) + bit, y, (frames[frame][x + (y * 16)]) & (0x80 >> bit));
                    }
                }
            }

            display.Update();

            toglights(tog);
            tog = 1 - tog;

            kali.patch.Delay(5 + blinking * 5);
        }
    }
    display.Update();
    */

    kali.patch.StartAudio(AudioCallback);
    kali.patch.InitTimer(handlar, nullptr);

    kali.inp.ProcessControls(&kali.patch);

    kali.patch.midi.StartReceive();

    // kali.SaveOptions();
    // kali.patch.Delay(10);

    // UI/OLED + MIDI + maintenance loop (decoupled from ISR/audio)
    uint32_t last_oled_ms = 0;
    const uint32_t min_oled_interval_ms = 33; // ~30 fps
    uint32_t last_midi_ms = 0;
    const uint32_t midi_poll_interval_ms = 1; // poll MIDI frequently in main loop
    while (1)
    {
        uint32_t now_ms = System::GetNow();

        // Poll MIDI in main loop (not ISR)
        if (now_ms - last_midi_ms >= midi_poll_interval_ms)
        {
            kali.HandleMIDI();
            last_midi_ms = now_ms;
        }

        // Deferred saves and codec attenuation updates (avoid ISR work)
        if (kali.saveSettings)
        {
            if (kali.initSettings)
            {
                kali.LoadOptions(kali.initSettings);
                kali.initSettings = false;
            }
            kali.SaveOptions();
            kali.saveSettings = false;
        }

        // Ensure sane attenuation defaults, and apply when changed
        if (kali.GetValue(OptionsPages::AdcAttenuation) == 0)
        {
            kali.SetOptionValue(OptionsPages::AdcAttenuation, 215);
        }
        if (kali.GetValue(OptionsPages::DacAttenuation) == 0)
        {
            kali.SetOptionValue(OptionsPages::DacAttenuation, 255);
        }
        if (kali.GetValue(OptionsPages::AdcAttenuation) != lastadcatten)
        {
            kali.patch.codec.SetAdcAttenuation(kali.GetValue(OptionsPages::AdcAttenuation));
            lastadcatten = kali.GetValue(OptionsPages::AdcAttenuation);
        }
        if (kali.GetValue(OptionsPages::DacAttenuation) != lastdacatten)
        {
            kali.patch.codec.SetDacAttenuation(kali.GetValue(OptionsPages::DacAttenuation));
            lastdacatten = kali.GetValue(OptionsPages::DacAttenuation);
        }

        now_ms = System::GetNow();
        if (now_ms - last_oled_ms >= min_oled_interval_ms)
        {
            kali.updateoled(nullptr);
            last_oled_ms = now_ms;
        }
        // Small sleep to yield CPU
        kali.patch.Delay(5);
    }
}
