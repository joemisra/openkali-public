#pragma once
#ifndef KALI_H
#define KALI_H

#define ENABLE_FILTERZ 1
#define ENABLE_BUTTANS 1
#define ENABLE_FIR_FILTER 1
#define ENABLE_SINSPREAD 1
#ifndef ENABLE_WAVETABLE_EDITOR
#define ENABLE_WAVETABLE_EDITOR 0
#endif

#include "consts.h"
#include "KaliOscillator.h"
#include "KaliConfig.h"
#include "dpt/daisy_dpt.h"
#include "daisysp.h"
#include "sys/system.h"
#include "KaliTypes.h"
#include "KaliInput.h"
#include "KaliInputState.h"
#include "KaliClock.h"
#include "KaliPlayheadEngine.h"
#include "KaliDsp.h"
#include "KaliState.h"
#include "KaliSpread.h"
#include "DelayPhasor.h"
#include "KaliMIDI.h"
#include "KaliVersion.h"
#include "KaliFreezeEngine.h"
#include "EnvelopeFollower.h" // Include the new header
#include "stringtables.h"
#include <memory>
#include <map>
// Add to Kali class:

using namespace daisy;
using namespace daisysp;
using namespace dpt;

class Kali : public std::enable_shared_from_this<Kali>
{
public:
    DPT patch;

    KaliOscillator warble[9];
    // Clock oscillators removed - using PLL approach for outputs
    // KaliOscillator pansine[2];

    KaliFreezeEngine freezer;

    KaliDelayLine<float, MAX_DELAY> *delays[4];

    // Biquad biquad[2];
    DcBlock dc[4];
    ReverbSc reverb;

    // maybe refactor later, but goal is have meta2 values pull from midi controls
    KaliMIDI midi;
    bool Meta1MIDI = false;
    bool Meta2MIDI = true;

    bool midi_clock_flag = false;
    bool midi_activity = false;

    float diffpll;
    // CpuLoadMeter cpu;

    KaliClock masterclock;

    // PLL variables for phase-locked clock outputs
    int left_clock_phase_accumulator = 0;  // Tracks phase for left output
    int right_clock_phase_accumulator = 0; // Tracks phase for right output
    bool left_clock_state = false;         // Current left output state
    bool right_clock_state = false;        // Current right output state
    int left_gate_timer = 0;               // Gate pulse duration for left
    int right_gate_timer = 0;              // Gate pulse duration for right

    float frequencybase[2], samps[2];

    int8_t delaymap[4];
    bool feedback_toggle[4] = {true, true, true, true};

    int size;

    // KaliKnob KnobState[10];

    KaliInput inp;

    // These were for smooth transitions between submodes, not used anymore.
    int last_mode;

    // Add method to dynamically set delay range for external sync
    void UpdateDelayRangeForExternalSync();

    float MIN_DELAY_WORKING = 4.f;
    float MAX_DELAY_WORKING = MAX_DELAY;
    float slewtime = size * 16;

    // Smoothing for external sync range updates to reduce jitter
    float smoothed_samples_per_beat = 24000.0f; // Default 120 BPM
    float last_calculated_min = 4.f;
    float last_calculated_max = MAX_DELAY;

    /*
    float MAX_DELAY_WORKING_STEP = 100;
    unsigned int MAX_DELAY_WORKING_MAX_STEPS = 4;
    unsigned int MAX_DELAY_WORKING_CURRENTSTEP = 1;

    void ChangeWorkingDelay (unsigned int whichstep) {
        MAX_DELAY_WORKING_CURRENTSTEP = whichstep;
        MAX_DELAY_WORKING = MAX_DELAY / (DSY_CLAMP(whichstep + 1, 1, MAX_DELAY_WORKING_MAX_STEPS) * MAX_DELAY_WORKING_STEP);
    }
    */

    bool isfrozen = false;

    // used with ParameterInterpolators
    // weird ass names - boop is the left delay time, koko is the right delay time
    float boop_basic, koko_basic, modmult, meta, meta2, warbfreq, warbamp, damp, time;
    float delaytimes[8]; // boop n koko - delay time l / r
    float delaytargets[4];

    // toggles
    bool lfo_unlink;
    // bool delay_unlink;

    // 0 - 255
    uint8_t turingprob;

    // start with a clear
    bool oled_dirty = true;

    enum Modes
    {
        Straight,
        Parvati,
        KaliMa,
        ExtLoop,
        /*OnlyReverb,*/
        LAST
    };

    enum LFOModes
    {
        SyncStraight,
        SyncSH,
        SyncTH,
        RandReset,
        RandShape,
        Turing,
        Jitter,
        Wavetable,
        // WavetableQuantized,
        UnlinkedStraight,
        Glacier,
        Polythene,
        FeedbackFollower,
        Sidechain,
        Envelope,
        Raw,
        LAST_LFO
    };

    enum FilterModes
    {
        FIR,
        SVF,
        OFF,
        FIR_FRONT,
        LAST_FILT
    };

    enum CV
    {
        META1,
        META2,
        LFO_RATE,
        L_TIME,
        R_TIME,
        CUTOFF,
        FEEDBACK,
        MIX,
        LFO_ADJUST,
        DELAY_ADJUST
    };

    int mode = Kali::Modes::Straight;
    int lfomode = Kali::LFOModes::SyncStraight;
    int filtmode = Kali::FilterModes::FIR;
    int lfotarget = 0;

    float prevgate1, prevgate2, mix;
    bool prevbut1, prevbut2, prevbut3, prevbut4;

    // Track if encoders were held (to prevent click-on-release after hold)
    bool e1_was_held = false;
    bool e2_was_held = false;

    float delchop = 1.f;

    int displaycountdown_max = 5000.0f;
    int displaycountdown = displaycountdown_max;

    CrossFade cf;

    KaliDSP dsp;
    // KaliH949DSP pdsp;

    /* a few things for reference on oled */
    float last_sample_in[4];
    float last_sample_out[4];
    float new_normal;
    float ppqn[4];

    float global_meta_multiplier = 1.0f;

    bool in_audio_callback = false;

    static const unsigned int POPUP_MESSAGE_LENGTH = 32;
    char *popupmessage;
    unsigned int popupcountdown = 200;

    void popup(char *popupmessage_)
    {
        popupmessage = popupmessage_;
    }

    // float apbuf[2][512];
    // Allpass ap[2];

    EnvelopeFollower<2, float> Follower;

    float damp_scaled = 0.5;

    float splat = 0.0;

    bool audioCallbackDone;

    // We set a flag when we want to save so it does not interrupt the audio callback,
    // since control states like encoder presses handled at the beginning of the audio callback.

    bool saveSettings;
    bool initSettings;
    bool saveLFOPreset;
    bool loadLFOPreset;
    bool saveDSPPreset;
    bool loadDSPPreset;

    void Init(float);

    void SetJit(float);

    unsigned int tick;
    float last_gate;
    int gate_count;
    unsigned int sync_trig_count;

    bool TriggerReceived;

    bool queue_midi_clock;

    bool periodically_sync;

    KaliEditState editstate;

    EnvelopeFollower<2, float> follower;

    KaliPlayheadEngine playhead_engine;

    void HitTick(size_t blocksize);

    float GateSpaceTime(size_t blocksize);

    void SetMode(uint8_t);

    void FlipLFOMode(bool inc);

    void SetLFOMode(int8_t, uint8_t);

    void HandleMIDI();

    void HandleCallbackSync();
    void HandleSyncTrigger();
    void HandleMIDIClock();

    float GetGainDb()
    {
        // FIXME: Re-implement this.
        return 215;
        // return (options.opt_global[OptionsPages::AdcAttenuation]->Value - 215) / 2.0f;
        // 215 = 0db
    }

    static inline bool HasChanged(float a, float b, float hys)
    {
        return ((a > b) ? a - b : b - a) > hys;
    }

    DelayPhasor acidburn[4];

    void ResetClocks()
    {
        masterclock.SampleAccumulator = 0;
        masterclock.TriggerAccumulator = 0;
    }

    void ResetAllThings()
    {
        ResetClocks();
        for (int i = 0; i < 6; i++)
        {
            warble[i].Reset();
            warble[i].cn.Sync();
        }
    }
    void SetLfoOption(int index, KaliLFOOptions *opt);

    // This is for MIDI monitoring

    MidiEvent MIDIEvents[5];
    unsigned int MIDIEventsIndex = 0;
    unsigned int MIDILastEventIndex = 0;

    unsigned int AddMidiEvent(MidiEvent e)
    {
        MIDIEvents[MIDIEventsIndex] = e;
        MIDILastEventIndex = MIDIEventsIndex;
        MIDIEventsIndex = (MIDIEventsIndex + 1) % 5;
        return MIDIEventsIndex;
    }

    MidiEvent GetLastMIDIEvent()
    {
        return MIDIEvents[MIDILastEventIndex];
    }

    void updateoled(void *data);
    // TODO: Move this to KaliInput?

    void UpdateVuMeters();
    void DrawActivityIndicators();
    void PrintToScreen(const daisy::Rectangle screen, const Alignment alignment, const FontDef f, bool on, const char *format, ...);
    void PrintLfoInfo(const daisy::Rectangle &screen, KaliOscillator *curr_lfo, KaliOption *option);
    void DrawLfoWaveform(KaliOscillator *curr_lfo);
    void UpdateLfoEditScreen(const daisy::Rectangle &screen);
    void UpdateOptionsScreen(const daisy::Rectangle &screen);
    void UpdateDspEditScreen(const daisy::Rectangle &screen);
#ifndef NDEBUG
    void UpdateDebugScreen(const daisy::Rectangle &screen);
    void UpdateMIDIScreen(const daisy::Rectangle &screen);
    void UpdateClockDebugScreen(const daisy::Rectangle &);
#endif
    void UpdatePresetsScreen(const daisy::Rectangle &screen);
    // Preset page state
    bool preset_save_mode = false; // false = Load, true = Save
    char last_preset_name[16] = {0};
    void PrintFormattedValue(const daisy::Rectangle &screen, float value, StringTableType stType, bool is_float);
    void PrintEditableValueBig(const daisy::Rectangle &screen, float value, StringTableType stType, bool is_float);
    void HandleEncoders(KaliInput *inp);
    // Optional wavetable editor integration
#if ENABLE_WAVETABLE_EDITOR
    class WavetableStepEditor *wavetable_editor_ptr = nullptr;
#endif
    int32_t e1inc;
    int32_t e2inc;

    KaliState *currentState = &KaliConfigOscState::getInstance();

    // Array of state pointers indexed by major mode

    KaliState *statesByMode[KaliEditState::LAST_PAGE];

    void InitStateMap()
    {
        statesByMode[KaliEditState::LFO_EDIT] = &KaliConfigOscState::getInstance();
        statesByMode[KaliEditState::OPTIONS] = &KaliConfigState::getInstance();
        statesByMode[KaliEditState::DSP_EDIT] = &KaliDSPConfigState::getInstance();
        statesByMode[KaliEditState::PRESETS] = &KaliPresetsState::getInstance();
        statesByMode[KaliEditState::MIDI_MODE] = &KaliMIDIState::getInstance();
        statesByMode[KaliEditState::DEBUG_MODE] = &KaliDebugState::getInstance();
        statesByMode[KaliEditState::CLOCK_DEBUG_MODE] = &KaliClockDebugState::getInstance();
    }

    void SetStateByInt(int newstate)
    {
        // bounds chk

        bool DEBUG_MODE = GetDebugMode();

        // if (newstate < 0)
        //     newstate = KaliEditState::LAST_PAGE - 1;
        // else if (newstate > KaliEditState::LAST_PAGE - 1)
        //     newstate = 0;

        // Skip debug type panels if it's disabled
        if (!DEBUG_MODE)
        {
            newstate = DSY_CLAMP(newstate, 0, KaliEditState::DSP_EDIT); // kinda awkward looking, whatever
        }
        else
        {
            newstate = DSY_CLAMP(newstate, 0, KaliEditState::LAST_PAGE - 1);
        }

        // if (!DEBUG_MODE && (newstate == KaliEditState::MIDI_MODE || newstate == KaliEditState::DEBUG_MODE || newstate == KaliEditState::CLOCK_DEBUG))
        //{
        // } else {
        // }

        // null pointer no no no way
        editstate.ClearModesAndSetMajor(newstate);

        // Set state directly using the array
        setState(*Kali::statesByMode[newstate]);
    }

    void setState(KaliState &newState)
    {
        currentState->exit(this);
        currentState = &newState;
        currentState->enter(this);
    }

    void handle_event(int event)
    {
        displaycountdown = displaycountdown_max;
        currentState->handle_event(this, event);
    }

    inline KaliState *getCurrentState() const { return currentState; }
    KaliState *getStateForMajorMode()
    {
        switch (editstate.major_mode)
        {
        default:
            return &KaliConfigOscState::getInstance();
        }
    }

    // TODO: FINISH
    void rotate(int howmany = 2)
    {
        int max_map = 4, index;
        for (int i = 0; i < max_map; i++)
        {
            index = (i + howmany) % 4;
            feedback_toggle[index] = false;
            delaymap[i] = delaymap[index];
            delaymap[i] += howmany;
        }
    }

    bool GetDebugMode()
    {
        return GetValue(OptionsPages::EnableDebugInfo) > 0.5f;
    }

    void SetDebugMode(bool d)
    {
        SetOptionValue(OptionsPages::EnableDebugInfo, d ? 1.0f : 0.0f);
    }

    KaliPlayheadEngine kpe;

    // void SaveOptions() {
    // uint32_t config_version = CONFIG_VERSION;

    //    uint32_t address_offset_ = 0;
    //    address_offset_ = 0x00 & (uint32_t)(~0xff);
    //    patch.qspi.Erase(address_offset_, sizeof(KaliOptions));

    /*
    KaliOptions tmpopt;
    */

    /*
    memcpy(tmpopt.LfoOptions, kali.options.LfoOptions, sizeof(MCLFOOptions) * 12);
    memcpy(tmpopt.Stacks, mc.options.SavedPresets, 8 * sizeof(MCPresetStack));

    //mc.patch.qspi.Write(address_offset_, sizeof(uint32_t), (uint8_t *)&config_version);
    mc.patch.qspi.Write(address_offset_, sizeof(MCOptions), (uint8_t *)&tmpopt);
    */
    //}

    /*
    void LoadOptions(bool init = true)
    {
        patch.ProcessAllControls();
        patch.e1.Debounce();
        patch.e2.Debounce();

        bool reset_options = false; // mc.patch.e1.RisingEdge() && mc.patch.e2.Pressed();

        uint32_t address_offset_ = 0;
        address_offset_ = 0x00 & (uint32_t)(~0xff);
        // uint32_t savedconfigver = reinterpret_cast<uint32_t>(mc.patch.qspi.GetData(address_offset_));
        // address_offset_ += sizeof(uint32_t);
        KaliOptions *data = reinterpret_cast<KaliOptions *>(patch.qspi.GetData(address_offset_));

        // KaliLFOOptions options[12];

        // memcpy(options, data->LfoOptions, 12 * sizeof(KaliLFOOptions));

        if (data->Version > 0)
        {
            for (int i = 0; i < 9; i++)
            {
                for (int j = 0; j < 64; j++)
                {
                    warble[i].preset.SetOption(j, data->LfoOptions[i][j]);
                }
            }
        }
    }
    */

    // uint16_t SelectedIndex = 0;
    // uint16_t SelectedOptionIndex = 0;

    void SetSelected(int index_)
    {
        editstate.SelectedIndex = index_;
    }

    void SetSelectedOption(int index_)
    {
        editstate.SelectedOptionIndex = index_;
    }

    void SetSelectedByEncoder(int enc)
    {
        // find length of current bank
        uint16_t banklength = GetBankLength(editstate.SelectedBank);

        if (enc != 0)
            editstate.SelectedBank = DSY_CLAMP(editstate.SelectedBank + enc, 0, banklength /*MAX_LFOS - 1*/);
    }

    uint16_t GetBankLength(int whichbank)
    {
        uint16_t banklength;

        switch (whichbank)
        {
        case BankType::Global:
            banklength = OptionsPages::KALI_OPTIONS_LAST;
            break;
        case BankType::LFO:
            banklength = LFOOptionsPages::KALI_LFO_OPTIONS_LAST;
            break;
        case BankType::DSP:
            banklength = DSPOptionsPages::KALI_DSP_OPTIONS_LAST;
            break;
        default:
            banklength = 0;
            break;
        }

        return banklength;
    }

    void SetSelectedOptionByEncoder(int enc, int bank = BankType::Global)
    {
        uint16_t banklength = GetBankLength(bank) - 1;

        if (enc != 0)
        {
            if (editstate.SelectedOptionIndex + enc < 0)
                editstate.SelectedOptionIndex = banklength;
            else if (editstate.SelectedOptionIndex + enc > banklength)
                editstate.SelectedOptionIndex = 0;
            else
                editstate.SelectedOptionIndex = DSY_CLAMP(editstate.SelectedOptionIndex + enc, 0, banklength);
        }
    }

    void ProcessEncoder(int inc)
    {
        KaliOption *o = GetSelectedOption();
        // o->Value = DSY_CLAMP(o->Value + inc, o->Min, o->Max);

        SetOptionValue(editstate.SelectedIndex, o->Value + inc);
    }

    void SetOptionValue(uint16_t which, float value, int bank = BankType::Global)
    {
        KaliOption *o = OptionRules[bank][which];
        o->Value = DSY_CLAMP(value, o->Min, o->Max);
    }

    void SetOptionValueByEncoder(int16_t which, int inc, int bank = BankType::Global)
    {
        if (inc != 0)
        {
            KaliOption *o = OptionRules[bank][which];
            o->Value = DSY_CLAMP(o->Value + (inc * o->Step), o->Min, o->Max);
        }
    }

    float GetSelectedOptionValue(int whichselected, int which)
    {
        const KaliOption *o = GetSelectedOption();
        return o->Value;
    }

    float GetValue(uint16_t index, int bank = BankType::Global, uint16_t subindex = 0)
    {
        switch (bank)
        {
        case BankType::Global:
            return OptionRules[bank][index]->Value;
        case BankType::LFO:
            return warble[index].preset.GetOption(subindex);
        case BankType::DSP:
            return OptionRules[BankType::DSP][index]->Value;
        default:
            return 0.0f;
        }
    }

    KaliOption *GetSelectedOption()
    {
        return GetSelectedOption(editstate.SelectedIndex);
    }

    KaliOption *GetSelectedOption(uint16_t which, int bank = BankType::Global)
    {
        return OptionRules[bank][which];
    }

    const char *GetStringFromTable(StringTableType stt, unsigned int which)
    {
        if (stt >= STRING_TABLE_TYPE_LAST)
        {
            return "ERR_ST";
        }
        if (which > MAX_STRING_TABLE_COUNT)
        {
            return "ERR_STVAL";
        }
        return string_tables[stt][which];
    }

    bool CheckFlag(uint16_t index, int bank = BankType::Global)
    {
        return (OptionRules[bank][index]->Value == 1);
    }
    KaliOptions *data; // data in memory
    KaliOptions options;
    void SaveOptions();
    void SaveLFOPreset(int, int);
    void LoadLFOPreset(int, int);
    void SaveDSPPreset();
    void LoadOptions(bool init);
    // Combined main preset (6 LFOs + 1 DSP) stored in QSPI
    void SaveMainPreset(int slot);
    bool LoadMainPreset(int slot);
    bool ReadMainPresetMeta(int slot, char *name_out, bool &valid);
    void SaveLastSelectedPresetSlot(int slot);
    int LoadLastSelectedPresetSlot();

    uint8_t max_notes = 2;
    uint8_t notes_pos = 0;
    uint8_t notes[127];
    uint8_t notes_active[2];

    bool IsUnlinked();

    /*
    const std::map<DSPModes, std::pair<Kali::Modes, int>> dspModeMappings = {
        {DSPModes::Chorus, {Kali::Modes::Parvati, ParvatiModeDSP::ParvatiModeSubmodes::Chorus}},
        {DSPModes::Parvati, {Kali::Modes::Parvati, ParvatiModeDSP::ParvatiModeSubmodes::Parvati}},
        {DSPModes::Shards, {Kali::Modes::Parvati, ParvatiModeDSP::ParvatiModeSubmodes::Shards}},
        {DSPModes::Knives, {Kali::Modes::Parvati, ParvatiModeDSP::ParvatiModeSubmodes::Knives}},
        {DSPModes::Splat, {Kali::Modes::Parvati, ParvatiModeDSP::ParvatiModeSubmodes::Splat}},
        {DSPModes::Gramma, {Kali::Modes::Parvati, ParvatiModeDSP::ParvatiModeSubmodes::Gramma}},
        {DSPModes::Grampa, {Kali::Modes::Parvati, ParvatiModeDSP::ParvatiModeSubmodes::Grampa}},
        {DSPModes::StraightLinked, {Kali::Modes::Straight, StraightDSP::StraightDSPSubmodes::StraightLinked}},
        {DSPModes::StraightUnlinked, {Kali::Modes::Straight, StraightDSP::StraightDSPSubmodes::StraightUnlinked}},
        {DSPModes::PingPongLinked, {Kali::Modes::Straight, StraightDSP::StraightDSPSubmodes::PingPongLinked}},
        {DSPModes::PingPongUnlinked, {Kali::Modes::Straight, StraightDSP::StraightDSPSubmodes::PingPongUnlinked}},
        {DSPModes::Resonate, {Kali::Modes::Straight, StraightDSP::StraightDSPSubmodes::Resonator}},
        {DSPModes::SimpleChorus, {Kali::Modes::Straight, StraightDSP::StraightDSPSubmodes::SimpleChorus}},
        {DSPModes::Sin, {Kali::Modes::KaliMa, KaliModeDSP::KaliModeSubmodes::Sin}},
        {DSPModes::Saturate, {Kali::Modes::KaliMa, KaliModeDSP::KaliModeSubmodes::Saturate}},
        {DSPModes::Diode, {Kali::Modes::KaliMa, KaliModeDSP::KaliModeSubmodes::Diode}},
        {DSPModes::Foldback, {Kali::Modes::KaliMa, KaliModeDSP::KaliModeSubmodes::Foldback}},
        {DSPModes::Modulo, {Kali::Modes::KaliMa, KaliModeDSP::KaliModeSubmodes::Modulo}},
        {DSPModes::OnlyReverb, {Kali::Modes::OnlyReverb, StraightDSP::StraightDSPSubmodes::StraightLinked}},
    };
    */
    /*
     void ApplyDistortion(float* dry, float knob8, float knob9, int algo, int amount, bool scale = false) {
         if (amount <= 0) {
             return;
         }

         float A = 1.f + (amount * TWOPI_F * 0.01f);
         float B = amount * 0.01f;

         float gain, threshold;

         switch (algo) {
         case 0: // Sine Distortion
             dry[0] = sinf(dry[0] * A) * 0.25f;
             dry[1] = sinf(dry[1] * A) * 0.25f;
             break;

         case 1: // Foldback Distortion
             gain = fmap(amount * 0.01f, 1.f, 3.f, daisysp::Mapping::LOG);
             threshold = fmap(A, 8.0f, 1.0f);
             dry[0] = KaliModeDSP::foldback(dry[0] * gain, threshold) * (scale ? 0.75f : 1.0f);
             dry[1] = KaliModeDSP::foldback(dry[1] * gain, threshold) * (scale ? 0.75f : 1.0f);
             break;

         case 2: // Tanh Distortion
             dry[0] = tanhf(dry[0] * (1.f * amount));
             dry[1] = tanhf(dry[1] * (1.f * amount));
             break;

         case 3: // Quantizer
             dry[0] = KaliModeDSP::quantizer(dry[0], 1.0f + (amount * 0.01f), B) * (scale ? 0.25f : 1.0f);
             dry[1] = KaliModeDSP::quantizer(dry[1], 1.0f + (amount * 0.01f), B) * (scale ? 0.25f : 1.0f);
             break;

         case 4: // Diode Clipper
             gain = fmap(amount * 0.01f, 1.f, 2.f, daisysp::Mapping::LOG);
             threshold = fmap(amount * 0.01f, 0.05f, 0.5f);
             dry[0] = KaliModeDSP::diode(dry[0] * gain, threshold) * (scale ? 0.25f : 1.0f);
             dry[1] = KaliModeDSP::diode(dry[1] * gain, threshold) * (scale ? 0.25f : 1.0f);
             break;

         case 5: // Modulo Distortion
             dry[0] = KaliModeDSP::modulo(dry[0] * gain, A, B);
             dry[1] = KaliModeDSP::modulo(dry[1] * gain, A, B);
             break;

         case 6: // Hard Clip
             dry[0] = fminf(fmaxf(dry[0], - amount * 0.01f), amount * 0.01f);
             dry[1] = fminf(fmaxf(dry[1], - amount * 0.01f), amount * 0.01f);
             break;

         case 7: // Soft Clip
             dry[0] = dry[0] / (1.f + fabsf(dry[0] * amount * 0.01f));
             dry[1] = dry[1] / (1.f + fabsf(dry[1] * amount * 0.01f));
             break;

         case 8: // Asymmetric Clipping
             dry[0] = (dry[0] > 0.f) ? tanhf(dry[0] * A) : dry[0] * B;
             dry[1] = (dry[1] > 0.f) ? tanhf(dry[1] * A) : dry[1] * B;
             break;
         }
     }
 };
 */

    float ApplyDistortion(float *dry, float knob8, float knob9, int algo, int amount, bool scale = false)
    {
        if (amount <= 0)
        {
            return 1.0f;
        }

        float A = 1.f + (amount * TWOPI_F * 0.01f);
        float B = amount * 0.01f;
        float C = 1.f + (amount * 0.01f);
        float gain = 0.0f, threshold = 0.0f;
        float feedback_gain = 1.f;

        for (int i = 0; i < 2; ++i)
        {
            switch (algo)
            {
            case 0: // Sine Distortion
                dry[i] = sinf(dry[i] * A * 1.5f);
                feedback_gain = 1.0f;
                break;

            case 1: // Foldback Distortion
                gain = fmap(B, 1.f, 10.f, daisysp::Mapping::LINEAR);
                threshold = fmap(B, 8.0f, 1.0f);
                dry[i] = KaliDSP::foldback(dry[i] * gain, threshold);
                feedback_gain = 1.0f; // 0.9f;
                break;

            case 2: // Tanh Distortion
                dry[i] = tanhf(dry[i] * A * 2.0f);
                feedback_gain = 1.0f;
                break;

            case 3: // Quantizer
                gain = fmap(B, 1.f, 3.f, daisysp::Mapping::LINEAR);
                dry[i] = KaliDSP::quantizer(dry[i] * gain, A * 2.0f, B * 2.0f);
                feedback_gain = 1.0f;
                break;

            case 4: // Diode Clipper
                gain = fmap(B, 1.f, 2.f, daisysp::Mapping::LINEAR);
                threshold = 1.f - fmap(B, 0.25f, 1.0f);
                dry[i] = KaliDSP::diode(dry[i] * gain, DSY_CLAMP(threshold, 0.f, 1.f));
                feedback_gain = 1.0f;
                break;

            case 5: // Hard Clip
                gain = fmap(B, 1.f, 10.0f, daisysp::Mapping::LINEAR);
                dry[i] *= gain;
                dry[i] = (dry[i] < -1.0) ? -1.0f : (dry[i] > 1.0 ? 1.0 : dry[i]);
                feedback_gain = 1.0f;
                break;

            case 6: // Soft Clip
                gain = fmap(B, 1.f, 10.f, daisysp::Mapping::LINEAR);
                dry[i] *= gain;
                dry[i] = dry[i] / (1.f + fabsf(dry[i]));
                feedback_gain = 1.0f;
                break;

            case 7: // Asymmetric Clipping
                gain = fmap(B, 1.f, 5.f, daisysp::Mapping::LINEAR);
                dry[i] *= gain;
                dry[i] = (dry[i] > 0.f) ? tanhf(dry[i] * A) : dry[i] * B;
                feedback_gain = 1.0f;
                break;

            case 8: // Modulo Distortion
                gain = fmap(B, 1.f, 5.f, daisysp::Mapping::LINEAR);
                dry[i] *= gain;
                dry[i] = KaliDSP::modulo(dry[i] * gain, A, B);
                break;

            default:
                feedback_gain = 1.0f;
                break;
            }
            float trim = GetValue(DSPOptionsPages::DistortionTrim, BankType::DSP);
            dry[i] *= trim;
        }

        return feedback_gain;
    }
};
#endif
