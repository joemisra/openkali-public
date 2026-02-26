#pragma once
#ifndef KALIOPTIONS_H
#define KALIOPTIONS_H
#endif

// Feature flags - must match KaliDsp.h
#define ENABLE_FFT_BLUR 0 // Disables SpectralBlur DSP mode
// Temporary compliance hold: disable reverb return path until a licensing-safe replacement is in place.
#define ENABLE_REVERB_RETURN 0

#include "dpt/daisy_dpt.h"
#include "daisy.h"
#include "daisysp.h"
#include "sys/system.h"
#include "stringtables.h"
#include "KaliConfig.h"
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include <unordered_map>
#include "dpt/daisy_dpt.h"

using namespace daisy;
using namespace daisy::dpt;

/* NOTES
KaliOption is a class that represents a single option in the Kali UI, for global options, dsp options, and also for the LFOs.

All values are stored as floats (this is a change from how it worked in the past)

*/

const int MOD_SOURCES_MAX = 18;

enum OptionsPages
{
    LfoRateMultiplier,
    LeftClockRateMultiplier,
    RightClockRateMultiplier,
    ExternalCvClockPPQN,
    SyncEngine,
    // PeriodicSync,
    SyncButtonMode,
    FreezeButtonMode,
    // GlobalLink,
    // UseFilter,
    UseAllpass,
    // SwapDelayAdj,
    ReverbWetSend,
    ReverbDrySend,
    ReverbFeedback,
    ReverbDamp,
    AdcAttenuation,
    DacAttenuation,
    InputWidth,
    // GlobalFollowerAttack,
    // GlobalFollowerRelease,
    InvertEncoders,
    EnableDebugInfo,
    GlobalPresetLoad,
    GlobalPresetSave,
    KALI_OPTIONS_LAST
};

// Delay range presets for different use cases
enum DelayRangePresets
{
    RANGE_PRECISION,    // 1-500ms - Fine detail work
    RANGE_STUDIO,       // 10ms-2s - Standard studio delays
    RANGE_AMBIENT,      // 50ms-8s - Ambient textures
    RANGE_LOOPER,       // 100ms-20s - Looping and long delays
    RANGE_EXPERIMENTAL, // 1ms-30s - Full range madness
    DELAY_RANGE_LAST
};

enum LFOOptionsPages
{
    Waveform,
    LFOMode,
    Oneshot,
    Meta,
    MetaDivider,
    Polarity,
    Adjust,
    Offset,
    Attenuate,
    PhaseOffset,
    GateIn,
    OffsetCal,
    AttenuateCal,
    Attack,
    Decay,
    Sustain,
    Release,
    Eschaton,
    Incandenza,
    FMSource,
    FMSourceAmount,
    AMSource,
    AMAmount,
    LFOPresetLoad,
    LFOPresetSave,
    KALI_LFO_OPTIONS_LAST
};

enum DSPOptionsPages
{
    Mode,
    P1,
    P2,
    P3,
    P4,
    Distortion,
    DistortionAmount,
    DistortionTarget,
    DistortionTrim,
    FilterType,
    DelayRangePreset, // Delay range preset selection
    DSPPresetLoad,
    DSPPresetSave,
    KALI_DSP_OPTIONS_LAST
};

enum DSPModes
{
    StraightLinked,
    PingPongLinked,
    StraightUnlinked,
    Reverse,
    Resonate,
    Chorus,
    Knuth,
    Granular,         // Original granular (±1 semitone on Meta1)
    GranularOctave,   // Wide pitch range (±12 semitones on Meta1)
    GranularTexture,  // Preset: Slow modulation, medium grain size
    GranularShimmer,  // Preset: Fast modulation, small grains, pitch up
    GranularCrystals, // Preset: Quantized time, no modulation
#if ENABLE_FFT_BLUR
    SpectralBlur, // FFT spectral blur/smear
#endif
    Fluid, // Navier–Stokes-inspired fluid modulation
    /*ShimmerGrain,
    SimpleChorus,
    Parvati,
    Knives,
    Shards,
    Splat,
    Grampa,
    Gramma,
    // Water,
    // Fire,
    /// LCR,
    // FirePong,
    // Dub,
    Sin,
    Saturate,
    Foldback,
    Diode,
    Modulo,
    Quantize,
    OnlyReverb,*/
    DSP_MODES_LAST
};

enum PresetPages
{
    LoadAll,
    SaveAll,
    LoadOne,
    SaveOne,
    PRESET_PAGES_LAST
};

class KaliOption
{
public:
    char Description[64];
    char ShortDescription[64];

    float Min = 0;
    float Max = 100;
    float Step;
    float DefaultValue = 0.0f;

    float Value;

    uint8_t Index = 0;

    float current_total_modulation;

    StringTableType STType = StringTableType::None;
    bool IsFloat = true;
    char Units;

    uint8_t Lock = 0;

    KaliOption(const char description[64], const char short_description[64], float min, float max, float step, float defaultvalue, StringTableType stt, bool isFloat, char units)
        : Min(min), Max(max), Step(step), DefaultValue(defaultvalue), STType(stt), IsFloat(isFloat), Units(units)
    {

        Value = DefaultValue;
        strncpy(Description, description, 64);
        strncpy(ShortDescription, short_description, 16);
    }

    KaliOption() {}

    void Reset()
    {
        Value = DefaultValue;
    }

    /**
     * @brief Processes the encoder input and updates a local value (does not update class Value).
     *
     * @param inc -1 = left turn, 1 = right turni, 0 = no change
     * @return The updated value after processing the encoder input.
     */

    float ProcessEncoderRelative(uint16_t value, int inc)
    {
        if (inc == 0)
            return value;

        value = DSY_CLAMP(value + (inc * Step), Min, Max);

        return value;
    }

    /**
     * @brief Processes the encoder input and updates Value accordingly (enforces bounds).
     *
     * @param inc -1 = left turn, 1 = right turni, 0 = no change
     * @return The updated value after processing the encoder input.
     */
    float ProcessEncoder(int inc)
    {
        if (inc == 0)
            return Value;

        Value = DSY_CLAMP(Value + (inc * Step), Min, Max);
        // Value = value;

        return Value;
    }

    /**
     * Sets the value of the option WITH bounds checking.
     *
     * @param value The value to set.
     * @return The updated value.
     */
    float Set(float value)
    {
        Value = DSY_CLAMP(value, Min, Max);
        ;
        return Value;
    }

    /**
     * Sets the value of the option WITHOUT bounds checking.
     *
     * @param value The value to set.
     */
    void SetUnsafe(float value)
    {
        Value = value;
    }
};

// Global, DSP, LFO. These are to be used as validators/filters.
extern KaliOption *OptionRules[3][32];

struct KaliPreset
{
    bool Initialized = false;
    char Name[64];

    float Options[32];

    // banktype, FIXME:
    int bt_ = 2;

    float GetOption(int which)
    {
        return Options[which];
    }

    float ResetOption(unsigned int index)
    {
        // reset to default
        if (index < LFOOptionsPages::KALI_LFO_OPTIONS_LAST)
        {
            Options[index] = OptionRules[bt_][index]->DefaultValue;
        }
        // OptionRules[BankType::LFO][index]->Set(LfoOptions[index]);
        return Options[index];
    }

    float SetOption(int index, float value)
    {
        if (index < LFOOptionsPages::KALI_LFO_OPTIONS_LAST)
            Options[index] = OptionRules[bt_][index]->Set(value);

        return Options[index];
    }

    float SetOptionByEncoder(int index, int inc)
    {
        if (inc != 0 && index < LFOOptionsPages::KALI_LFO_OPTIONS_LAST)
        {
            // int step = OptionRules[bt_][index]->Step;
            Options[index] = OptionRules[bt_][index]->Set(Options[index] + inc);
        }

        return Options[index];
    }

    float SetOptionUnsafe(int index, float value)
    {
        if (index < 20)
            Options[index] = value;

        return value;
    }
};

struct KaliPresetStack
{
    char BankName[64];
    KaliPreset LFOPresets[12];
};

struct KaliLFOOptions
{
    /*
    int waveform = 0;
    int mode = 0;
    int meta = 1;
    int meta_divider = 1;
    int bipolar = 1;
    uint16_t lfo_adjust = 5000; // 0.00 to 100.00
    float offset = 0.f;
    float attenuate = 100.0f;
    float offset_cal = 0.f;
    float attenuate_cal = 100.0f;
    */

    // note the above should eventually be stored in the preset
    KaliPreset Preset;

    bool operator==(const KaliLFOOptions &rhs)
    {
        // Compare the entire struct at once using std::memcmp
        return std::memcmp(this, &rhs, sizeof(*this)) == 0;
    }
    bool operator!=(const KaliLFOOptions &rhs) { return !operator==(rhs); }
};

// TODO: This needs to be the layout for what we store in memory.
struct KaliOptions
{
    float Version;

    float GlobalOptions[32];  // Current State
    float LFOOptions[10][32]; // Current State
    float DSPOptions[32];     // Current State
    float ClockOptions[32];   // Current State

    KaliPreset GlobalPresets[32]; // Preset States
    KaliPreset LFOPresets[32];    // Preset States
    KaliPreset DSPPresets[32];    // Preset States
    KaliPreset ClockPresets[32];  // Preset States

    // float Presets[32][32];

    bool operator==(const KaliOptions &rhs)
    {
        // Compare the entire struct at once using std::memcmp
        return std::memcmp(this, &rhs, sizeof(*this)) == 0;
    }

    bool operator!=(const KaliOptions &rhs) { return !operator==(rhs); }
};

class KaliOptionsManager
{
public:
    daisy::QSPIHandle qspi;

    // References to this exist
    // KaliLFOOptions LfoOptions[8];

    void Init(daisy::QSPIHandle qspi_, bool reset);

    void Save();
    void Reset();
};
