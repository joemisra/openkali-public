#pragma once
#ifndef KALIINPUT_H
#define KALIINPUT_H
#endif

#include "ParameterInterpolator.h"
#include "dpt/daisy_dpt.h"
#include "daisy.h"
#include "daisysp.h"
#include "KaliOptions.h"
#include "KaliWavetable.h"

using namespace daisy;
using namespace daisysp;
using namespace dpt;

class KaliOption;

class KaliKnob
{
public:
    float _Value = 0.0;
    float _Last = 0.0;
    float Offset = 0.0;
    float Scale = 1.0;
    bool AllowBipolar = false;
    bool Polarity;
    bool EnableWavetable = false;
    float Hysteresis = 0.001f;
    float increment_;
    float _LastInterpolated;
    float _Interpolated;
    KaliOption *opt;
    KaliWavetable wavetable;

    bool Lock = false;
    size_t size = 96;
    int slew = 1;

    // Has this ever changed?
    bool Touched = false;
    // Did this change last update?
    bool Changed = false;
    // Use Hysteresis?
    bool UseHysteresis = true;

    // Soft takeover / pickup mode
    bool SoftTakeoverEnabled = false;
    float SoftTakeoverTarget = 0.0f; // Value knob needs to reach
    bool PickedUp = true;            // Has knob crossed target since preset load?

    float sizetimesslewrecip = 1.0f / (float)(size * slew);

    // Enable soft takeover mode with target value
    void EnableSoftTakeover(float target)
    {
        SoftTakeoverEnabled = true;
        SoftTakeoverTarget = target;
        PickedUp = false;
    }

    // Disable soft takeover mode
    void DisableSoftTakeover()
    {
        SoftTakeoverEnabled = false;
        PickedUp = true;
    }

    void UpdateValue(float value)
    {
        Touched = true;

        // Soft takeover logic: check if knob has crossed the target value
        if (SoftTakeoverEnabled && !PickedUp)
        {
            // Check if we've crossed the target (with small tolerance)
            float tolerance = 0.02f; // 2% tolerance
            if ((_Last < SoftTakeoverTarget && value >= SoftTakeoverTarget - tolerance) ||
                (_Last > SoftTakeoverTarget && value <= SoftTakeoverTarget + tolerance) ||
                (value >= SoftTakeoverTarget - tolerance && value <= SoftTakeoverTarget + tolerance))
            {
                PickedUp = true;
                SoftTakeoverEnabled = false; // Disable once picked up
            }
            else
            {
                // Don't update actual value until picked up
                _Last = value;
                return;
            }
        }

        if (HasChanged(value, Hysteresis))
        {
            _LastInterpolated = _Interpolated;
            _Last = _Value;
            _Value = value;
            _Interpolated = _LastInterpolated;
            Polarity = value < 0.0f;
            increment_ = (_Value - _Last) * sizetimesslewrecip;
        }
    }

    bool HasChanged(float b, float hys)
    {
        float a = _Value;
        Changed = ((a > b) ? a - b : b - a) > hys;
        return Changed;
    }

    float Next()
    {
        _Interpolated += increment_;

        if ((increment_ < 0.0f) && (_Interpolated <= _Value))
        {
            _Interpolated = _Value;
            increment_ = 0.0f;
        }
        if ((increment_ > 0.0f) && (_Interpolated >= _Value))
        {
            _Interpolated = _Value;
            increment_ = 0.0f;
        }

        return _Interpolated;
    }

    float Value()
    {
        if (EnableWavetable)
            return wavetable.ReadWavetableLERP(_Value);
        else
            return _Value;
    }

    float ValueO()
    {
        return ValueMappedByOption();
    }

    float ValueMappedByOption()
    {
        return daisysp::fmap(_Value, opt->Min, opt->Max);
    }

    float ValueM(float min, float max)
    {
        return daisysp::fmap(_Value, min, max);
    }

    float ValueMapped(float min, float max)
    {
        return daisysp::fmap(_Value, min, max);
    }

    ParameterInterpolator GetInterpolator(size_t size)
    {
        ParameterInterpolator interpolator(&_Last, _Value, size);
        return interpolator;
    }
};

class KaliInput
{
public:
    float Meta;
    float MetaMapped;
    float Meta2;
    float Meta2Mapped;
    float LFORate;
    float LFORateMapped;
    float TimeL;
    float TimeLMapped;

    float TimeR;
    float TimeRMapped;
    float Feedback;
    float FeedbackMapped;
    float Cutoff;
    float CutoffMapped;
    float Mix;
    float MixMapped;
    float LFOAdjust;
    float DelayAdjust;

    float fm = 0.0;
    float am = 0.0;

    bool Gate[2];
    bool buttons[4];

    KaliKnob Knobs[10];

    void ProcessControls(DPT *patch);
};