#include "dpt/daisy_dpt.h"
#include "Kali.h"
#include "KaliOptions.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;
using namespace dpt;

#define CONFIG_VERSION 0xFF01

KaliOption *OptionRules[3][32] = {
    {
        // KaliOption(const char description[64], const char short_description[64], float min, float max, float step, float defaultvalue, StringTableType stt, bool isFloat, char units) : Min(min), Max(max), Step(step), DefaultValue(defaultvalue), STType(stt), IsFloat(isFloat), Units(units)
        new KaliOption("LFO Multiplier", "LFO Mult", 1, 255, 1, 24, StringTableType::None, false, ' '),
        new KaliOption("Clock Out PPQN", "Out PPQN", 2, 127, 2, 24, StringTableType::None, false, ' '), // Renamed to clarify purpose
        new KaliOption("R Time Divider Lx / x", "R Clock Div", 1, 16, 1, 1, StringTableType::None, false, ' '),
        new KaliOption("External CV Clock PPQN", "CV PPQN", 1, 127, 1, 1, StringTableType::None, false, ' '), // Fixed: Default to 1 for typical CV clocks
        new KaliOption("Sync Mode", "Sync", 0, 2, 1, 0, StringTableType::STSyncMode, false, ' '),
        /*
        new KaliOption("Use PLL(TODO)", "PLL(TODO)", 0, 1, 1, 0, StringTableType::None, false, ' '),
        */
        new KaliOption("Sync Btn (1 = Reset)", "Sync/Rst", 0, 1, 1, 0, StringTableType::None, false, ' '),
        new KaliOption("Freeze Btn (1 = Reset)", "Frze/Rst", 0, 1, 1, 01, StringTableType::None, false, ' '),
        // new KaliOption("Global Sync Link", "GlobalLnk", 0, 1, 1, 0, StringTableType::None, false, ' '),
        // new KaliOption("Toggle Filter", "Filter?", 0, 1, 1, 1, StringTableType::None, false, ' '),
        new KaliOption("Toggle Allpass", "Allpass?", 0, 1, 1, 1, StringTableType::None, false, ' '),
        // new KaliOption("Swap Delay Adjust Function", "SwpDelAdj", 0, 1, 1, 0, StringTableType::None, false, ' '),
        new KaliOption("Reverb Wet Send", "RvWSend", 0, 50, 5, 0, StringTableType::None, false, ' '),
        new KaliOption("Reverb Dry Send", "RvDSend", 0, 50, 5, 0, StringTableType::None, false, ' '),
        new KaliOption("Reverb Feedback", "RvFeedBk", 50, 99, 1, 85, StringTableType::None, false, ' '),
        new KaliOption("Reverb Damp (Hz)", "RvDampHz", 1000, 19000, 250, 7500, StringTableType::None, false, ' '),
        new KaliOption("Input Gain", "InGain", 0, 255, 1, 215, StringTableType::None, false, ' '),
        new KaliOption("Output Gain", "OutGain", 0, 255, 1, 255, StringTableType::None, false, ' '),
        new KaliOption("Input Width", "InWidth", 0.f, 1.f, 1.f, 1.f, StringTableType::None, true, ' '),
        // new KaliOption("Env Follower Attack", "FolloAtt", 0, 250, 5, 5, StringTableType::None, false, ' '),
        // new KaliOption("Env Follower Release", "FolloRel", 0, 250, 5, 100, StringTableType::None, false, ' '),
        new KaliOption("Invert Encoders", "InvertEnc", 0, 1, 1, 0, StringTableType::None, false, ' '), // 1 if using bournes encoders
        new KaliOption("Enable Debug Info", "Debug", 0, 1, 1, 1, StringTableType::None, false, ' '),   // show or don't show debug stuff
        new KaliOption("Load Preset", "LoadPset", 0, 32, 1, 0, StringTableType::None, false, ' '),
        new KaliOption("Save Preset", "SavePset", 0, 32, 1, 0, StringTableType::None, false, ' '),
    },
    {
        // Expose full DSP mode range so we can select Granular/Shimmer, names from STDSPModeNames
        new KaliOption("DSP Mode", "DSP Mode", 0, DSPModes::DSP_MODES_LAST - 1, 1, 1, StringTableType::STDSPModeNames, false, ' '),
        new KaliOption("Parameter 1", "P1", 0, 100, 1, 0, StringTableType::None, false, ' '),
        new KaliOption("Parameter 2", "P2", 0, 100, 1, 0, StringTableType::None, false, ' '),
        new KaliOption("Parameter 3", "P3", 0, 100, 1, 0, StringTableType::None, false, ' '),
        new KaliOption("Parameter 4", "P4", 0, 100, 1, 0, StringTableType::None, false, ' '),
        new KaliOption("Distort Algo", "DistAlg", 0, 7, 1, 0, StringTableType::STDistortionTypes, false, ' '),
        new KaliOption("Distort Amount", "DistAmt", 0.0, 99.f, 1.0f, 0.f, StringTableType::None, false, ' '),
        new KaliOption("Distort Target", "DistTgt", 0, 3, 1, 0, StringTableType::STDistortionTargets, false, ' '),
        new KaliOption("Distort Trim", "DstTrim", 0.0f, 1.0f, 0.1f, 1.0f, StringTableType::None, true, '%'),
        new KaliOption("Filter", "Filter", 0, 2, 1, 0, StringTableType::STFilterMode, false, ' '),
        new KaliOption("Delay Range", "DelayRng", 0, DelayRangePresets::DELAY_RANGE_LAST - 1, 1, DelayRangePresets::RANGE_STUDIO, StringTableType::STDelayRange, false, ' '),
        new KaliOption("Load Preset", "LoadPset", 0, 32, 1, 0, StringTableType::None, false, ' '),
        new KaliOption("Save Preset", "SavePset", 0, 32, 1, 0, StringTableType::None, false, ' '),
    },
    {
        new KaliOption("Waveform", "Waveform", 0, daisysp::Oscillator::WAVE_LAST - 1, 1.f, 0.f, StringTableType::STLFOShapes, false, ' '),
        new KaliOption("Mode", "Mode", 0, 13, 1, 0, StringTableType::STLFOModeNames, false, ' '),
        new KaliOption("Oneshot (Single Cycle/Env)", "Oneshot", 0, 1, 1, 0, StringTableType::None, false, ' '),
        new KaliOption("Multiply L Time", "Numera", 1, 100, 1, 1, StringTableType::None, false, ' '),
        new KaliOption("Divide Numerator", "Denomi", 1, 1000, 1, 1.f, StringTableType::None, false, ' '),
        new KaliOption("CV Out Polarity", "Polarity", 0.f, 3.f, 1.f, 1.f, StringTableType::STPolarityNames, false, ' '),
        new KaliOption("LFO Adjust (Varies)", "Adjust", 0.f, 1.f, 0.0625f, 0.f, StringTableType::None, true, '%'),
        // make it fit
        new KaliOption("Offset %", "Offset", -2048.f, 2048.f, 128.f, 0.f, StringTableType::None, false, '%'),
        new KaliOption("Attenuate %", "Atten", 1.f, 100.f, 1.f, 100.f, StringTableType::None, false, '%'),
        // does this even work
        new KaliOption("Phase Offset", "PhzOff", 0.f, 360.f, 1.f, 0.f, StringTableType::None, true, 'o'),
        new KaliOption("Trigger/Reset", "Trig/Rst", 0.f, 2.f, 1.f, 0.f, StringTableType::STGateSources, false, ' '),
        new KaliOption("Calibrated Offset", "OffCal", -2048.f, 2047.f, 1.f, 0.f, StringTableType::None, false, ' '),
        new KaliOption("Calibrated Attenuation", "AttCal", 1.f, 100.f, 1.f, 100.f, StringTableType::None, false, '%'),
        // envelope related
        new KaliOption("ADSR Attack", "Attack", 0.f, 10.f, 0.001f, 0.f, StringTableType::None, true, ' '),
        new KaliOption("ADSR Decay", "Decay", 0.f, 10.f, 0.001f, 0.f, StringTableType::None, true, ' '),
        new KaliOption("ADSR Sustain", "Sustain", 0.f, 1.f, 0.01f, 0.5f, StringTableType::None, true, ' '),
        new KaliOption("ADSR Release", "Release", 0.f, 10.f, 0.001f, 0.2f, StringTableType::None, true, ' '),
        // self control
        new KaliOption("EOC Action", "Eschaton", 0.f, 7.f, 1.f, 0.0f, StringTableType::STEOCActions, true, ' '),
        new KaliOption("EOC Action Target", "PGOAT", 0.f, 10.f, 1.f, 0.0f, StringTableType::STLFOTargetNames, true, ' '),
        new KaliOption("FM From", "FM From", 0.f, 10.f, 1.f, 0.f, StringTableType::STLFOTargetNames, true, ' '),
        new KaliOption("FM Amount", "FM Amt", -100.f, 100.f, 1.f, 0.f, StringTableType::None, true, '%'),
        new KaliOption("AM From", "AM From", 0, 10, 1, 0, StringTableType::STLFOTargetNames, false, ' '),
        new KaliOption("AM Amount", "AM Amt", -100.0f, 100.0f, 1.0f, 0.0f, StringTableType::None, true, '%'),
        // preset related
        new KaliOption("Load Preset", "Load", -1, 32, 1, 0, StringTableType::None, false, 'p'),
        new KaliOption("Save Preset", "Save", 0, 32, 1, 0, StringTableType::None, false, 'p'),
    }};
