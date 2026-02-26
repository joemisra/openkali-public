/* This was written based on existing systems by Claude 2.5 Sonnet. */
#pragma once
#ifndef KALI_CONFIG_H
#define KALI_CONFIG_H

#include <cstdint>
#include <cstring>         // for memset, memcmp
#include "KaliTypes.h"     // for MAX_OPTIONS references and option enums
#include "KaliOptions.h"   // for MAX_OPTIONS references and option enums
#include "dpt/daisy_dpt.h" // for DPT reference

using namespace daisy;
using namespace daisysp;
using namespace dpt;

// Constants for array sizes
constexpr int MAX_OPTIONS = 32; // Restored - SDRAM only, doesn't affect flash
constexpr int MAX_PRESETS = 32; // Restored - SDRAM only, doesn't affect flash
constexpr int MAX_LFOS = 6;     // Restored - minimal code impact

// Use bit fields to pack boolean flags
struct ConfigFlags
{
    uint8_t Initialized : 1;
    uint8_t Reserved : 7; // Reserved for future use
};

// Compact preset structure
struct CompactPreset
{
    char Name[16]; // Restored to 16 - SDRAM only, doesn't affect flash
    float Options[MAX_OPTIONS];
    ConfigFlags Flags;
    uint8_t BankType; // Previously stored as int, now uint8_t
};

// Main configuration structure
struct OptimizedKaliConfig
{
    float Version;

    // Current state (stored as arrays of floats)
    float GlobalOptions[MAX_OPTIONS];
    float LFOOptions[MAX_LFOS][MAX_OPTIONS];
    float DSPOptions[MAX_OPTIONS];
    float ClockOptions[MAX_OPTIONS];

    // Preset banks
    CompactPreset GlobalPresets[MAX_PRESETS];
    CompactPreset LFOPresets[MAX_PRESETS];
    CompactPreset DSPPresets[MAX_PRESETS];
    CompactPreset ClockPresets[MAX_PRESETS];

    // Helper methods
    void Init()
    {
        Version = 256.5f;
        memset(GlobalOptions, 0, sizeof(GlobalOptions));
        memset(LFOOptions, 0, sizeof(LFOOptions));
        memset(DSPOptions, 0, sizeof(DSPOptions));
        memset(ClockOptions, 0, sizeof(ClockOptions));

        for (int i = 0; i < MAX_PRESETS; i++)
        {
            GlobalPresets[i].Flags.Initialized = false;
            LFOPresets[i].Flags.Initialized = false;
            DSPPresets[i].Flags.Initialized = false;
            ClockPresets[i].Flags.Initialized = false;
        }
    }

    bool operator==(const OptimizedKaliConfig &rhs) const
    {
        return memcmp(this, &rhs, sizeof(*this)) == 0;
    }

    bool operator!=(const OptimizedKaliConfig &rhs) const
    {
        return !operator==(rhs);
    }
};

#endif // KALI_CONFIG_H