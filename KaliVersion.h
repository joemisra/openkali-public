/* This was written based on existing systems by Claude 2.5 Sonnet. */

#pragma once
#ifndef KALI_VERSION_H
#define KALI_VERSION_H

#include <cstdint>
#include "KaliConfig.h"  // for OptimizedKaliConfig
#include "KaliOptions.h" // for OptionRules and enums
#include "dpt/daisy_dpt.h"

using namespace daisy;
using namespace daisysp;
using namespace dpt;

// Version format: Major.Minor.Patch stored as uint32_t
struct FirmwareVersion
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t reserved;

    bool operator<(const FirmwareVersion &other) const
    {
        if (major != other.major)
            return major < other.major;
        if (minor != other.minor)
            return minor < other.minor;
        return patch < other.patch;
    }

    bool operator==(const FirmwareVersion &other) const
    {
        return major == other.major &&
               minor == other.minor &&
               patch == other.patch;
    }

    uint32_t toInt() const
    {
        return (major << 24) | (minor << 16) | (patch << 8);
    }

    static FirmwareVersion fromInt(uint32_t version)
    {
        return {
            static_cast<uint8_t>((version >> 24) & 0xFF),
            static_cast<uint8_t>((version >> 16) & 0xFF),
            static_cast<uint8_t>((version >> 8) & 0xFF),
            0};
    }
};

// Magic number to detect valid config
static const uint32_t KALI_CONFIG_MAGIC = 0x4B414C49; // "KALI" in ASCII

// Header stored at the start of NVRAM
struct ConfigHeader
{
    uint32_t magic;          // Magic number to detect valid config
    FirmwareVersion version; // Firmware version that wrote this config
    uint32_t checksum;       // Simple checksum of config data
    uint32_t size;           // Size of config data
};

class KaliVersionManager
{
public:
    static constexpr FirmwareVersion CURRENT_VERSION = {2, 0, 0, 1}; // Update for each release

    static bool needsInitialization(const ConfigHeader &header)
    {
        // Invalid magic number means fresh device or corrupted config
        if (header.magic != KALI_CONFIG_MAGIC)
        {
            return true;
        }

        // Major version change requires initialization
        if (header.version.major != CURRENT_VERSION.major)
        {
            return true;
        }

        return false;
    }

    static uint32_t calculateChecksum(const void *data, size_t size);
    static void initializeConfig(OptimizedKaliConfig &config);
    static void writeConfig(DPT &patch, const OptimizedKaliConfig &config);
    static bool readConfig(DPT &patch, OptimizedKaliConfig &config);
};

#endif // KALI_VERSION_H