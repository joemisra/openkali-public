/* This was written based on existing systems by Claude 2.5 Sonnet. */

#include "KaliVersion.h"

// Provide ODR definition for C++14 builds (avoids undefined reference with LTO)
constexpr FirmwareVersion KaliVersionManager::CURRENT_VERSION;

uint32_t KaliVersionManager::calculateChecksum(const void *data, size_t size)
{
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    uint32_t sum = 0;
    for (size_t i = 0; i < size; i++)
    {
        sum = (sum << 1) | (sum >> 31); // Rotate left
        sum ^= bytes[i];
    }
    return sum;
}

void KaliVersionManager::initializeConfig(OptimizedKaliConfig &config)
{
    config.Init();

    // Set initial values for V2.0
    config.Version = CURRENT_VERSION.toInt();

    // Set default values based on OptionRules
    for (int i = 0; i < OptionsPages::KALI_OPTIONS_LAST; i++)
    {
        config.GlobalOptions[i] = OptionRules[BankType::Global][i]->DefaultValue;
    }

    for (int i = 0; i < DSPOptionsPages::KALI_DSP_OPTIONS_LAST; i++)
    {
        config.DSPOptions[i] = OptionRules[BankType::DSP][i]->DefaultValue;
    }

    for (int i = 0; i < MAX_LFOS; i++)
    {
        for (int j = 0; j < LFOOptionsPages::KALI_LFO_OPTIONS_LAST; j++)
        {
            config.LFOOptions[i][j] = OptionRules[BankType::LFO][j]->DefaultValue;
        }
    }
}

void KaliVersionManager::writeConfig(DPT &patch, const OptimizedKaliConfig &config)
{
    uint32_t address_offset_ = 0;
    address_offset_ = 0x00 & (uint32_t)(~0xff);

    ConfigHeader header = {
        KALI_CONFIG_MAGIC,
        CURRENT_VERSION,
        calculateChecksum(&config, sizeof(config)),
        sizeof(config)};

    // Erase the sector
    patch.qspi.Erase(address_offset_, sizeof(header) + sizeof(config));

    // Write header
    patch.qspi.Write(address_offset_, sizeof(header), (uint8_t *)&header);

    // Write config directly after header
    patch.qspi.Write(address_offset_ + sizeof(header), sizeof(config), (uint8_t *)&config);
}

bool KaliVersionManager::readConfig(DPT &patch, OptimizedKaliConfig &config)
{
    uint32_t address_offset_ = 0;
    address_offset_ = 0x00 & (uint32_t)(~0xff);

    // Get the data directly from QSPI memory
    const ConfigHeader *header = reinterpret_cast<const ConfigHeader *>(
        patch.qspi.GetData(address_offset_));

    // Check if we need to initialize
    if (needsInitialization(*header))
    {
        initializeConfig(config);
        writeConfig(patch, config);
        return true;
    }

    // Read config data from memory after header
    const OptimizedKaliConfig *stored_config = reinterpret_cast<const OptimizedKaliConfig *>(
        patch.qspi.GetData(address_offset_ + sizeof(ConfigHeader)));

    // Copy the config data
    memcpy(&config, stored_config, sizeof(OptimizedKaliConfig));

    // Verify checksum
    uint32_t checksum = calculateChecksum(&config, sizeof(config));
    if (checksum != header->checksum)
    {
        initializeConfig(config);
        writeConfig(patch, config);
        return true;
    }

    return false;
}