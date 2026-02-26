/*
 * Created by Claude for the Kali firmware project
 * https://claude.ai/chat
 * Simplified for flash optimization
 */

#pragma once
#ifndef PRESET_NAME_GENERATOR_H
#define PRESET_NAME_GENERATOR_H

#include <cstddef>
#include "dpt/daisy_dpt.h"

using namespace daisy;
using namespace dpt;

class PresetNameGenerator
{
public:
    PresetNameGenerator() {}
    ~PresetNameGenerator() {}

    // Generate random preset name (4-8 characters by default)
    // Writes result to provided buffer
    static void generateName(DPT *patch, char *buffer, size_t bufsize,
                             size_t minLength = 4, size_t maxLength = 8);
};

#endif // PRESET_NAME_GENERATOR_H
