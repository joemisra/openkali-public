#pragma once
#ifndef WAVETABLE_STEP_EDITOR_H
#define WAVETABLE_STEP_EDITOR_H

#include "daisy.h"
#include "daisysp.h"

class WavetableStepEditor
{
public:
    WavetableStepEditor() : table_(nullptr) {}

    void Init(uint8_t* table16)
    {
        table_ = table16;
    }

    void SetStep(uint8_t idx, uint8_t val)
    {
        if(!table_ || idx >= 16) return;
        table_[idx] = val;
    }

    uint8_t GetStep(uint8_t idx) const
    {
        if(!table_ || idx >= 16) return 0;
        return table_[idx];
    }

    // Map CC[0..15] => step; val is 0..127, scale to 0..255
    void ProcessMidiCC(int stepIdx, int val)
    {
        if(!table_) return;
        uint8_t v = static_cast<uint8_t>(daisysp::fclamp(val * 2, 0, 255));
        SetStep(static_cast<uint8_t>(stepIdx), v);
    }

private:
    uint8_t* table_;
};

#endif

