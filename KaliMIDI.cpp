#include "KaliMIDI.h"
#include "sys/system.h"
#include "Kali.h"
#include "KaliOptions.h"

KaliMIDI::KaliMIDI()
{
    // Initialize all CC values to 0 and reset rate limit timers
    for (int i = 0; i < 128; i++)
    {
        CC[i] = 0;
        CC_Received[i] = false;
        CC_LastUpdateTime[i] = 0;
    }
    // Initialize CC lookup to -1 (no mapping)
    for(int ch = 0; ch < 16; ++ch)
        for(int cc = 0; cc < 128; ++cc)
            cc_lookup[ch][cc] = -1;
    
    // Initialize CC mappings
    num_mappings = 0;
    for (int i = 0; i < MAX_CC_MAPPINGS; i++)
    {
        cc_mappings[i] = {0, 0, 0, 0, -1, nullptr};
    }
}
KaliMIDI::~KaliMIDI()
{
}

bool KaliMIDI::receiveControlChange(ControlChangeEvent e)
{
    // Validate CC number to prevent OOB access
    if(e.control_number < 0 || e.control_number > 127)
        return false;
    // Extra aggressive rate limiting for P1-P4 CCs to prevent system overload
    uint32_t currentTime = daisy::System::GetNow();
    uint32_t minTime = 10; // Default 10ms
    
    // More aggressive limiting for P1-P4 parameters (CC 21-24) since they affect DSP
    if (e.control_number >= 21 && e.control_number <= 24)
    {
        minTime = 25; // 25ms minimum for P1-P4 (40Hz max update rate)
    }
    
    if (currentTime - CC_LastUpdateTime[e.control_number] >= minTime)
    {
        CC[e.control_number] = e.value;
        CC_Received[e.control_number] = true; // Mark as received for override logic
        CC_LastUpdateTime[e.control_number] = currentTime;
        return true;
    }
    return false;
}

bool KaliMIDI::receiveControlChange(ControlChangeEvent e, Kali* kali)
{
    // Call the original function first
    bool updated = receiveControlChange(e);
    
    // Then process mapped CCs if we have a kali instance
    if (kali && updated)
    {
        processMappedCC(kali, e.channel, e.control_number, e.value);
    }
    return updated;
}

void KaliMIDI::receiveNoteOn(NoteOnEvent e)
{
    if(e.note >= 0 && e.note < 128)
        NoteOnBuffer[e.note] = e;
    LastNote = e;
}

void KaliMIDI::receiveNoteOff(NoteOffEvent e)
{
    if(e.note >= 0 && e.note < 128)
        NoteOnBuffer[e.note].velocity = 0;
}

std::vector<NoteOnEvent> KaliMIDI::getActiveNotes() const
{
    std::vector<NoteOnEvent> activeNotes;
    for (const auto &note : NoteOnBuffer)
    {
        if (note.velocity > 0)
        {
            activeNotes.push_back(note);
        }
    }
    return activeNotes;
}

void KaliMIDI::clearNoteBuffer()
{
    for (auto &note : NoteOnBuffer)
    {
        note.velocity = 0;
    }
}

void KaliMIDI::resetCCOverrides()
{
    // Reset CC_Received flags to return to front panel control
    for (int i = 0; i < 128; i++)
    {
        CC_Received[i] = false;
    }
}

void KaliMIDI::resetCCOverride(int cc_number)
{
    // Reset specific CC override to return to front panel control for that parameter
    if (cc_number >= 0 && cc_number < 128)
    {
        CC_Received[cc_number] = false;
    }
}

void KaliMIDI::initializeCCMappings(Kali* kali)
{
    num_mappings = 0;
    // Reset lookup
    for(int ch = 0; ch < 16; ++ch)
        for(int cc = 0; cc < 128; ++cc)
            cc_lookup[ch][cc] = -1;
    
    // Channels 0-5: LFO Options (6 LFOs, each gets a channel)
    for (int lfo = 0; lfo < 6; lfo++)
    {
        int channel = 0 + lfo; // Channels 0,1,2,3,4,5
        addCCMapping(channel, 1, BankType::LFO, 0, lfo);  // Waveform
        addCCMapping(channel, 2, BankType::LFO, 1, lfo);  // LFOMode
        addCCMapping(channel, 3, BankType::LFO, 2, lfo);  // Oneshot
        addCCMapping(channel, 4, BankType::LFO, 3, lfo);  // Meta
        addCCMapping(channel, 5, BankType::LFO, 4, lfo);  // MetaDivider
        addCCMapping(channel, 6, BankType::LFO, 5, lfo);  // Polarity
        addCCMapping(channel, 7, BankType::LFO, 6, lfo);  // Adjust
        addCCMapping(channel, 8, BankType::LFO, 7, lfo);  // Offset
        addCCMapping(channel, 9, BankType::LFO, 8, lfo);  // Attenuate
        addCCMapping(channel, 10, BankType::LFO, 9, lfo); // PhaseOffset
        addCCMapping(channel, 11, BankType::LFO, 10, lfo); // GateIn
        addCCMapping(channel, 12, BankType::LFO, 11, lfo); // OffsetCal
        addCCMapping(channel, 13, BankType::LFO, 12, lfo); // AttenuateCal
        addCCMapping(channel, 14, BankType::LFO, 13, lfo); // Attack
        addCCMapping(channel, 15, BankType::LFO, 14, lfo); // Decay
        addCCMapping(channel, 16, BankType::LFO, 15, lfo); // Sustain
        addCCMapping(channel, 17, BankType::LFO, 16, lfo); // Release
        addCCMapping(channel, 18, BankType::LFO, 17, lfo); // Eschaton
        addCCMapping(channel, 19, BankType::LFO, 18, lfo); // Incandenza
        addCCMapping(channel, 20, BankType::LFO, 19, lfo); // FMSource
        addCCMapping(channel, 21, BankType::LFO, 20, lfo); // FMSourceAmount
        addCCMapping(channel, 22, BankType::LFO, 21, lfo); // AMSource
        addCCMapping(channel, 23, BankType::LFO, 22, lfo); // AMAmount
    }

    // Channel 6: DSP Options
    addCCMapping(6, 1, BankType::DSP, 0);   // Mode
    addCCMapping(6, 2, BankType::DSP, 1);   // P1
    addCCMapping(6, 3, BankType::DSP, 2);   // P2
    addCCMapping(6, 4, BankType::DSP, 3);   // P3
    addCCMapping(6, 5, BankType::DSP, 4);   // P4
    addCCMapping(6, 6, BankType::DSP, 5);   // Distortion
    addCCMapping(6, 7, BankType::DSP, 6);   // DistortionAmount
    addCCMapping(6, 8, BankType::DSP, 7);   // DistortionTarget
    addCCMapping(6, 9, BankType::DSP, 8);   // DistortionTrim
    addCCMapping(6, 10, BankType::DSP, 9);  // FilterType
    addCCMapping(6, 11, BankType::DSP, 10); // DelayRangePreset

    // Channel 7: Global Options
    addCCMapping(7, 1, BankType::Global, 0);   // LfoRateMultiplier
    addCCMapping(7, 2, BankType::Global, 1);   // LeftClockRateMultiplier
    addCCMapping(7, 3, BankType::Global, 2);   // RightClockRateMultiplier
    addCCMapping(7, 4, BankType::Global, 3);   // ExternalCvClockPPQN
    addCCMapping(7, 5, BankType::Global, 4);   // SyncEngine
    addCCMapping(7, 6, BankType::Global, 5);   // SyncButtonMode
    addCCMapping(7, 7, BankType::Global, 6);   // FreezeButtonMode
    addCCMapping(7, 8, BankType::Global, 7);   // UseAllpass
    addCCMapping(7, 9, BankType::Global, 8);   // ReverbWetSend
    addCCMapping(7, 10, BankType::Global, 9);  // ReverbDrySend
    addCCMapping(7, 11, BankType::Global, 10); // ReverbFeedback
    addCCMapping(7, 12, BankType::Global, 11); // ReverbDamp
    addCCMapping(7, 13, BankType::Global, 12); // AdcAttenuation
    addCCMapping(7, 14, BankType::Global, 13); // DacAttenuation
    addCCMapping(7, 15, BankType::Global, 14); // InputWidth
    addCCMapping(7, 16, BankType::Global, 15); // InvertEncoders
    addCCMapping(7, 17, BankType::Global, 16); // EnableDebugInfo
}

void KaliMIDI::addCCMapping(int channel, int cc, int bank_type, int option_index, int lfo_index, CCCallback callback)
{
    if (num_mappings < MAX_CC_MAPPINGS)
    {
        cc_mappings[num_mappings] = {channel, cc, bank_type, option_index, lfo_index, callback};
        if(channel >= 0 && channel < 16 && cc >= 0 && cc < 128)
            cc_lookup[channel][cc] = num_mappings;
        num_mappings++;
    }
}

void KaliMIDI::processMappedCC(Kali* kali, int channel, int cc, int value)
{
    if(channel < 0 || channel >= 16 || cc < 0 || cc >= 128)
        return;
    int idx = cc_lookup[channel][cc];
    if(idx < 0 || idx >= num_mappings)
        return;
    CCMapping& mapping = cc_mappings[idx];
    if (mapping.callback)
    {
        mapping.callback(kali, channel, cc, value);
    }
    else
    {
        defaultCCHandler(kali, channel, cc, value, mapping.bank_type, mapping.option_index, mapping.lfo_index);
    }
}

void KaliMIDI::defaultCCHandler(Kali* kali, int channel, int cc, int value, int bank_type, int option_index, int lfo_index)
{
    // Convert MIDI value (0-127) to option range
    float normalized_value = value / 127.0f;
    
    switch (bank_type)
    {
    case BankType::Global:
        {
            KaliOption* option = OptionRules[BankType::Global][option_index];
            float scaled_value = option->Min + (normalized_value * (option->Max - option->Min));
            kali->SetOptionValue(option_index, scaled_value, BankType::Global);
        }
        break;
        
    case BankType::DSP:
        {
            KaliOption* option = OptionRules[BankType::DSP][option_index];
            float scaled_value = option->Min + (normalized_value * (option->Max - option->Min));
            kali->SetOptionValue(option_index, scaled_value, BankType::DSP);
        }
        break;
        
    case BankType::LFO:
        if (lfo_index >= 0 && lfo_index < 6)
        {
            KaliOption* option = OptionRules[BankType::LFO][option_index];
            float scaled_value = option->Min + (normalized_value * (option->Max - option->Min));
            kali->warble[lfo_index].preset.SetOption(option_index, scaled_value);
        }
        break;
    }
}
