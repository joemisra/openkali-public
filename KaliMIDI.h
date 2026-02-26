#pragma once
#ifndef KALIMIDI_H
#define KALIMIDI_H

#include <daisy.h>
#include <array>
#include <vector>
#include <functional>

using namespace daisy;

// Forward declarations
class Kali;

// MIDI CC callback function type
using CCCallback = std::function<void(Kali*, int channel, int cc, int value)>;

// MIDI CC mapping structure
struct CCMapping {
    int channel;
    int cc;
    int bank_type;     // BankType::Global, DSP, or LFO
    int option_index;  // Index into the options array
    int lfo_index;     // For LFO bank, which LFO (0-5), -1 for others
    CCCallback callback; // Optional custom callback, nullptr for default
};

class KaliMIDI
{
public:
    KaliMIDI();
    ~KaliMIDI();

    int CC[128];
    bool CC_Received[128]; // Track if CC has been received (for override logic)
    uint32_t CC_LastUpdateTime[128]; // Track last update time for each CC to prevent flooding
    // Track note state for each MIDI note (0-127 inclusive)
    std::array<NoteOnEvent, 128> NoteOnBuffer;
    NoteOnEvent LastNote;
    
    // MIDI CC mapping system
    // Total mappings required: Global(17) + DSP(11) + LFO(6*23)=166
    // Provide headroom to avoid overflow when adding mappings
    static const int MAX_CC_MAPPINGS = 192;
    CCMapping cc_mappings[MAX_CC_MAPPINGS];
    int num_mappings;
    // Fast lookup: [channel 0-15][cc 0-127] -> mapping index or -1
    int cc_lookup[16][128];

    void initialize();
    void sendNoteOn(int channel, int note, int velocity);
    void sendNoteOff(int channel, int note, int velocity);
    void sendControlChange(int channel, int control, int value);
    void receiveNoteOn(NoteOnEvent e);
    void receiveNoteOff(NoteOffEvent e);
    // Returns true if the CC value was updated (passes rate limiting)
    bool receiveControlChange(ControlChangeEvent e);
    // Overload with kali instance; processes mapped CCs only when updated
    bool receiveControlChange(ControlChangeEvent e, Kali* kali);

    std::vector<NoteOnEvent> getActiveNotes() const;
    void clearNoteBuffer();
    void resetCCOverrides(); // Reset CC_Received flags to return to front panel control
    void resetCCOverride(int cc_number); // Reset specific CC override
    
    // MIDI CC mapping system
    void initializeCCMappings(Kali* kali);
    void processMappedCC(Kali* kali, int channel, int cc, int value);
    void addCCMapping(int channel, int cc, int bank_type, int option_index, int lfo_index = -1, CCCallback callback = nullptr);
    
private:
    void defaultCCHandler(Kali* kali, int channel, int cc, int value, int bank_type, int option_index, int lfo_index);
};
#endif
