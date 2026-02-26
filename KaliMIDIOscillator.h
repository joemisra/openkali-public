#pragma once
#ifndef KALIMIDIOSC_H
#define KALIMIDIOSC_H

#include "KaliMIDI.h"
#include <daisysp.h>

using namespace daisysp;

class KaliMIDIOscillator
{
public:
    KaliMIDIOscillator();
    ~KaliMIDIOscillator();

    enum OscillatorType
    {
        SINE = 0,
        TRIANGLE,
        SAW,
        SQUARE,
        POLYBLEP_SAW,
        POLYBLEP_SQUARE
    };

    void initialize(float sampleRate);

    // Set the oscillator type
    void setWaveform(OscillatorType type);

    // Process MIDI events
    void handleMidiEvent(const daisy::NoteOnEvent &e);
    void handleMidiEvent(const daisy::NoteOffEvent &e);
    void handleMidiEvent(const daisy::ControlChangeEvent &e);

    // Process audio sample
    float process();

    // MIDI CC mappings
    static constexpr int CC_WAVEFORM = 74; // CC for waveform type
    static constexpr int CC_DETUNE = 94;   // CC for detune amount

private:
    // Convert MIDI note number to frequency
    float midiToFreq(float midiNote);

    // Internal oscillators
    Oscillator mainOsc_;
    Oscillator detuneOsc_;

    // Parameters
    float amplitude_;
    float currentNote_;
    float detuneAmount_;
    OscillatorType currentType_;
    bool noteActive_;

    // Track active notes for polyphony support later
    std::vector<daisy::NoteOnEvent> activeNotes_;
};

#endif // KALIMIDIOSC_H