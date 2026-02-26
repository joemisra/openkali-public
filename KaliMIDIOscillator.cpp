#include "KaliMIDIOscillator.h"
#include <cmath>

KaliMIDIOscillator::KaliMIDIOscillator()
    : amplitude_(0.0f), currentNote_(60.0f), detuneAmount_(0.0f), currentType_(SINE), noteActive_(false)
{
}

KaliMIDIOscillator::~KaliMIDIOscillator()
{
}

void KaliMIDIOscillator::initialize(float sampleRate)
{
    mainOsc_.Init(sampleRate);
    detuneOsc_.Init(sampleRate);

    // Initialize with default values
    mainOsc_.SetWaveform(Oscillator::WAVE_SIN);
    detuneOsc_.SetWaveform(Oscillator::WAVE_SIN);
    mainOsc_.SetAmp(0.5f);
    detuneOsc_.SetAmp(0.5f);

    amplitude_ = 0.0f;
    detuneAmount_ = 0.0f;

    setWaveform(SINE);
}

void KaliMIDIOscillator::setWaveform(OscillatorType type)
{
    currentType_ = type;

    switch (type)
    {
    case SINE:
        mainOsc_.SetWaveform(Oscillator::WAVE_SIN);
        detuneOsc_.SetWaveform(Oscillator::WAVE_SIN);
        break;
    case TRIANGLE:
        mainOsc_.SetWaveform(Oscillator::WAVE_TRI);
        detuneOsc_.SetWaveform(Oscillator::WAVE_TRI);
        break;
    case SAW:
        mainOsc_.SetWaveform(Oscillator::WAVE_SAW);
        detuneOsc_.SetWaveform(Oscillator::WAVE_SAW);
        break;
    case SQUARE:
        mainOsc_.SetWaveform(Oscillator::WAVE_SQUARE);
        detuneOsc_.SetWaveform(Oscillator::WAVE_SQUARE);
        break;
    case POLYBLEP_SAW:
        mainOsc_.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        detuneOsc_.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        break;
    case POLYBLEP_SQUARE:
        mainOsc_.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
        detuneOsc_.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
        break;
    }
}

void KaliMIDIOscillator::handleMidiEvent(const daisy::NoteOnEvent &e)
{
    if (e.velocity > 0)
    {
        currentNote_ = static_cast<float>(e.note);
        float freq = midiToFreq(currentNote_);
        mainOsc_.SetFreq(freq);
        detuneOsc_.SetFreq(freq * (1.0f + detuneAmount_));

        amplitude_ = e.velocity / 127.0f;
        noteActive_ = true;

        // Store active note
        activeNotes_.push_back(e);
    }
    else
    {
        // Handle note off (velocity 0)
        handleMidiEvent(daisy::NoteOffEvent{e.channel, e.note, 0});
    }
}

void KaliMIDIOscillator::handleMidiEvent(const daisy::NoteOffEvent &e)
{
    // Remove this note from active notes
    for (size_t i = 0; i < activeNotes_.size(); i++)
    {
        if (activeNotes_[i].note == e.note)
        {
            activeNotes_.erase(activeNotes_.begin() + i);
            break;
        }
    }

    // If there are other notes still playing, switch to the last one pressed
    if (!activeNotes_.empty())
    {
        daisy::NoteOnEvent lastNote = activeNotes_.back();
        currentNote_ = static_cast<float>(lastNote.note);
        float freq = midiToFreq(currentNote_);
        mainOsc_.SetFreq(freq);
        detuneOsc_.SetFreq(freq * (1.0f + detuneAmount_));
        amplitude_ = lastNote.velocity / 127.0f;
    }
    else
    {
        // No notes left, silence the oscillator
        noteActive_ = false;
        amplitude_ = 0.0f;
    }
}

void KaliMIDIOscillator::handleMidiEvent(const daisy::ControlChangeEvent &e)
{
    switch (e.control_number)
    {
    case CC_WAVEFORM:
        // Map CC value (0-127) to waveform type
        setWaveform(static_cast<OscillatorType>(e.value % 6));
        break;

    case CC_DETUNE:
        // Map CC value (0-127) to detune amount (0.0-0.1)
        detuneAmount_ = (e.value / 127.0f) * 0.1f;
        // Update detune osc frequency
        if (noteActive_)
        {
            float freq = midiToFreq(currentNote_);
            detuneOsc_.SetFreq(freq * (1.0f + detuneAmount_));
        }
        break;
    }
}

float KaliMIDIOscillator::process()
{
    if (!noteActive_)
    {
        return 0.0f;
    }

    // Mix main oscillator with detuned oscillator
    return (mainOsc_.Process() + detuneOsc_.Process() * detuneAmount_) * amplitude_;
}

float KaliMIDIOscillator::midiToFreq(float midiNote)
{
    return 440.0f * powf(2.0f, (midiNote - 69.0f) / 12.0f);
}

/*
// Example usage in your main application
#include "KaliMIDIOscillator.h"

// In your setup
float sampleRate = 48000.0f;
KaliMIDIOscillator osc;
osc.initialize(sampleRate);

// In your MIDI callback
void MyMidiCallback(daisy::MidiEvent event) {
    switch(event.type) {
        case NoteOn:
            osc.handleMidiEvent(event.AsNoteOn());
            break;
        case NoteOff:
            osc.handleMidiEvent(event.AsNoteOff());
            break;
        case ControlChange:
            osc.handleMidiEvent(event.AsControlChange());
            break;
    }
}

// In your audio callback
void AudioCallback(float* out, size_t size) {
    for(size_t i = 0; i < size; i++) {
        out[i] = osc.process();
    }
}*/