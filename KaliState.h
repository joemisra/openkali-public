#pragma once
#include "Kali.h"

// Forward declaration to resolve circular dependency/include
class Kali;

// EditState management
class KaliEditState
{
public:
    enum PageTypeMajor
    {
        LFO_EDIT,
        OPTIONS,
        DSP_EDIT,
        PRESETS,
        MIDI_MODE,
        DEBUG_MODE,
        CLOCK_DEBUG_MODE,
        LAST_PAGE
    };

    enum OptionsTask
    {
        IsEditing,
        IsEditingModulation,
        IsEditingModulationValue
    };

    enum PresetTask
    {
        IsLoading,
        IsSaving,
        IsMutating,
        IsMedusa
    };

    bool cal = false;
    int whichtask;
    int major_mode = PageTypeMajor::LFO_EDIT;
    int minor_mode = 0;
    int minor_mode_subject = 0;
    int minor_mode_modulation_subject = 0;
    int minor_mode_target = 0;

    bool isediting = false;
    bool iseditingmodulation = false;
    bool iseditingmodulationvalue = false;
    bool iseditingsubject = false;
    bool iseditingpresets = false;

    int SelectedIndex = 0;
    int SelectedOptionIndex = 0;
    int SelectedModMapIndex = 0;
    int SelectedPresetIndex = 0;
    int SelectedBank = 1;

    uint16_t encoder_timers[2];

    void ProcessEncoder(int inc);

    void ClearModes()
    {
        SelectedIndex = SelectedOptionIndex = SelectedPresetIndex = SelectedModMapIndex = 0;
        minor_mode = minor_mode_subject = minor_mode_target = minor_mode_modulation_subject = 0;
        isediting = iseditingsubject = iseditingpresets = iseditingmodulation = iseditingmodulationvalue = false;
        encoder_timers[0] = 0;
        encoder_timers[1] = 0;
    }

    void ClearModesAndSetMajor(int major_mode_)
    {
        ClearModes();
        major_mode = major_mode_;
    }
};

// Event types
enum KaliEvent
{
    IDLE,
    LEFTCLICK,
    LEFTHOLD,
    LEFTTURN,
    LEFTHOLDANDTURN,
    RIGHTCLICK,
    RIGHTHOLD,
    RIGHTTURN,
    RIGHTHOLDANDTURN,
    BOTHHELD,
    LFOADJTURN,
    DELAYADJTURN,
    KALIEVENT_LAST
};

// Base state class
class KaliState
{
public:
    virtual void enter(Kali *kali) = 0;
    virtual void handle_event(Kali *kali, int event) = 0;
    virtual void exit(Kali *kali) = 0;
    // virtual KaliState &getNextInstance() = 0;
    // virtual KaliState &getPrevInstance() = 0;
    virtual ~KaliState() {}

    unsigned int order = 0;
};

// Config state for global settings
class KaliConfigState : public KaliState
{
public:
    void enter(Kali *kali) override {}
    void handle_event(Kali *kali, int event) override;
    void exit(Kali *kali) override {}
    // KaliState &getNextInstance() override;
    // KaliState &getPrevInstance() override;

    static KaliState &getInstance();

private:
    KaliConfigState() {}
    KaliConfigState(const KaliConfigState &) = delete;
    KaliConfigState &operator=(const KaliConfigState &) = delete;
};

// LFO configuration state
class KaliConfigOscState : public KaliState
{
public:
    void enter(Kali *kali) override {}
    void handle_event(Kali *kali, int event) override;
    void exit(Kali *kali) override {}
    // KaliState &getNextInstance() override;
    // KaliState &getPrevInstance() override;

    static KaliState &getInstance();

private:
    KaliConfigOscState() {}
    KaliConfigOscState(const KaliConfigOscState &) = delete;
    KaliConfigOscState &operator=(const KaliConfigOscState &) = delete;

    void handlePresetOperations(Kali *kali);
};

// DSP configuration state
class KaliDSPConfigState : public KaliState
{
public:
    void enter(Kali *kali) override {}
    void handle_event(Kali *kali, int event) override;
    void exit(Kali *kali) override {}
    // KaliState &getNextInstance() override;
    // KaliState &getPrevInstance() override;

    static KaliState &getInstance();

private:
    KaliDSPConfigState() {}
    KaliDSPConfigState(const KaliDSPConfigState &) = delete;
    KaliDSPConfigState &operator=(const KaliDSPConfigState &) = delete;
};

// Debug state
class KaliDebugState : public KaliState
{
public:
    void enter(Kali *kali) override {}
    void handle_event(Kali *kali, int event) override;
    void exit(Kali *kali) override {}
    // KaliState &getNextInstance() override;
    // KaliState &getPrevInstance() override;

    static KaliState &getInstance();

private:
    KaliDebugState() {}
    KaliDebugState(const KaliDebugState &) = delete;
    KaliDebugState &operator=(const KaliDebugState &) = delete;
};

class KaliClockDebugState : public KaliState
{
public:
    void enter(Kali *kali) override {}
    void handle_event(Kali *kali, int event) override;
    void exit(Kali *kali) override {}
    // KaliState &getNextInstance() override;
    // KaliState &getPrevInstance() override;

    static KaliState &getInstance();

private:
    KaliClockDebugState() {}
    KaliClockDebugState(const KaliDebugState &) = delete;
    KaliClockDebugState &operator=(const KaliDebugState &) = delete;
};

// MIDI configuration state
class KaliMIDIState : public KaliState
{
public:
    void enter(Kali *kali) override {}
    void handle_event(Kali *kali, int event) override;
    void exit(Kali *kali) override {}
    // KaliState &getNextInstance() override;
    // KaliState &getPrevInstance() override;

    static KaliState &getInstance();

private:
    KaliMIDIState() {}
    KaliMIDIState(const KaliMIDIState &) = delete;
    KaliMIDIState &operator=(const KaliMIDIState &) = delete;
};

// Preset management state
class KaliPresetsState : public KaliState
{
public:
    void enter(Kali *kali) override {}
    void handle_event(Kali *kali, int event) override;
    void exit(Kali *kali) override {}
    // KaliState &getNextInstance() override;
    // KaliState &getPrevInstance() override;

    static KaliState &getInstance();

private:
    KaliPresetsState() {}
    KaliPresetsState(const KaliPresetsState &) = delete;
    KaliPresetsState &operator=(const KaliPresetsState &) = delete;
};