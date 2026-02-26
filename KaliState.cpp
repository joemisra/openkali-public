#include "daisy.h"
#include "Kali.h"
#include "KaliState.h"
#include <algorithm>
#include <array>

namespace
{
    // Local enum for state ordering
    enum KaliEditStateLookup
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

    // Define state order using local enum
    static const std::array<KaliEditStateLookup, LAST_PAGE> STATE_ORDER = {
        LFO_EDIT,
        OPTIONS,
        DSP_EDIT,
        PRESETS,
        MIDI_MODE,
        DEBUG_MODE,
        CLOCK_DEBUG_MODE};

    // Convert local enum to PageTypeMajor
    // KaliEditState::PageTypeMajor toPageType(KaliEditStateLookup lookup)
    //{
    //    return static_cast<KaliEditState::PageTypeMajor>(lookup);
    //}

    // Gets next state in order (clamped)
    KaliEditState::PageTypeMajor GetStateByEncoder(KaliEditState::PageTypeMajor currentState, int encoder)
    {
        int new_state = DSY_CLAMP(static_cast<int>(currentState) + encoder, 0, KaliEditState::LAST_PAGE - 1);

        return static_cast<KaliEditState::PageTypeMajor>(new_state);
    }

    // Common state transition helper
    void handleStateTransition(Kali *kali, KaliEditState::PageTypeMajor nextState)
    {
        kali->editstate.ClearModesAndSetMajor(nextState);
    }

    // Common encoder processing helper
    void handleRightTurn(Kali *kali, bool isEditing, int bankType)
    {
        if (bankType == BankType::LFO)
        {
            // this is a special case, we need to edit based on selected lfo
            if (isEditing)
            {
                // if editing, this will change the value of whatever option is selected (the right side of the screen)
                kali->warble[kali->editstate.SelectedIndex].preset.SetOptionByEncoder(kali->editstate.SelectedOptionIndex, kali->e1inc);
            }
            else
            {
                // if not editing, this changes which option is selected (the right side of the screen)
                kali->SetSelectedOptionByEncoder(kali->e1inc, BankType::LFO);

                // an old variable
                // kali->editstate.minor_mode_target = kali->editstate.SelectedIndex;
            }
        }
        else
        {
            if (isEditing)
            {
                // set selected option's value by encoder
                kali->SetOptionValueByEncoder(kali->editstate.SelectedOptionIndex, kali->e1inc, bankType);
            }
            else
            {
                kali->SetSelectedOptionByEncoder(kali->e1inc, bankType);
                kali->editstate.minor_mode_target = kali->editstate.SelectedIndex;
            }
        }
    }

    // Common left turn handler
    void handleLeftTurn(Kali *kali, bool isEditingSubject)
    {
        if (isEditingSubject)
        {
            // For debug pages, there are 5 entries [0..4]
            if (kali->editstate.major_mode == KaliEditState::DEBUG_MODE || kali->editstate.major_mode == KaliEditState::CLOCK_DEBUG_MODE)
            {
                kali->editstate.SelectedIndex = DSY_CLAMP(kali->editstate.SelectedIndex + kali->e2inc, 0, 4);
            }
            else
            {
                kali->editstate.SelectedIndex = DSY_CLAMP(kali->editstate.SelectedIndex + kali->e2inc, 0, 5);
            }
        }
        else
        {
            kali->SetStateByInt(kali->editstate.major_mode + kali->e2inc);
        }
    }
}

// State class implementations
KaliState &KaliConfigState::getInstance()
{
    static KaliConfigState singleton;
    return singleton;
}

void KaliConfigState::handle_event(Kali *kali, int event)
{
    switch (event)
    {
    case KaliEvent::LEFTCLICK:
        handleStateTransition(kali, KaliEditState::DSP_EDIT);
        kali->setState(KaliDSPConfigState::getInstance());
        break;

    case KaliEvent::LEFTHOLD:
        if (++kali->editstate.encoder_timers[1] >= 800)
        {
            handleStateTransition(kali, KaliEditState::OPTIONS);
        }
        break;

    case KaliEvent::LEFTTURN:
        kali->SetStateByInt(kali->editstate.major_mode + kali->e2inc);
        break;

    case KaliEvent::RIGHTCLICK:
        kali->editstate.isediting = !kali->editstate.isediting;
        break;

    case KaliEvent::RIGHTHOLD:
        if (++kali->editstate.encoder_timers[0] >= 800)
        {
            kali->editstate.iseditingmodulation = true;
        }
        break;

    case KaliEvent::RIGHTTURN:
        handleRightTurn(kali, kali->editstate.isediting, BankType::Global);

        break;
    }
}

KaliState &KaliConfigOscState::getInstance()
{
    static KaliConfigOscState singleton;
    return singleton;
}

void KaliConfigOscState::handle_event(Kali *kali, int event)
{
    switch (event)
    {
    case KaliEvent::LFOADJTURN:
        // kali->editstate.SelectedIndex = kali->inp.Knobs[Kali::CV::LFO_ADJUST].Value() * 6.f;
        break;

    case KaliEvent::DELAYADJTURN:
        if (kali->editstate.isediting)
        {
            kali->warble[kali->editstate.SelectedIndex].preset.SetOptionUnsafe(
                kali->editstate.SelectedOptionIndex,
                (int)(kali->inp.Knobs[Kali::CV::DELAY_ADJUST].Value() * LFOOptionsPages::KALI_LFO_OPTIONS_LAST));
        }
        break;

    case KaliEvent::LEFTCLICK:
        kali->editstate.iseditingsubject = !kali->editstate.iseditingsubject;
        break;

    case KaliEvent::RIGHTCLICK:
        if (kali->editstate.isediting)
        {
            handlePresetOperations(kali);
        }
        kali->editstate.isediting = !kali->editstate.isediting;
        break;

    case KaliEvent::RIGHTTURN:
        handleRightTurn(kali, kali->editstate.isediting, BankType::LFO);
        break;

    case KaliEvent::LEFTTURN:
        handleLeftTurn(kali, kali->editstate.iseditingsubject);
        break;
    }
}

void KaliConfigOscState::handlePresetOperations(Kali *kali)
{
    if (kali->editstate.SelectedOptionIndex == LFOOptionsPages::LFOPresetLoad)
    {
        auto loadPresetNum = kali->warble[kali->editstate.SelectedIndex].preset.GetOption(kali->editstate.SelectedOptionIndex);
        kali->LoadLFOPreset(kali->editstate.SelectedIndex, loadPresetNum);
    }
    else if (kali->editstate.SelectedOptionIndex == LFOOptionsPages::LFOPresetSave)
    {
        // std::string name = kali->nameGen.generateName(&kali->patch, 4, 16);
        // strncpy(kali->warble[kali->editstate.SelectedIndex].preset.Name, name.c_str(), 16);
        kali->saveSettings = true;
    }
}

KaliState &KaliDSPConfigState::getInstance()
{
    static KaliDSPConfigState singleton;
    return singleton;
}

void KaliDSPConfigState::handle_event(Kali *kali, int event)
{
    switch (event)
    {
    case KaliEvent::LEFTCLICK:
        handleStateTransition(kali, KaliEditState::LFO_EDIT);
        kali->setState(KaliConfigOscState::getInstance());
        break;

    case KaliEvent::RIGHTCLICK:
        kali->editstate.isediting = !kali->editstate.isediting;
        break;

    case KaliEvent::RIGHTTURN:
        handleRightTurn(kali, kali->editstate.isediting, BankType::DSP);
        break;

    case KaliEvent::LEFTTURN:
        kali->SetStateByInt(kali->editstate.major_mode + kali->e2inc);
        break;
    }
}

KaliState &KaliDebugState::getInstance()
{
    static KaliDebugState singleton;
    return singleton;
}

void KaliDebugState::handle_event(Kali *kali, int event)
{
    switch (event)
    {
    case KaliEvent::LEFTCLICK:
        // RESET TO DEFAULTS
        kali->initSettings = true;
        kali->saveSettings = true;
        break;
    case KaliEvent::RIGHTCLICK:
        // RESET TO DEFAULTS
        kali->saveSettings = true;
        // kali->initSettings = true;
        break;

    case KaliEvent::LEFTTURN:
        kali->SetStateByInt(kali->editstate.major_mode + kali->e2inc);
        break;

    case KaliEvent::RIGHTTURN:
        kali->editstate.SelectedIndex = DSY_CLAMP(kali->editstate.SelectedIndex + kali->e1inc, 0, 4);
        break;
    }
}

KaliState &KaliClockDebugState::getInstance()
{
    static KaliClockDebugState singleton;
    return singleton;
}

void KaliClockDebugState::handle_event(Kali *kali, int event)
{
    switch (event)
    {
    case KaliEvent::RIGHTCLICK:
        // RESET TO DEFAULTS
        kali->saveSettings = true;
        kali->initSettings = true;
        break;

    case KaliEvent::LEFTTURN:
        kali->SetStateByInt(kali->editstate.major_mode + kali->e2inc);
        break;

    case KaliEvent::RIGHTTURN:
        kali->editstate.SelectedIndex = DSY_CLAMP(kali->editstate.SelectedIndex + kali->e1inc, 0, 4);
        break;
    }
}

KaliState &KaliMIDIState::getInstance()
{
    static KaliMIDIState singleton;
    return singleton;
}

void KaliMIDIState::handle_event(Kali *kali, int event)
{
    if (event == KaliEvent::LEFTTURN)
    {
        kali->SetStateByInt(kali->editstate.major_mode + kali->e2inc);
    }
}

KaliState &KaliPresetsState::getInstance()
{
    static KaliPresetsState singleton;
    return singleton;
}

void KaliPresetsState::handle_event(Kali *kali, int event)
{
    switch (event)
    {
    case KaliEvent::LEFTTURN:
        // Keep left encoder as global mode change (navigate pages)
        if (kali->e2inc != 0)
        {
            kali->SetStateByInt(kali->editstate.major_mode + kali->e2inc);
        }
        break;

    case KaliEvent::RIGHTTURN:
        // Use right encoder to change preset slot
        if (kali->e1inc != 0)
        {
            int s = kali->editstate.SelectedPresetIndex + kali->e1inc;
            if (s < 0)
                s = MAX_PRESETS - 1;
            else if (s >= MAX_PRESETS)
                s = 0;
            kali->editstate.SelectedPresetIndex = s;
        }
        break;

    case KaliEvent::LEFTCLICK:
        // Cancel/exit save mode (if in save mode), otherwise do nothing
        if (kali->preset_save_mode)
        {
            kali->preset_save_mode = false;
        }
        break;

    case KaliEvent::RIGHTHOLD:
        // Right hold enters save mode
        if (!kali->preset_save_mode)
        {
            kali->preset_save_mode = true;
        }
        break;

    case KaliEvent::RIGHTCLICK:
    {
        // Right click executes current action (load or save)
        int slot = kali->editstate.SelectedPresetIndex;
        if (kali->preset_save_mode)
        {
            kali->SaveMainPreset(slot);
            kali->preset_save_mode = false; // Exit save mode after saving
        }
        else
        {
            kali->LoadMainPreset(slot);
        }
        break;
    }
    default:
        break;
    }
}

// Use state order for transitions
KaliState &getStateInstance(KaliEditState::PageTypeMajor state)
{
    switch (state)
    {
    case KaliEditState::LFO_EDIT:
        return KaliConfigOscState::getInstance();
    case KaliEditState::OPTIONS:
        return KaliConfigState::getInstance();
    case KaliEditState::DSP_EDIT:
        return KaliDSPConfigState::getInstance();
    case KaliEditState::PRESETS:
        return KaliPresetsState::getInstance();
    case KaliEditState::MIDI_MODE:
        return KaliMIDIState::getInstance();
    case KaliEditState::DEBUG_MODE:
        return KaliDebugState::getInstance();
    // back to beginning
    case KaliEditState::LAST_PAGE:
    default:
        return KaliConfigOscState::getInstance();
    }
}

// State transition implementations
// KaliState &KaliConfigState::getNextInstance()
// {
//     return getStateInstance(SetStateiBy());
//     return getStateInstance(GetStateByEncoder(KaliEditState::OPTIONS));
// }
// KaliState &KaliConfigState::getPrevInstance()
// {
//     return getStateInstance(getPrevState(KaliEditState::OPTIONS));
// }

// KaliState &KaliConfigOscState::getNextInstance()
// {
//     return getStateInstance(getNextState(KaliEditState::LFO_EDIT));
// }
// KaliState &KaliConfigOscState::getPrevInstance()
// {
//     return getStateInstance(getPrevState(KaliEditState::LFO_EDIT));
// }

// KaliState &KaliDSPConfigState::getNextInstance()
// {
//     return getStateInstance(getNextState(KaliEditState::DSP_EDIT));
// }
// KaliState &KaliDSPConfigState::getPrevInstance()
// {
//     return getStateInstance(getPrevState(KaliEditState::DSP_EDIT));
// }

// KaliState &KaliMIDIState::getNextInstance()
// {
//     return getStateInstance(getNextState(KaliEditState::MIDI_MODE));
// }
// KaliState &KaliMIDIState::getPrevInstance()
// {
//     return getStateInstance(getPrevState(KaliEditState::MIDI_MODE));
// }

// KaliState &KaliPresetsState::getNextInstance()
// {
//     return getStateInstance(getNextState(KaliEditState::PRESETS));
// }
// KaliState &KaliPresetsState::getPrevInstance()
// {
//     return getStateInstance(getPrevState(KaliEditState::PRESETS));
// }

// KaliState &KaliDebugState::getNextInstance()
// {
//     return getStateInstance(getNextState(KaliEditState::DEBUG_MODE));
// }
// KaliState &KaliDebugState::getPrevInstance()
// {
//     return getStateInstance(getPrevState(KaliEditState::DEBUG_MODE));
// }

void KaliEditState::ProcessEncoder(int inc)
{
    if (isediting)
    {
        SelectedOptionIndex = SelectedOptionIndex + inc;
    }
    else
    {
        SelectedIndex = SelectedIndex + inc;
    }
}