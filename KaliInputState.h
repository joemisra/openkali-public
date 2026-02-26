#pragma once

#include "consts.h"
#include "DelayPhasor.h"
#include "KaliDelayLine.h"
#include "KaliInput.h"

class Kali;

// Input state class unchanged
class KaliInputState
{
public:
    KaliInput *inp;
    bool freeze;
    float curdamp;
    float curtime;
    float delaytimes[4];
    float delaytargets[4];
    float override;
    float warb[4];
    float curmet;
    float curmet2;
    float cursplat;
    float dry[4];
    float wet[4];
    float whichout[4];
    DelayPhasor *dp[4];
    size_t size;
    bool allpass;
    KaliDelayLine<float, MAX_DELAY> *delays[4];
    float MAX_DELAY_WORKING;
    float config_new[4];
    Kali *k;
};