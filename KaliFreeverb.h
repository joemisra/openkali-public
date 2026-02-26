/*
 * Created with assistance from Claude 3.5 Sonnet
 * https://claude.anthropic.com/chat/...
 *
 * Implementation based on original Freeverb algorithm by Jezar at Dreampoint
 */

#pragma once
#ifndef KALI_FREEVERB_H
#define KALI_FREEVERB_H

#include <cstring>
#include "daisysp.h"

using namespace daisysp;

class KaliFreeverb
{
public:
    static constexpr int numcombs = 8;
    static constexpr int numallpasses = 4;
    static constexpr float muted = 0;
    static constexpr float fixedgain = 0.015f;
    static constexpr float scalewet = 3;
    static constexpr float scaledry = 2;
    static constexpr float scaledamp = 0.4f;
    static constexpr float scaleroom = 0.28f;
    static constexpr float offsetroom = 0.7f;
    static constexpr float initialroom = 0.5f;
    static constexpr float initialdamp = 0.5f;
    static constexpr float initialwet = 1 / scalewet;
    static constexpr float initialdry = 0;
    static constexpr float initialwidth = 1;
    static constexpr float initialmode = 0;
    static constexpr float freezemode = 0.5f;

    // Comb filter tunings - these are taken from original Freeverb
    static constexpr int combtuningL1 = 1116;
    static constexpr int combtuningL2 = 1188;
    static constexpr int combtuningL3 = 1277;
    static constexpr int combtuningL4 = 1356;
    static constexpr int combtuningL5 = 1422;
    static constexpr int combtuningL6 = 1491;
    static constexpr int combtuningL7 = 1557;
    static constexpr int combtuningL8 = 1617;

    static constexpr int allpasstuningL1 = 556;
    static constexpr int allpasstuningL2 = 441;
    static constexpr int allpasstuningL3 = 341;
    static constexpr int allpasstuningL4 = 225;

    void Init(float samplerate)
    {
        this->samplerate = samplerate;
        bufferL2_idx = 0;

        // Clear buffers
        memset(bufferL2, 0, sizeof(float) * (combtuningL1 + 1));
        for (int i = 0; i < numcombs; i++)
        {
            memset(combL[i], 0, sizeof(float) * (combtuningL1 + 1));
            combidxL[i] = 0;
        }

        for (int i = 0; i < numallpasses; i++)
        {
            memset(allpassL[i], 0, sizeof(float) * (allpasstuningL1 + 1));
            allpassidxL[i] = 0;
        }

        // Set default values
        SetWet(initialwet);
        SetRoomSize(initialroom);
        SetDry(initialdry);
        SetDamp(initialdamp);
        SetWidth(initialwidth);
        SetMode(initialmode);

        // Buffer will be full of silence at start
        gain = fixedgain;
    }

    void Process(float in, float &outL)
    {
        float input = in * gain;
        float outL = 0;
        float outL2 = 0; // Second output for stereo width

        // Store input for direct path
        float inputL = input;

        // Parallel comb filters
        for (int i = 0; i < numcombs; i++)
        {
            outL += CombProcess(input, i);
        }

        // Series allpass filters
        for (int i = 0; i < numallpasses; i++)
        {
            outL = AllpassProcess(outL, i);
        }

        // Store in second buffer for stereo spread
        bufferL2[bufferL2_idx] = outL;
        if (++bufferL2_idx >= combtuningL1)
        {
            bufferL2_idx = 0;
        }

        // Get delayed output for stereo spread
        outL2 = bufferL2[bufferL2_idx];

        // Calculate final output
        outL = outL * wet1 + outL2 * wet2 + in * dry;
    }

    void SetRoomSize(float value)
    {
        roomsize = value * scaleroom + offsetroom;
        UpdateCombDamp();
    }

    void SetDamp(float value)
    {
        damp = value * scaledamp;
        UpdateCombDamp();
    }

    void SetWet(float value)
    {
        wet = value * scalewet;
        UpdateWetDry();
    }

    void SetDry(float value)
    {
        dry = value * scaledry;
    }

    void SetWidth(float value)
    {
        width = value;
        UpdateWetDry();
    }

    void SetMode(float value)
    {
        mode = value;
        UpdateCombDamp();
    }

    void SetFreeze(bool frozen)
    {
        if (frozen)
        {
            roomsize = 1;
            damp = 0;
            gain = muted;
        }
        else
        {
            roomsize = initialroom;
            damp = initialdamp;
            gain = fixedgain;
        }
        UpdateCombDamp();
    }

private:
    float samplerate;
    float gain;
    float roomsize, damp;
    float wet1, wet2, wet;
    float dry;
    float width;
    float mode;

    // Buffer for second output path
    float bufferL2[combtuningL1 + 1];
    int bufferL2_idx;

    // Comb filters
    float combL[numcombs][combtuningL1 + 1];
    int combidxL[numcombs];
    float combfeedback[numcombs]; // Feedback for each comb
    float combdamp1[numcombs];    // Dampening for each comb
    float combdamp2[numcombs];

    // Allpass filters
    float allpassL[numallpasses][allpasstuningL1 + 1];
    int allpassidxL[numallpasses];

    float CombProcess(float input, int comb)
    {
        float output = combL[comb][combidxL[comb]];
        float temp = (output * combdamp2[comb]) + (combfeedback[comb] * combdamp1[comb]);

        combfeedback[comb] = temp;
        combL[comb][combidxL[comb]] = input + (temp * roomsize);

        if (++combidxL[comb] >= GetCombTuning(comb))
        {
            combidxL[comb] = 0;
        }

        return output;
    }

    float AllpassProcess(float input, int allpass)
    {
        float output = allpassL[allpass][allpassidxL[allpass]];
        float bufout = output;

        output = -input + bufout;
        allpassL[allpass][allpassidxL[allpass]] = input + (bufout * 0.5f);

        if (++allpassidxL[allpass] >= GetAllpassTuning(allpass))
        {
            allpassidxL[allpass] = 0;
        }

        return output;
    }

    void UpdateCombDamp()
    {
        for (int i = 0; i < numcombs; i++)
        {
            combdamp1[i] = damp;
            combdamp2[i] = 1 - damp;
        }
    }

    void UpdateWetDry()
    {
        wet1 = wet * (width * 0.5f + 0.5f);
        wet2 = wet * ((1 - width) * 0.5f);
    }

    static constexpr int GetCombTuning(int comb)
    {
        switch (comb)
        {
        case 0:
            return combtuningL1;
        case 1:
            return combtuningL2;
        case 2:
            return combtuningL3;
        case 3:
            return combtuningL4;
        case 4:
            return combtuningL5;
        case 5:
            return combtuningL6;
        case 6:
            return combtuningL7;
        case 7:
            return combtuningL8;
        default:
            return combtuningL1;
        }
    }

    static constexpr int GetAllpassTuning(int allpass)
    {
        switch (allpass)
        {
        case 0:
            return allpasstuningL1;
        case 1:
            return allpasstuningL2;
        case 2:
            return allpasstuningL3;
        case 3:
            return allpasstuningL4;
        default:
            return allpasstuningL1;
        }
    }
};

#endif // KALI_FREEVERB_H