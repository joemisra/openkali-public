#ifndef STEREO_SPREAD_EFFECT_H
#define STEREO_SPREAD_EFFECT_H

#include <cmath>
#include <cstdint>

class KaliSpread {
public:
    void Init(int sampleRate_, int combDelay_, int haasDelay_) {
        sampleRate = sampleRate_;
        combDelay = combDelay_;
        haasDelay = haasDelay_;
        reset();
    }
    void setCombDelay(float delaySec) {
        combDelay = delaySec;
        updateDelays();
    }

    void sethaasDelay(float delaySec) {
        haasDelay = delaySec;
        updateDelays();
    }

    void process(float* left, float* right, uint32_t numSamples) {
        for (uint32_t i = 0; i < numSamples; ++i) {
            // Apply comb filter
            float delayedLeft = combBufferL[combReadIndex];
            float delayedRight = combBufferR[combReadIndex];

            combBufferL[combWriteIndex] = left[i] + feedback * delayedLeft;
            combBufferR[combWriteIndex] = right[i] + feedback * delayedRight;

            combReadIndex = (combReadIndex + 1) % combBufferSize;
            combWriteIndex = (combWriteIndex + 1) % combBufferSize;

            // Apply Haas effect
            float haasLeft = haasBufferL[haasReadIndex];
            float haasRight = haasBufferR[haasReadIndex];

            haasBufferL[haasWriteIndex] = left[i];
            haasBufferR[haasWriteIndex] = right[i];

            haasReadIndex = (haasReadIndex + 1) % haasBufferSize;
            haasWriteIndex = (haasWriteIndex + 1) % haasBufferSize;

            left[i] = haasLeft;
            right[i] = haasRight;
        }
    }

    void reset() {
        combReadIndex = combWriteIndex = 0;
        haasReadIndex = haasWriteIndex = 0;
        updateDelays();
        clearBuffers();
    }

private:
    void updateDelays() {
        combBufferSize = static_cast<uint32_t>(sampleRate * combDelay);
        haasBufferSize = static_cast<uint32_t>(sampleRate * haasDelay);
    }

    void clearBuffers() {
        for (uint32_t i = 0; i < combBufferSize; ++i) {
            combBufferL[i] = combBufferR[i] = 0.0f;
        }
        for (uint32_t i = 0; i < haasBufferSize; ++i) {
            haasBufferL[i] = haasBufferR[i] = 0.0f;
        }
    }

    float sampleRate;
    float combDelay;
    float haasDelay;
    float feedback = 0.5f;

    uint32_t combBufferSize = 0;
    uint32_t haasBufferSize = 0;

    uint32_t combReadIndex = 0;
    uint32_t combWriteIndex = 0;
    uint32_t haasReadIndex = 0;
    uint32_t haasWriteIndex = 0;

    static const uint32_t maxBufferSize = 48;
    float combBufferL[maxBufferSize];
    float combBufferR[maxBufferSize];
    float haasBufferL[maxBufferSize];
    float haasBufferR[maxBufferSize];
};

#endif // STEREO_SPREAD_EFFECT_H