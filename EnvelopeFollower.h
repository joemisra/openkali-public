#pragma once
#ifndef ENVELOPEFOLLOWER_H
#define ENVELOPEFOLLOWER_H

#include <cmath>
#include <cstdlib>

template <int Channels = 2, typename Value = double>
class EnvelopeFollower
{
public:
    EnvelopeFollower()
    {
        for (int i = 0; i < Channels; i++)
            m_env[i] = 0;
    }

    Value operator[](int channel) const
    {
        return m_env[channel];
    }

    void Setup(int sampleRate, double attackMs, double releaseMs)
    {
        m_a = pow(0.01, 1.0 / (attackMs * sampleRate * 0.001));
        m_r = pow(0.01, 1.0 / (releaseMs * sampleRate * 0.001));
    }

    void Process(size_t samples, Value **src)
    {
        for (int i = 0; i < Channels; i++)
        {
            const Value *cur = src[i];
            double e = m_env[i];

            for (int n = samples; n; n--)
            {
                double v = std::abs(*cur++);
                if (v > e)
                    e = m_a * (e - v) + v;
                else
                    e = m_r * (e - v) + v;
            }
            m_env[i] = e;

            if (++last32posstepped == 24)
            {
                last32[i][last32pos] = e * -1.f;
                if (++last32pos == 64)
                    last32pos = 0;
                last32posstepped = 0;
            }
        }
    }

    float last32[2][64];
    int last32pos = 0;
    int last32posstepped = 0;
    double m_env[Channels];

protected:
    double m_a; // Attack coefficient
    double m_r; // Release coefficient
};

#endif