/*
Copyright (c) 2020 Electrosmith, Corp

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE file or at
https://opensource.org/licenses/MIT.
*/

#pragma once
#ifndef DSY_DELAY_GRAIN_H
#define DSY_DELAY_GRAIN_H
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
namespace daisysp
{

// Generic DelayLine class
template <typename T, size_t max_size>
class DelayLine
{
  public:
    DelayLine() {}
    ~DelayLine() {}

    void Init() { Reset(); }

    void Reset()
    {
        for(size_t i = 0; i < max_size; i++)
        {
            line_[i] = T(0);
        }
        write_ptr_ = 0;
        delay_     = 1;
    }

    inline void SetDelay(size_t delay)
    {
        frac_  = 0.0f;
        delay_ = delay < max_size ? delay : max_size - 1;
    }

    inline void SetDelay(float delay)
    {
        int32_t int_delay = static_cast<int32_t>(delay);
        frac_             = delay - static_cast<float>(int_delay);
        delay_ = static_cast<size_t>(int_delay) < max_size ? int_delay : max_size - 1;
    }

    inline void Write(const T sample)
    {
        line_[write_ptr_] = sample;
        write_ptr_        = (write_ptr_ - 1 + max_size) % max_size;
    }

    inline const T Read() const
    {
        T a = line_[(write_ptr_ + delay_) % max_size];
        T b = line_[(write_ptr_ + delay_ + 1) % max_size];
        return a + (b - a) * frac_;
    }

    inline const T Read(float delay) const
    {
        int32_t delay_integral   = static_cast<int32_t>(delay);
        float   delay_fractional = delay - static_cast<float>(delay_integral);
        const T a = line_[(write_ptr_ + delay_integral) % max_size];
        const T b = line_[(write_ptr_ + delay_integral + 1) % max_size];
        return a + (b - a) * delay_fractional;
    }

    inline const T ReadLinear(float delay) const
    {
        int32_t delay_integral   = static_cast<int32_t>(delay);
        float   delay_fractional = delay - static_cast<float>(delay_integral);

        size_t read_idx_a = (write_ptr_ + delay_integral) % max_size;
        size_t read_idx_b = (read_idx_a + 1) % max_size;

        const T a = line_[read_idx_a];
        const T b = line_[read_idx_b];

        return a + (b - a) * delay_fractional;
    }

    inline const T ReadHermite(float delay, int max = 1.0f) const
    {
        int32_t delay_integral   = static_cast<int32_t>(delay);
        float   delay_fractional = delay - static_cast<float>(delay_integral);

        int32_t t = (write_ptr_ + delay_integral + max_size) % max_size;

        const T xm1 = line_[(t - 1 + max_size) % max_size];
        const T x0  = line_[t];
        const T x1  = line_[(t + 1) % max_size];
        const T x2  = line_[(t + 2) % max_size];

        float c  = 0.5f * (x1 - xm1);
        float a  = c + (x2 - x0) * 0.5f - (x1 - x0);
        float b  = (x1 - x0) - c - a;

        return ((a * delay_fractional + b) * delay_fractional + c) * delay_fractional + x0;
    }

    inline void ReadHermite4(float delays[4], T outputs[4]) const
    {
        for(int i = 0; i < 4; i++)
        {
            outputs[i] = ReadHermite(delays[i]);
        }
    }

    inline float ReadLinear4(float delays[4])
    {
        float result;
        for(int i = 0; i < 4; i++)
        {
            result += ReadLinear(delays[i]);
        }
        return result * 0.25f;
    }

    inline const T Allpass(const T sample, size_t delay, const T coefficient)
    {
        T read  = line_[(write_ptr_ + delay) % max_size];
        T write = sample + coefficient * read;
        Write(write);
        return -write * coefficient + read;
    }

    inline size_t GetWritePtr() { return write_ptr_; }

  private:
    float  frac_;
    size_t write_ptr_;
    size_t delay_;
    T      line_[max_size];
};

} // namespace daisysp
#endif