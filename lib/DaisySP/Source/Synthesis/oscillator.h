#pragma once
#ifndef DSY_OSCILLATOR_H
#define DSY_OSCILLATOR_H
#include <stdint.h>
#include "Utility/dsp.h"
#ifdef __cplusplus

namespace daisysp
{
/** Synthesis of several waveforms, including polyBLEP bandlimited waveforms.
*/
class Oscillator
{
  public:
    Oscillator() {}
    ~Oscillator() {}
    /** Choices forff output waveforms, POLYBLEP are appropriately labeled. Others are naive forms.
    */
    enum
    {
        WAVE_SIN,
        WAVE_TRI,
        WAVE_SAW,
        WAVE_RAMP,
        WAVE_SQUARE,
        WAVE_POLYBLEP_TRI,
        WAVE_POLYBLEP_SAW,
        WAVE_POLYBLEP_SQUARE,
        WAVE_NOISE,
        WAVE_LAST,
    };


    /** Initializes the Oscillator

        \param sample_rate - sample rate of the audio engine being run, and the frequency that the Process function will be called.

        Defaults:
        - freq_ = 100 Hz
        - amp_ = 0.5
        - waveform_ = sine wave.
    */
    void Init(float sample_rate)
    {
        sr_        = sample_rate;
        sr_recip_  = 1.0f / sample_rate;
        freq_      = 100.0f;
        amp_       = 0.5f;
        pw_        = 0.5f;
        pw_rad_    = pw_ * TWOPI_F;
        phase_     = 0.0f;
        phase_inc_ = CalcPhaseInc(freq_);
        waveform_  = WAVE_SIN;
        eoc_       = true;
        eor_       = true;
    }


    /** Changes the frequency of the Oscillator, and recalculates phase increment.
    */
    inline void SetFreq(const float f)
    {
        freq_      = f;
        phase_inc_ = CalcPhaseInc(f);
    }


    /** Sets the amplitude of the waveform.
    */
    inline void SetAmp(const float a) { permamp_ = amp_ = a; }
    /** Sets the waveform to be synthesized by the Process() function.
    */
    inline void SetWaveform(const uint8_t wf)
    {
        waveform_ = wf < WAVE_LAST ? wf : WAVE_SIN;
    }

    /** Sets the pulse width for WAVE_SQUARE and WAVE_POLYBLEP_SQUARE (range 0 - 1)
      */
    inline void SetPw(const float pw)
    {
        pw_     = fclamp(pw, 0.0f, 1.0f);
        pw_rad_ = pw_ * TWOPI_F;
    }

    /** Returns true if cycle is at end of rise. Set during call to Process.
    */
    inline bool IsEOR() { return eor_; }

    /** Returns true if cycle is at end of cycle. Set during call to Process.
    */
    inline bool IsEOC() { return eoc_; }

    /** Returns true if cycle rising.
    */
    inline bool IsRising() { return phase_ < PI_F; }

    /** Returns true if cycle falling.
    */
    inline bool IsFalling() { return phase_ >= PI_F; }

    /** Processes the waveform to be generated, returning one sample. This should be called once per sample period.
    */
    float Process();

    void SetCoefficients(float a, float b) {
        coeffa_ = a;
        coeffb_ = b;
    }

    void CheckLock() {
        if(eoc_)
            lock_ = true;
    }

    void ReleaseLock() {
        lock_ = false;
    }
    


    /** Adds a value 0.0-1.0 (mapped to 0.0-TWO_PI) to the current phase. Useful for PM and "FM" synthesis.
    */
    void PhaseAdd(float _phase) { phase_ += (_phase * TWOPI_F); }

    void PhaseAdj(float _phase) { 
        float diff = (phase_ - _phase);
        if(IsEOR())
                phase_ += -diff * 0.01f; 
    }

    float PhaseGet() { return phase_; }
    /** Resets the phase to the input argument. If no argumeNt is present, it will reset phase to 0.0;
    */
    void Reset(float _phase = 0.0f) { lock_ = false; phase_ = _phase; }

    float Offset = 0.0f;

  private:
    float   CalcPhaseInc(float f);
    uint8_t waveform_;
    float   amp_, permamp_, freq_, pw_, pw_rad_;
    float   sr_, sr_recip_, phase_, phase_inc_;
    float   last_out_, last_freq_;
    float   offset_;
    float   coeffa_, coeffb_;
    bool    eor_, eoc_;
    bool    lock_;
};
} // namespace daisysp
#endif
#endif
