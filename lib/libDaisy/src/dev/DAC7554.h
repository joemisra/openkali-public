#pragma once
#ifndef DSY_DEV_DAC_7554_H
#define DSY_DEV_DAC_7554_H /**< Macro */
#include <stdlib.h>
#include <stdint.h>
#include <vector>
#include "daisy_core.h"

using namespace std;


namespace daisy
{

/** @addtogroup analog_digital_conversion
    @{ 
*/

/** 
    Driver for DAC7554, based on code from driver for DAC8568
    Based on Code from Westlicht Performer   - https://westlicht.github.io/performer/
    Port for Daisy by Making Sound Machines  - https://github.com/makingsoundmachines

    You will need adjusted SPI settings for this to work:
    In libdaisy\src\per\spi.cpp
    hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH; // was SPI_POLARITY_LOW;
    hspi1.Init.NSS               = SPI_NSS_SOFT; // was SPI_NSS_HARD_OUTPUT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // was SPI_BAUDRATEPRESCALER_8;
*/


class Dac7554
{
  public:
    //uint16_t cmd = (2 << 14) | (address << 12) | data;
    //buff[buff_index++] = (cmd >> 8) & 0xff;
    //buff[buff_index++] = cmd & 0xff;
    struct __attribute__((packed)) DAC7554TransmitBuffer
    {
        /** register address */
        uint16_t cmd1;
        uint16_t cmd2;
    };

    /** GPIO Pins that need to be used independent of peripheral used. */
    enum Pins
    {
        SYNC = 7, /**< Sync pin. */
    };

    enum class Type
    {
        DAC7554,
    };



    static constexpr int Channels = 4; // #define CONFIG_DAC_CHANNELS 8

    typedef int16_t Value;

    vector<uint8_t> buf;

    Dac7554(Type type = Type::DAC7554) {}
    ~Dac7554() {}


    // configuration currently only uses SPI1, w/ soft chip select.

    /** 
    Takes an argument for the pin cfg
    \param pin_cfg should be a pointer to an array of Dac7554::NUM_PINS dsy_gpio_pins
    */
    void Init();
    void Set(int channel, Value value) { _values[channel] = value; }
    void Write(int16_t gogo[4]);
    void WriteDac7554();

    bool dac_ready;
    bool lock;
    Value    _values[Channels];
  private:
    void Clear(void* context, int result);
    void Reset();
    void SetInternalRef(bool enabled);

    enum ClearCode
    {
        ClearZeroScale = 0,
        ClearMidScale  = 1,
        ClearFullScale = 2,
        ClearIgnore    = 3,
    };

    void SetClearCode(ClearCode code);

    uint32_t _dataShift = 0;
    };
    /** @} */
} // namespace daisy

#endif