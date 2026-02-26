#pragma once
#ifndef DSY_OLED_FONTS_H
#define DSY_OLED_FONTS_H
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @addtogroup utility
    @{
*/

    /** Utility for displaying fonts on OLED displays \n 
Migrated to work with libdaisy from stm32-ssd1306 
@author afiskon on github.
*/

    /** Font struct */
    typedef struct
    {
        const uint8_t   FontWidth;  /*!< Font width in pixels */
        uint8_t         FontHeight; /*!< Font height in pixels */
        const uint16_t *data;       /*!< Pointer to data font data array */
    } FontDef;


    /** These are the different sizes of fonts (width x height in pixels per character) */
    extern FontDef Font_5x5;
    extern FontDef Font_6x8;
    extern FontDef Font_7x10;  /**< & */
    extern FontDef Font_10x10; /**< & */
    extern FontDef Font_11x18; /**< & */
    extern FontDef Font_16x26; /**< & */

    /** Initialize Font_10x10 by generating it from Font_5x5 (saves ~1.9KB flash) */
    void InitFont10x10();

#ifdef __cplusplus
}
#endif

#endif
/** @} */
