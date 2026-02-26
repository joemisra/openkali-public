#include "stringtables.h"
#include <cstring>
#include "dpt/daisy_dpt.h"
#include "util/oled_fonts.h"

char *DSY_SDRAM_BSS string_tables[MAX_STRING_TABLE][MAX_STRING_TABLE_COUNT];

// Store string data in flash but copy to SDRAM at init
#define NUM_SOURCE_STRING_TABLES 18
static const char *const source_strings[NUM_SOURCE_STRING_TABLES][MAX_STRING_TABLE_COUNT] = {
    {},
    {"Sin", "Tri", "Saw", "Ramp", "[ ]", "Ptri", "PSaw", "P[ ]", "Noise"},
    {"Core", "S+H", "T+H", "RandRst", "RndShp", "Jitter", "Osc", "Glac", "Clocks", "Follow", "Side", "Env", "Turing", "Raw", "MIDI"},
    {
        "Str8", "pipo", "Str8 un", "Reverse", "Reson", "Chorus", "Knuth", "Grain", "GranOct", "GranTxt", "GranShim", "GranCry", "SpBlur", "Fluid"
        /*, "Shimmer", "Parvati", "Knives", "Shards",
        "Splat", "Grampa", "Gramma",
        //"Water", "Fire", "LCR", "FirePong", "Dub",
        "Sinstereo", "Saturate", "Foldback", "Diode", "Modulo", "Quantize", "OnlyReverb"*/
    },
    {"1L", "2L", "3L", "1R", "2R", "3R", "LL", "RR", "??"},
    {"1", "2", "FM1", "FM2"},
    {"+", "+/-", "-", "-/+"},
    {"Pitch1", "Pitch2", "Meta1", "Meta2", "FM1", "FM2", "DIV1", "DIV2", "Gate1", "Gate2", "1L", "2L", "3L", "1R", "2R", "3R", "LL", "RR"},
    {"Off", "1", "2"},
    {"-", "Restart", "Halt", "Toggle", "Flip", "RndWv", "Scramble", "Teleport"},
    {"1L", "2L", "3L", "1R", "2R", "3R", "Any", "Other", "All"},
    {"Save", "Load", "Mutate", "Medusa"},
    {"Sin", "Fold", "Tanh", "Quant", "Diode", "HClip", "SClip", "Mod"},
    {"Off", "Dry", "Wet", "Both"},
    {"Internal", "Ext CV", "MIDI Clk"},
    {"FIR", "IIR", "Off"},
    {"Prec", "Studio", "Amb", "Loop", "Exp"}, // Delay range presets
};

// Buffer to store all string data in SDRAM
// Reduced from 16384 to save flash memory overhead
static char DSY_SDRAM_BSS string_buffer[8192];
static size_t buffer_pos = 0;

void InitStringTables()
{
    buffer_pos = 0;

    // Initialize all entries to nullptr first
    for (int table = 0; table < MAX_STRING_TABLE; table++)
    {
        for (int entry = 0; entry < MAX_STRING_TABLE_COUNT; entry++)
        {
            string_tables[table][entry] = nullptr;
        }
    }

    // Copy strings from source_strings to SDRAM buffer
    for (int table = 0; table < NUM_SOURCE_STRING_TABLES; table++)
    {
        for (int entry = 0; entry < MAX_STRING_TABLE_COUNT; entry++)
        {
            if (source_strings[table][entry] != nullptr)
            {
                // Copy string to SDRAM buffer
                size_t len = strlen(source_strings[table][entry]) + 1;
                if (buffer_pos + len < sizeof(string_buffer))
                {
                    strcpy(&string_buffer[buffer_pos], source_strings[table][entry]);
                    string_tables[table][entry] = &string_buffer[buffer_pos];
                    buffer_pos += len;
                }
                else
                {
                    string_tables[table][entry] = nullptr; // Buffer overflow protection
                }
            }
            else
            {
                string_tables[table][entry] = nullptr;
            }
        }
    }
}

// Generate 10x10 font from 5x5 at runtime to save ~1.9KB flash
static uint16_t DSY_SDRAM_BSS Font10x10_generated[95 * 10];
static bool Font10x10_initialized = false;

void InitFont10x10()
{
    if (Font10x10_initialized)
        return;

    // Generate 10x10 font by doubling each 5x5 character
    for (int ch = 0; ch < 95; ch++)
    {
        for (int row = 0; row < 5; row++)
        {
            uint16_t src = Font_5x5.data[ch * 5 + row];
            // Double horizontally
            uint16_t doubled = 0;
            for (int bit = 0; bit < 5; bit++)
            {
                if (src & (1 << (15 - bit)))
                {
                    doubled |= (3 << (14 - bit * 2));
                }
            }
            // Write twice (doubling vertically)
            Font10x10_generated[ch * 10 + row * 2] = doubled;
            Font10x10_generated[ch * 10 + row * 2 + 1] = doubled;
        }
    }

    // Point Font_10x10 to our generated data
    Font_10x10.data = Font10x10_generated;
    Font10x10_initialized = true;
}
