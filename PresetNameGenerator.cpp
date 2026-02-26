/*
 * Created by Claude for the Kali firmware project
 * https://claude.ai/chat
 * Simplified for flash memory optimization - compact word list
 */

#include "PresetNameGenerator.h"
#include <cstdio>
#include <cstring>

// Compact word list - 3-5 letter words, stored in flash (read-only)
// Total: ~60 words * 6 bytes avg = ~360 bytes (vs 5KB+ for original vectors)
static const char *const word_list[] = {
    // Weapons & Items (short names only)
    "Blade", "Sword", "Spear", "Staff", "Axe", "Bow", "Mace", "Whip",
    "Dagger", "Saber", "Lance", "Harp", "Shield", "Helm", "Crown",
    // Elements & Magic
    "Fire", "Ice", "Bolt", "Wind", "Dark", "Light", "Holy", "Chaos",
    "Mana", "Aura", "Nova", "Echo", "Void", "Flux", "Prism",
    // Mystical concepts
    "Rune", "Sigil", "Glyph", "Charm", "Ward", "Hex", "Moon", "Star",
    "Dawn", "Dusk", "Shade", "Mist", "Ember", "Frost", "Storm",
    // Metals & Materials
    "Iron", "Steel", "Gold", "Silver", "Jade", "Ruby", "Pearl",
    "Crystal", "Diamond", "Onyx", "Opal"};

static const int word_count = sizeof(word_list) / sizeof(word_list[0]);

void PresetNameGenerator::generateName(DPT *patch, char *buffer, size_t bufsize, size_t minLength, size_t maxLength)
{
    // Clamp to 4-8 character output
    if (minLength < 4)
        minLength = 4;
    if (maxLength > 8)
        maxLength = 8;
    if (minLength > maxLength)
        minLength = maxLength;

    // Pick 1-2 random words and combine
    int num_words = (patch->GetRandomValue() % 2) + 1; // 1 or 2 words
    buffer[0] = '\0';

    for (int i = 0; i < num_words && strlen(buffer) < maxLength; i++)
    {
        const char *word = word_list[patch->GetRandomValue() % word_count];
        size_t word_len = strlen(word);
        size_t current_len = strlen(buffer);

        // Check if we can fit this word
        if (current_len + word_len < bufsize - 1)
        {
            strcat(buffer, word);
        }
    }

    // Truncate to max length if needed
    if (strlen(buffer) > maxLength)
    {
        buffer[maxLength] = '\0';
    }

    // If too short, pad with a vowel and consonant
    while (strlen(buffer) < minLength && strlen(buffer) < bufsize - 1)
    {
        static const char vowels[] = "AEIOU";
        buffer[strlen(buffer)] = vowels[patch->GetRandomValue() % 5];
        buffer[strlen(buffer) + 1] = '\0';
    }
}
