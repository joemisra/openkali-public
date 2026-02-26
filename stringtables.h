#pragma once
#ifndef STRING_TABLES_H
#define STRING_TABLES_H

#include "dpt/daisy_dpt.h"

#define MAX_STRING_TABLE 24       // Restored - minimal flash impact
#define MAX_STRING_TABLE_COUNT 20 // Keep reduced - affects flash array size

extern char *DSY_SDRAM_BSS string_tables[MAX_STRING_TABLE][MAX_STRING_TABLE_COUNT];
void InitStringTables();

enum StringTableType
{
    None,
    STLFOShapes,
    STLFOModeNames,
    STDSPModeNames,
    STLFOOutputNames,
    STVOctInputNames,
    STPolarityNames,
    STModSources,
    STGateSources,
    STEOCActions,
    STLFOTargetNames,
    STPresetsActions,
    STDistortionTypes,
    STDistortionTargets,
    STSyncMode,
    STFilterMode,
    STDelayRange,
    STRING_TABLE_TYPE_LAST
};

#endif