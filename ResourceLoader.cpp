#include "ResourceLoader.h"
#include "KaliOptions.h"
#include "stringtables.h"
#include "tools/output.h"

using namespace daisy;
using namespace dpt;

void LoadResources(DPT &patch)
{
    (void)patch; // QSPI not yet used
    InitOptionRules();
    InitStringTables();
    InitFontData();
}
