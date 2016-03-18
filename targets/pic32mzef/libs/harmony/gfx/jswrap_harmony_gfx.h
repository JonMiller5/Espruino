#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "system_config.h"
#include "system_definitions.h"


extern GFX_STATUS jswrap_GFX_RectangleFillDraw(
                            SYS_MODULE_INDEX gfxIndex,
                            unsigned int left,
                            unsigned int top,
                            unsigned int right,
                            unsigned int bottom);

bool jswrap_GFX_HGC_DrawItem(int itemId);
bool jswrap_GFX_HGC_DrawScreen_Primitives(unsigned int screenId);
bool jswrap_GFX_HGC_ChangeScreen(unsigned int*  screenId );
