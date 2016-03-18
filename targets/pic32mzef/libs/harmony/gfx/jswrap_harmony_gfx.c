
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "system_config.h"
#include "system_definitions.h"

#include "gfx_primitive.h"

#include "jswrap_harmony_gfx.h"
#include "jsinteractive.h" // Pull inn the jsiConsolePrint function


// Let's define the JavaScript class that will contain our `world()` method. We'll call it `Hello`
/*JSON{
  "type" : "class",
  "class" : "Harmony_gfx"
}*/

// Now, we define the `jswrap_hello_world` to be a `staticmethod` on the `Hello` class
/*JSON{
  "type" : "staticmethod",
  "class" : "Harmony_gfx",
  "name" : "RectangleFillDraw",
  "generate" : "jswrap_GFX_RectangleFillDraw",
  "params" : [
    ["gfxIndex","JsVar","parameter 0."]
  ]
}*/
#if 1
GFX_STATUS jswrap_GFX_RectangleFillDraw(
                            SYS_MODULE_INDEX gfxIndex,
                            unsigned int left,
                            unsigned int top,
                            unsigned int right,
                            unsigned int bottom)
{
  return GFX_RectangleFillDraw(gfxIndex,
                               left,
                               top,
                               right,
                               bottom);
}

/*JSON{
  "type" : "staticmethod",
  "class" : "Harmony_gfx",
  "name" : "DrawItem",
  "generate" : "jswrap_GFX_RectangleFillDraw",
  "params" : [
    ["itemId","JsVar","parameter 0."]
  ]
}*/

bool jswrap_GFX_HGC_DrawItem(int itemId)
{
  return GFX_HGC_DrawItem(itemId);
}


/*JSON{
  "type" : "staticmethod",
  "class" : "Harmony_gfx",
  "name" : "DrawScreen_Primitives",
  "generate" : "jswrap_GFX_HGC_DrawScreen_Primitives",
  "params" : [
    ["screenId","JsVar","parameter 0."]
  ]
}*/
bool jswrap_GFX_HGC_DrawScreen_Primitives(unsigned int screenId)
{
    return GFX_HGC_DrawScreen_Primitives(screenId);
}


/*JSON{
  "type" : "staticmethod",
  "class" : "Harmony_gfx",
  "name" : "ChangeScreen",
  "generate" : "jswrap_GFX_HGC_ChangeScreen",
  "params" : [
    ["screenId","JsVar","parameter 0."]
  ]
}*/
bool jswrap_GFX_HGC_ChangeScreen(unsigned int*  screenId )
{
//    __builtin_software_breakpoint();
    return GFX_HGC_ChangeScreen(*screenId);
}

#endif
