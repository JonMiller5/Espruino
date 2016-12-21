/*
 *  Copyright 2016 Microchip Technology Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
 
#include "system_config.h"
#include "system_definitions.h"
#include "jswrap_ccall_sanity_test.h"
#include "jsinteractive.h"
 
// https://github.com/espruino/Espruino/blob/master/libs/README.md

/*JSON{
  "type" : "class",
  "class" : "pic32mzef_wifire_sanity"
}*/

/*JSON{
  "type"     : "staticmethod",
  "class"    : "pic32mzef_wifire_sanity",
  "name"     : "test1",
  "generate" : "jswrap_pic32mzef_wifire_sanity_test1",
  "return"   : [ "JsVar", ""],
  "params"   : [
    ["p0","JsVar","parameter 0."]
  ]
}*/
JsVar* jswrap_pic32mzef_wifire_sanity_test1(JsVar* p0) {
    JsVar* retval = NULL;
    jsiConsolePrint("jswrap_pic32mzef_wifire_sanity_test1\r\n");
    if (jsvIsInt(p0))
      retval = jsvNewFromInteger((JsVarInt)(p0->varData.integer));
    else
      jsExceptionHere(JSET_ERROR, "parameter must be an integer.");
    return retval;
}

/*JSON{
  "type"     : "staticmethod",
  "class"    : "pic32mzef_wifire_sanity",
  "name"     : "test2",
  "generate" : "jswrap_pic32mzef_wifire_sanity_test2",
  "return"   : [ "JsVar", ""],
  "params"   : [
    ["p0","JsVar","parameter 0."],
    ["p1","JsVar","parameter 1."],
    ["p2","JsVar","parameter 2."]
  ]
}*/
JsVar* jswrap_pic32mzef_wifire_sanity_test2(JsVar* p0, JsVar* p1, JsVar* p2) {
    JsVar* retval = NULL;

    jsiConsolePrint("jswrap_pic32mzef_wifire_sanity_test2\r\n");
    if (NULL==p0 || !jsvIsSimpleInt(p0))
    {
      jsExceptionHere(JSET_ERROR, "parameter 0 must be an integer.");
      return NULL;
    }
    if (NULL==p1 || !jsvIsSimpleInt(p1))
    {
      jsExceptionHere(JSET_ERROR, "parameter 1 must be an integer.");
      return NULL;
    }
    if (NULL==p2 || !jsvIsSimpleInt(p2))
    {
      jsExceptionHere(JSET_ERROR, "parameter 2 must be an integer.");
      return NULL;
    }

    retval = jsvNewFromInteger((JsVarInt)(p0->varData.integer) + 
                               (JsVarInt)(p1->varData.integer) + 
                               (JsVarInt)(p2->varData.integer));
    return retval;
}

/*JSON{
  "type"     : "staticmethod",
  "class"    : "pic32mzef_wifire_sanity",
  "name"     : "test3",
  "generate" : "jswrap_pic32mzef_wifire_sanity_test3",
  "return"   : [ "JsVar", ""],
  "params"   : [
    ["p0","int32","parameter 0."],
    ["p1","int32","parameter 1."],
    ["p2","int32","parameter 2."],
    ["p3","int32","parameter 3."],
    ["p4","int32","parameter 4."],
    ["p5","int32","parameter 5."],
    ["p6","int32","parameter 6."],
    ["p7","int32","parameter 7."]
  ]
}
 */ 
/* MAX_ARGS is defined as 12 in jsnative.c */
JsVar* jswrap_pic32mzef_wifire_sanity_test3(int p0, int p1, int p2, int p3, int p4,
                                             int p5, int p6, int p7)
{
    JsVar* retval = NULL;

    jsiConsolePrint("jswrap_pic32mzef_wifire_sanity_test3\r\n");

    retval = jsvNewFromInteger((JsVarInt)(p0) + 
                               (JsVarInt)(p1) + 
                               (JsVarInt)(p2) + 
                               (JsVarInt)(p3) + 
                               (JsVarInt)(p4) + 
                               (JsVarInt)(p5) +
                               (JsVarInt)(p6) +
                               (JsVarInt)(p7)
                               );
    return retval;
}

// Let's define the JavaScript class that will contain our `world()` method. We'll call it `Hello`
/*JSON{
  "type" : "class",
  "class" : "Hello"
}*/

// Now, we define the `jswrap_hello_world` to be a `staticmethod` on the `Hello` class
/*JSON{
  "type" : "staticmethod",
  "class" : "Hello",
  "name" : "world",
  "generate" : "jswrap_hello_world"
}*/
void jswrap_hello_world() {
    jsiConsolePrint("Hello World!\r\n");
}

