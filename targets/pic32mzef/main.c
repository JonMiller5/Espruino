/*
 * This file is part of Espruino, a JavaScript interpreter for Microcontrollers
 *
 * Copyright (C) 2013 Gordon Williams <gw@pur3.co.uk>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * ----------------------------------------------------------------------------
 * Platform Specific entry point
 * ----------------------------------------------------------------------------
 */
#include "platform_config.h"
#include "jsinteractive.h"
#include "jshardware.h"
#include "main.h"

#include "system_config.h"
#include "system_definitions.h"

int main() {

  jshInit();
  jsvInit();
  jsiInit(false);
  //debug_all_leds_on();

  while (1)
  {
      /* Maintain system services */
      //SYS_DEVCON_Tasks(sysObj.sysDevcon);

      /* Maintain Device Drivers */

      /* Maintain Middleware & Other Libraries */
      GFX_Tasks(sysObj.gfxObject0);
      GFX_HGC_Tasks(sysObj.gfxObject0);
      
    jsiLoop();
  }
  //jsiKill();
  //jsvKill();
  //jshKill();
}
