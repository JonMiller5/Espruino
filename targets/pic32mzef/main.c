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
#include "app.h"



int main() {
  jshInit();
  jsvInit();
  jsiInit(false);
  //debug_all_leds_on();

  while (1)
  {
    jsiLoop();
  }
  //jsiKill();
  //jsvKill();
  //jshKill();
}

/* Stub for now */
void APP_Initialize ( void )
{
}
