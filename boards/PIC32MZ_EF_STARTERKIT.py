#!/bin/false
# This file is part of Espruino, a JavaScript interpreter for Microcontrollers
#
# Copyright (C) 2013 Gordon Williams <gw@pur3.co.uk>
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
# ----------------------------------------------------------------------------------------
# This file contains information for a specific board - the available pins, and where LEDs,
# Buttons, and other in-built peripherals are. It is used to build documentation as well
# as various source and header files for Espruino.
# ----------------------------------------------------------------------------------------

import pinutils;
info = {
 'name' : "PIC32MZ EF STARTER KIT",
 'link' :  [ "http://www.microchip.com/Developmenttools/ProductDetails.aspx?PartNO=DM320007-C" ],
 'default_console' : "EV_SERIAL2", # FIXME: This was S2 because of pin conflict. Not sure if it's really an issue?
 'variables' : 5450,
 'binary_name' : 'espruino_%v_pic32mz_ef_sk.bin',
};
#FIXME for PIC32MZ
chip = {
  'part' : "32MZ2048EFM144",
  'family' : "PIC32MZEF",
  'package' : "",
  'ram' : 512,
  'flash' : 2048,
  'speed' : 200,
  'usart' : 6,
  'spi' : 6,
  'i2c' : 5,
  'adc' : 48,
  'dac' : 0,
};
# left-right, or top-bottom order
board = {
};
devices = {
};

board_css = """
""";

def get_pins():
  pins = pinutils.generate_pins(0,7)  
  # just fake pins D0 .. D7
  return pins
