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
 'name' : "PIC32MZ EF WIFIRE",
 'link' :  [ "http://chipkit.net/wiki/index.php?title=ChipKIT_Wi-Fire" ],
 'default_console' : "EV_SERIAL4",
 'default_console_baudrate' : "9600",
 'variables' : 5450,
 'binary_name' : 'espruino_%v_pic32mz_ef_wifire.bin',
};
#FIXME for PIC32MZ
chip = {
  'part' : "32MZ2048EFG100",
  'family' : "PIC32MZEF",
  'package' : "TQFP100",
  'ram' : 512,
  'flash' : 2048,
  'speed' : 200,
  'usart' : 6,
  'spi' : 6,
  'i2c' : 5,
  'adc' : 48,
  'dac' : 0,
};


devices = {
  'BTN1' : { 'pin' : 'B12' },
  'BTN2' : { 'pin' : 'B13' },
  # 'BTN3' : { 'pin' : 'B14' }, Not allowed due to multiplexing with USART
  'LED1' : { 'pin' : 'G6' },
  'LED2' : { 'pin' : 'D4' },
  'LED3' : { 'pin' : 'B11' },
  'LED4' : { 'pin' : 'G15' },
};

def get_pins():
  pins = [
      { "name":"PG6", "sortingname":"G6", "port":"G", "num":"6", "functions":{}, "csv":{} },
      { "name":"PD4", "sortingname":"D4", "port":"D", "num":"4", "functions":{}, "csv":{} },
      { "name":"PB11", "sortingname":"B11", "port":"B", "num":"11", "functions":{}, "csv":{} },
      { "name":"PG15", "sortingname":"G15", "port":"G", "num":"15", "functions":{}, "csv":{} },
      { "name":"PB12", "sortingname":"B12", "port":"B", "num":"12", "functions":{}, "csv":{} },
      { "name":"PB13", "sortingname":"B13", "port":"B", "num":"13", "functions":{}, "csv":{} },
      { "name":"PB14", "sortingname":"B14", "port":"B", "num":"14", "functions":{}, "csv":{} },
      { "name":"PD1", "sortingname":"D1", "port":"D", "num":"1", "functions":{"SPI1_SCK":0}, "csv":{}},
      { "name":"PD14", "sortingname":"D14", "port":"D", "num":"14", "functions":{"SPI1_MISO":0}, "csv":{}},
      { "name":"PG8", "sortingname":"G8", "port":"G", "num":"8", "functions":{"SPI1_MOSI":0}, "csv":{}}
    ];
  return pins

board_css = """
""";

 
#def get_pins():
#  pins = pinutils.scan_pin_file([], 'pic32mz2048efg100.csv', 1, 5, 6)
#  return pinutils.only_from_package(pinutils.fill_gaps_in_pin_list(pins), chip["package"])
  
  
  
