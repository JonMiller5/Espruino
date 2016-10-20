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
/* 
 * ----------------------------------------------------------------------------
 * Platform Specific part of Hardware interface Layer
 * ----------------------------------------------------------------------------
 */
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "jshardware.h"
#include "jstimer.h"
#include "jsutils.h"
#include "jsparse.h"
#include "jsinteractive.h"
#include "jswrap_io.h"
#include "jswrap_date.h" // for non-F1 calendar -> days since 1970 conversion

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "system_config.h"
#include "system_definitions.h"
#include "peripheral/usart/plib_usart.h"
#include "peripheral/devcon/plib_devcon.h"
#include "peripheral/spi/plib_spi.h"
#include <assert.h>
#ifndef __conditional_software_breakpoint
#define __conditional_software_breakpoint(a) ((void)(0))
#endif

#ifndef TERMINAL_USART
// The chipKIT WiFire board uses USART4 connected to an FTDI USART-to-USB device
#define TERMINAL_USART USART_ID_4
#endif

#ifdef PIC32MZ_EF_SK_USART
//  #include "uart_interface.h"
#endif

#if 0
// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************

/*** DEVCFG0 ***/

#pragma config DEBUG =      ON
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx2
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_2X
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_2X
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    EC
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF

/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_3
#pragma config FPLLRNG =    RANGE_5_10_MHZ
#pragma config FPLLICLK =   PLL_POSC
#pragma config FPLLMULT =   MUL_50
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_24MHZ

/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FMIIEN =     OFF
#pragma config FETHIO =     ON
#pragma config PGL1WAY =    OFF
#pragma config PMDL1WAY =   OFF
#pragma config IOL1WAY =    OFF
#pragma config FUSBIDIO =   OFF

/*** BF1SEQ0 ***/

#pragma config TSEQ =       0x0001
#pragma config CSEQ =       0xFFFE
#endif

#define CORE_TICK_RATE (F_CPU/2000000ul)



extern void __attribute__((nomips16, noreturn, far, weak)) __pic32_software_reset();
extern void SYS_Initialize ( void* data );
static int init = 0; // Temp hack to get jsiOneSecAfterStartup() going.

bool __pic32_WriteString(char* ptr)
{
  if (*ptr == '\0')
    return true;

    /* Write a character at a time, only if transmitter is empty */
    while (*ptr != '\0')
    {
      while(PLIB_USART_TransmitterBufferIsFull(TERMINAL_USART));
        /* Send character */
        PLIB_USART_TransmitterByteSend(TERMINAL_USART, *ptr);

        /* Increment to address of next character */
        ptr++;

    }
    return false;
}

static inline __pic32_init_core_timer(void)
{
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Configure the core timer
// clear the count reg
_CP0_SET_COUNT(0);
// set up the period in the compare reg
_CP0_SET_COMPARE(CORE_TICK_RATE);

// The Core timer should halt when we are halted at a debug breakpoint.
_CP0_BIC_DEBUG(_CP0_DEBUG_COUNTDM_MASK);

// set up the core timer interrupt with a priority of 6 and zero sub-priority
IFS0CLR = _IFS0_CTIF_MASK;
IPC0CLR = _IPC0_CTIP_MASK;
IPC0SET = (6 << _IPC0_CTIP_POSITION);
IPC0CLR = _IPC0_CTIS_MASK; 
IPC0SET = (0 << _IPC0_CTIS_POSITION);
IEC0CLR = _IEC0_CTIE_MASK;
IEC0SET = (1 << _IEC0_CTIE_POSITION);

}

// Now implement the Espruino HAL API...
void jshInit()
{
    JshUSARTInfo inf;
    PRISS = 0x76543210u;

    SYS_Initialize ( NULL );
    
    jshUSARTSetup(EV_SERIAL4,&inf);
    SYS_INT_SourceEnable(INT_SOURCE_USART_4_RECEIVE);
    __pic32_init_core_timer();
    

    jshInitDevices();
    /* Maintain system services */
    SYS_DEVCON_Tasks(sysObj.sysDevcon);

    init = 1;
}

// When 'reset' is called - we try and put peripherals back to their power-on state
void jshReset() {

}

void jshKill() {
    __pic32_software_reset();
}

// stuff to do on idle
void jshIdle() {

  if (init == 1)
  {
    jsiOneSecondAfterStartup(); // Do this the first time we enter jshIdle() after we have called jshInit() and never again.
    init = 0;
  }
  jshUSARTKick(EV_SERIAL1);

}

/// Get this IC's serial number. Passed max # of chars and a pointer to write to. Returns # of chars
int jshGetSerialNumber(unsigned char *data, int maxChars) {
  return 0;
}

// is the serial device connected?
bool jshIsUSBSERIALConnected() {
  return false;
}

volatile JsSysTime SysTickValue = 0;
/// Get the system time (in ticks)
JsSysTime jshGetSystemTime()
{
  return SysTickValue;
}

/// Set the system time (in ticks) - this should only be called rarely as it could mess up things like jsinteractive's timers!
void jshSetSystemTime(JsSysTime newTime) {
  jshInterruptOff();
  SysTickValue = newTime;
  _CP0_SET_COUNT(0);
  _CP0_SET_COMPARE(CORE_TICK_RATE);
  jshInterruptOn();
  jshGetSystemTime(); // force update of the time
}
/// Convert a time in Milliseconds to one in ticks
JsSysTime jshGetTimeFromMilliseconds(JsVarFloat ms)
{
  return (JsSysTime)(ms * 1000);
}
/// Convert ticks to a time in Milliseconds
JsVarFloat jshGetMillisecondsFromTime(JsSysTime time)
{
  return ((JsVarFloat)time)/1000;
}

// software IO functions...
void jshInterruptOff() {
    __builtin_disable_interrupts();
}
void jshInterruptOn() {
    __builtin_enable_interrupts();
}
void jshDelayMicroseconds(int microsec) {
  JsSysTime initial = SysTickValue;
  if (microsec <= 0)
  {
    return; 
  }
  while(SysTickValue < (initial + microsec));
}

void jshPinSetValue(Pin pin, bool value) {
  if (value == 1)
  {
    switch (pinInfo[pin].port)
    {
        case JSH_PORTB:
            PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_B, pinInfo[pin].pin);
            break;
        case JSH_PORTD:
            PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_D, pinInfo[pin].pin);
            break;
        case JSH_PORTG:
            PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_G, pinInfo[pin].pin);
            break;        
        case JSH_PORTH:
#if (__PIC32_PIN_COUNT__==144)
            PLIB_PORTS_PinSet (PORTS_ID_0, PORT_CHANNEL_H, pinInfo[pin].pin);
#endif
            break;

        default:
            __conditional_software_breakpoint(0);
    }
  }
  else
  {
    switch (pinInfo[pin].port)
    {
        case JSH_PORTB:
            PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_B, pinInfo[pin].pin);
            break;
        case JSH_PORTD:
            PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_D, pinInfo[pin].pin);
            break;
        case JSH_PORTG:
            PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_G, pinInfo[pin].pin);
            break;        
        case JSH_PORTH:
#if (__PIC32_PIN_COUNT__==144)
            PLIB_PORTS_PinClear (PORTS_ID_0, PORT_CHANNEL_H, pinInfo[pin].pin);
#endif
            break;
        default:
            __conditional_software_breakpoint(0);
    }
  }
}
bool jshPinGetValue(Pin pin) {
    bool pinValue = false;
    switch (pinInfo[pin].port)
    {
        case JSH_PORTB:
                pinValue = PLIB_PORTS_PinGet (PORTS_ID_0, PORT_CHANNEL_B, pinInfo[pin].pin);
                break;
        case JSH_PORTH:
#if (__PIC32_PIN_COUNT__==144)
                pinValue = PLIB_PORTS_PinGet (PORTS_ID_0, PORT_CHANNEL_H, pinInfo[pin].pin);
#endif
                break;
            default:
                __conditional_software_breakpoint(0);
    }
    return pinValue;
}
// ------

/// Set the pin state
void jshPinSetState(Pin pin, JshPinState state) {
//    __conditional_software_breakpoint(0);
}
/** Get the pin state (only accurate for simple IO - won't return JSHPINSTATE_USART_OUT for instance).
 * Note that you should use JSHPINSTATE_MASK as other flags may have been added */
JshPinState jshPinGetState(Pin pin) {
//    __conditional_software_breakpoint(0);
  return JSHPINSTATE_UNDEFINED;
}

// Returns an analog value between 0 and 1
JsVarFloat jshPinAnalog(Pin pin)
{
  return 0.0;
}
/// Returns a quickly-read analog value in the range 0-65535
int jshPinAnalogFast(Pin pin) {
  return 0;
}

JshPinFunction jshPinAnalogOutput(Pin pin, JsVarFloat value, JsVarFloat freq, JshAnalogOutputFlags flags) { // if freq<=0, the default is used
  return JSH_NOTHING;
}

void jshPinPulse(Pin pin, bool value, JsVarFloat time) {
    __conditional_software_breakpoint(0);
  //return JSH_NOTHING;
}
///< Can the given pin be watched? it may not be possible because of conflicts
bool jshCanWatch(Pin pin) {
    __conditional_software_breakpoint(0);
  return false;
}

IOEventFlags jshPinWatch(Pin pin, bool shouldWatch) {
  return EV_SERIAL4;
} // start watching pin - return the EXTI associated with it

/// Given a Pin, return the current pin function associated with it
JshPinFunction jshGetCurrentPinFunction(Pin pin) {
  return JSH_NOTHING;
}

/// Given a pin function, set that pin to the 16 bit value (used mainly for DACs and PWM)
void jshSetOutputValue(JshPinFunction func, int value) {
    __conditional_software_breakpoint(0);
}

/// Enable watchdog with a timeout in seconds
void jshEnableWatchDog(JsVarFloat timeout) {
    __conditional_software_breakpoint(0);
}

/** Check the pin associated with this EXTI - return true if it is a 1 */
bool jshGetWatchedPinState(IOEventFlags device) {
  return false;
}

bool jshIsEventForPin(IOEvent *event, Pin pin) {
  return false;
}

/** Is the given device initialised? */
bool jshIsDeviceInitialised(IOEventFlags device) {
  return false;
}

/** Set up a UART, if pins are -1 they will be guessed */
void jshUSARTSetup(IOEventFlags device, JshUSARTInfo *inf) {
  if (device == EV_USBSERIAL) {
    return; // eep!
  }
  else if (device != EV_SERIAL4)
  {
    jsExceptionHere(JSET_INTERNALERROR, "Unknown serial port device.");
    return;
  }
  

  PLIB_USART_Enable(TERMINAL_USART);
  __pic32_WriteString("\r\n\nEspruino on PIC32MZ MCU - Proof of Concept\r\n");
}
/** Kick a device into action (if required). For instance we may need
 * to set up interrupts */

void jshUSARTKick(IOEventFlags device) {

  if (device != EV_SERIAL4)
  {
	  return;
  }

  int check_valid_char = jshGetCharToTransmit(EV_SERIAL4);
  if (check_valid_char >= 0)
  {
    uint8_t character = (uint8_t) check_valid_char;
    while(PLIB_USART_TransmitterBufferIsFull(TERMINAL_USART));
    /* Send character */
    PLIB_USART_TransmitterByteSend(TERMINAL_USART, character);
  }

}

/** Set up SPI, if pins are -1 they will be guessed */
void jshSPISetup(IOEventFlags device, JshSPIInfo *inf)
{
    if (device != EV_SPI1)
    {
      __builtin_software_breakpoint();
      return;
    }

    PLIB_SPI_Enable(SPI_ID_1);
}

/** Send data through the given SPI device (if data>=0), and return the result
 * of the previous send (or -1). If data<0, no data is sent and the function
 * waits for data to be returned */
int jshSPISend(IOEventFlags device, int data)
{
  uint8_t recv;

  if (device != EV_SPI1)
  {
      __builtin_software_breakpoint();
      return;
  }

//  __builtin_software_breakpoint();

  if (data >= 0)
  {
    if (!PLIB_SPI_TransmitBufferIsFull(SPI_ID_1)) {
        PLIB_SPI_BufferWrite(SPI_ID_1, (uint8_t)data);
    }
  }
  else
  {
    while (!PLIB_SPI_ReceiverBufferIsFull(SPI_ID_1));
  }
  
  recv = PLIB_SPI_BufferRead(SPI_ID_1);
  
  return recv;
}

/** Send 16 bit data through the given SPI device. */
void jshSPISend16(IOEventFlags device, int data)
{
  if (device != EV_SPI1)
  {
      __builtin_software_breakpoint();
      return;
  }

//  __builtin_software_breakpoint();

  while (PLIB_SPI_TransmitBufferIsFull(SPI_ID_1));
  PLIB_SPI_BufferWrite16bit(SPI_ID_1, (uint16_t)data);
}

/** Set whether to send 16 bits or 8 over SPI */
void jshSPISet16(IOEventFlags device, bool is16)
{
  if (device != EV_SPI1)
  {
      __builtin_software_breakpoint();
      return;
  }

//  __builtin_software_breakpoint();

  if (is16)
  {
    PLIB_SPI_CommunicationWidthSelect(SPI_ID_1, SPI_COMMUNICATION_WIDTH_16BITS);
  }
  else
  {
    PLIB_SPI_CommunicationWidthSelect(SPI_ID_1, SPI_COMMUNICATION_WIDTH_8BITS);
  }
}

/** Set whether to use the receive interrupt or not */
void jshSPISetReceive(IOEventFlags device, bool isReceive)
{
  if (device != EV_SPI1)
  {
      __builtin_software_breakpoint();
      return;
  }
  // TODO

}

/** Wait until SPI send is finished, and flush all received data */
void jshSPIWait(IOEventFlags device)
{
  if (device != EV_SPI1)
  {
      __builtin_software_breakpoint();
      return;
  }

  /* Loop until not sending */
  while (PLIB_SPI_IsBusy(SPI_ID_1));
}

/** Set up I2C, if pins are -1 they will be guessed */
void jshI2CSetup(IOEventFlags device, JshI2CInfo *inf) {

}
/** Addresses are 7 bit - that is, between 0 and 0x7F. sendStop is whether to send a stop bit or not */
void jshI2CWrite(IOEventFlags device, unsigned char address, int nBytes, const unsigned char *data, bool sendStop) {

}
void jshI2CRead(IOEventFlags device, unsigned char address, int nBytes, unsigned char *data, bool sendStop) {

}

/// Return start address and size of the flash page the given address resides in. Returns false if no page
bool jshFlashGetPage(uint32_t addr, uint32_t *startAddr, uint32_t *pageSize) {
  return false;
}
/// Erase the flash page containing the address
void jshFlashErasePage(uint32_t addr) {

}
/// Read data from flash memory into the buffer
void jshFlashRead(void *buf, uint32_t addr, uint32_t len) {

}
/// Write data to flash memory from the buffer
void jshFlashWrite(void *buf, uint32_t addr, uint32_t len) {

}

/// Save contents of JsVars into Flash
void jshSaveToFlash() {

}
/// Load contents of JsVars from Flash
void jshLoadFromFlash() {

}
/// Returns true if flash contains something useful
bool jshFlashContainsCode() {
  return false;
}

/// Enter simple sleep mode (can be woken up by interrupts). Returns true on success
bool jshSleep(JsSysTime timeUntilWake) {
//  __WFI(); // Wait for interrupt is a hint instruction that suspends execution until one of a number of events occurs.
  return true;
}


/// Start the timer and get it to interrupt after 'period'
void jshUtilTimerStart(JsSysTime period) {
}
/// Reschedult the timer (it should already be running) to interrupt after 'period'
void jshUtilTimerReschedule(JsSysTime period) {
}
/// Stop the timer
void jshUtilTimerDisable() {

}

// On SYSTick interrupt, call this
inline void __attribute__((always_inline)) jshDoSysTick() {
  /* Handle the delayed Ctrl-C -> interrupt behaviour (see description by EXEC_CTRL_C's definition)  */
  if (execInfo.execute & EXEC_CTRL_C_WAIT)
    execInfo.execute = (execInfo.execute & ~EXEC_CTRL_C_WAIT) | EXEC_INTERRUPTED;
  if (execInfo.execute & EXEC_CTRL_C)
    execInfo.execute = (execInfo.execute & ~EXEC_CTRL_C) | EXEC_CTRL_C_WAIT;
}

// the temperature from the internal temperature sensor
JsVarFloat jshReadTemperature()
{
  return 0.0;
}
// The voltage that a reading of 1 from `analogRead` actually represents
JsVarFloat jshReadVRef()
{
  return 0.0;
}
/* Get a random number - either using special purpose hardware or by
 * reading noise from an analog input. If unimplemented, this should
 * default to `rand()` */
unsigned int jshGetRandomNumber() {
  return 0;
}

void __ISR_AT_VECTOR(_CORE_TIMER_VECTOR, IPL6SRS) __attribute__((no_fpu)) CoreTimerHandler(void)
{
    unsigned long old_count, period;
    old_count = _CP0_GET_COUNT();
    
    SysTickValue++;
    jshDoSysTick();
    
    // clear the interrupt flag
    IFS0CLR = _IFS0_CTIF_MASK;

    // update the period
    period = CORE_TICK_RATE;
    period += old_count;
    _CP0_SET_COMPARE(period);
}


void __ISR_AT_VECTOR(_UART4_TX_VECTOR, IPL1SRS) __attribute__((no_fpu)) _IntHandlerDrvUsartTransmitInstance0(void)
{
    /* TODO: Add code to process interrupt here */

    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_4_TRANSMIT);
}
void __ISR_AT_VECTOR(_UART4_RX_VECTOR, IPL1SRS) _IntHandlerDrvUsartReceiveInstance0(void)
{
    __conditional_software_breakpoint(0);
	if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_4_RECEIVE))
    {
      uint8_t data;
        /* Make sure receive buffer has data available */
        if (PLIB_USART_ReceiverDataIsAvailable(TERMINAL_USART))
        {
            /* Get the data from the buffer */
            data = PLIB_USART_ReceiverByteReceive(TERMINAL_USART);
        }
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_4_RECEIVE);
        jshPushIOCharEvent(EV_SERIAL4, (char) data);
    }
    else
    {
        __conditional_software_breakpoint(0);
    }

}
void __ISR_AT_VECTOR(_UART4_FAULT_VECTOR, IPL1SRS) __attribute__((no_fpu)) _IntHandlerDrvUsartErrorInstance0(void)
{
//    __conditional_software_breakpoint(0);
    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_4_ERROR);
}
