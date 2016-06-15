/*
 * Copyright (c) 2016  All rights reserved.
 * Author: Vitor Matos
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *   3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************
//
// Bare minimum hardware resources allocated for rosserial tcp communication.
// * 1 heartbeat LED if desired
// * Systick Interrupt handler
//
//*****************************************************************************

#ifndef ROS_LIB_TIVAC_HARDWARE_H
#define ROS_LIB_TIVAC_HARDWARE_H

#include <stdbool.h>
#include <stdint.h>
extern "C"
{
  #include <inc/hw_types.h>
  #include <inc/hw_memmap.h>
  #include <inc/hw_ints.h>
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/rom.h>
  #include <driverlib/rom_map.h>
  #include <driverlib/systick.h>
  #include <driverlib/pin_map.h>  
  #include <driverlib/uart.h>
  #include <utils/uartstdio.h>
  #include "ethClient.h"
}

#define SYSTICKHZ  100UL
#define SYSTICKMS  (1000 / SYSTICKHZ)

// Should only be for TM4C129
#ifdef TARGET_IS_TM4C129_RA0
#define LED1        GPIO_PIN_1  // D1 LED
#define LED_PORT    GPIO_PORTN_BASE
#define LED_PERIPH  SYSCTL_PERIPH_GPION
#ifndef TM4C129FREQ
#error "Must define system clock frequency on: TM4C129FREQ"
#endif
#else
#error "This package only works for Tiva C Launchpad Connected"
#endif

extern volatile uint32_t g_ui32milliseconds;
extern volatile uint32_t g_ui32heartbeat;

class TivaCHardware
{
  public:
    TivaCHardware() {}

    void init(const char *portName)
    {
      this->ui32SysClkFreq = TM4C129FREQ;

      // Setup LEDs
    #ifdef LED_HEARTBEAT
      MAP_SysCtlPeripheralEnable(LED_PERIPH);
      MAP_GPIOPinTypeGPIOOutput(LED_PORT, LED1);
    #endif

      // Make sure the main oscillator is enabled because this is required by
      // the PHY.  The system must have a 25MHz crystal attached to the OSC
      // pins.  The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
      // frequency is 10MHz or higher.
      MAP_SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

      // Set up timer such that it produces one tick for each millisecond
      SysTickIntRegister(TivaCHardware::SystickIntHandler);
      MAP_SysTickPeriodSet(this->ui32SysClkFreq/SYSTICKHZ);
      MAP_SysTickEnable();
      MAP_SysTickIntEnable();
      
      this->ConfigUART(115200); // trash?
      
      // Init ethernet tcp/ip 
      ethInit(this->ui32SysClkFreq, portName);

      // Enable processor interrupts.
      MAP_IntMasterEnable();
    }

    // read a byte -1 = failure
    int read()
    {
      return ethReadByte();
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length)
    {
      ethWrite(data, length);
    }

    // returns milliseconds since start of program
    uint32_t time()
    {
      return g_ui32milliseconds;
    }

    // Timing variables and System Tick interrupt handler.
    static void SystickIntHandler()
    {
      g_ui32milliseconds += SYSTICKMS;
#ifdef LED_HEARTBEAT
      if (++g_ui32heartbeat >= SYSTICKHZ)
      {
        MAP_GPIOPinWrite(LED_PORT, LED1, MAP_GPIOPinRead(LED_PORT, LED1)^LED1);
        g_ui32heartbeat = 0;
      }
#endif
      // Call the lwIP timer handler.
      lwIPTimer(SYSTICKMS);
    }

    // System frequency
    uint32_t ui32SysClkFreq;
    uint32_t getSysClkFreq(void)
    {
      return this->ui32SysClkFreq;
    }

    // Not really accurate ms delay. But good enough for our purposes.
    // For a more elaborate delay check out ``Energia/hardware/lm4f/cores/lm4f/wiring.c``
    void delay(uint32_t ms)
    {
      while (ms > 500)
      {
        MAP_SysCtlDelay(this->ui32SysClkFreq/3/1000 * 500);
        ms -= 500;
      }
      MAP_SysCtlDelay(this->ui32SysClkFreq/3/1000 * ms);
    }
    
    void ConfigUART(uint32_t baud)
    {
      // Enable the GPIO Peripheral used by the UART.
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
      // Enable UART0
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
      // Configure GPIO Pins for UART mode.
      MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
      MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
      MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
      // Use the internal 16MHz oscillator as the UART clock source.
      UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
      UARTStdioConfig(0, baud, 16000000);
    }
};
#endif  // ROS_LIB_TIVAC_HARDWARE_H
