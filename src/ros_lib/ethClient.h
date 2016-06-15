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
 
#ifndef ROS_LIB_ETHCLIENT_H
#define ROS_LIB_ETHCLIENT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <utils/lwiplib.h>

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

// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
#define ETHERNET_INT_PRIORITY   0xC0

struct ConnetionInfo { 
  uint32_t ui32heartbeat_hz;
  
  uint32_t ui32User0;
  uint32_t ui32User1;
  uint8_t pui8MACArray[6];
  
  struct ip_addr sLocalIP;
  struct ip_addr sServerIP;
  struct tcp_pcb *psTCP;
  
  volatile enum
  {
      iEthNoConnection,
      iEthDHCPWait,
      iEthDHCPComplete,
      iEthTCPConnectWait,
      iEthTCPConnectComplete,
      iEthQueryWait,
      iEthTCPOpen,
      iEthIdle
  } eState;
};

void ethInit(uint32_t ui32SysClkFreq, const char *ipAddress);
void ethReset(void);
void ethWrite(uint8_t* data, int length);
int ethReadByte(void);

#ifdef __cplusplus
}
#endif

#endif  // ROS_LIB_ETHCLIENT_H
