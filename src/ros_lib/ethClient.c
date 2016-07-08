/*
 * Copyright (c) 2016, All rights reserved.
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

#include <stdbool.h>
#include <stdint.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/systick.h>
#include <driverlib/pin_map.h>  
#include <utils/uartstdio.h> // trash
#include <utils/ringbuf.h>
#include "ethClient.h"


#define INFO_UART
//~ #define DEBUG_UART

#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE 256
#endif

struct ConnetionInfo g_sConnection;

static tRingBufObject g_rxBuffer;
uint8_t g_ui8rxBufferData[RX_BUFFER_SIZE];

void ethInit(uint32_t ui32SysClkFreq, const char *ipAddress)
{
  // Configure the hardware MAC address for Ethernet Controller filtering of
  // incoming packets.  The MAC address will be stored in the non-volatile
  // USER0 and USER1 registers.
  MAP_FlashUserGet(&g_sConnection.ui32User0, &g_sConnection.ui32User1);
  if ((g_sConnection.ui32User0 == 0xffffffff) || (g_sConnection.ui32User1 == 0xffffffff))
  {
    // We should never get here.  This is an error if the MAC address has
    // not been programmed into the device.  Exit the program.
    // Let the user know there is no MAC address
    MAP_GPIOPinWrite(LED_PORT, LED1, LED1);
    while(1);
  }

  // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
  // address needed to program the hardware registers, then program the MAC
  // address into the Ethernet Controller registers.
  g_sConnection.pui8MACArray[0] = ((g_sConnection.ui32User0 >>  0) & 0xff);
  g_sConnection.pui8MACArray[1] = ((g_sConnection.ui32User0 >>  8) & 0xff);
  g_sConnection.pui8MACArray[2] = ((g_sConnection.ui32User0 >> 16) & 0xff);
  g_sConnection.pui8MACArray[3] = ((g_sConnection.ui32User1 >>  0) & 0xff);
  g_sConnection.pui8MACArray[4] = ((g_sConnection.ui32User1 >>  8) & 0xff);
  g_sConnection.pui8MACArray[5] = ((g_sConnection.ui32User1 >> 16) & 0xff);
  
  // Set the interrupt priorities.  We set the SysTick interrupt to a higher
  // priority than the Ethernet interrupt to ensure that the file system
  // tick is processed if SysTick occurs while the Ethernet handler is being
  // processed.  This is very likely since all the TCP/IP and HTTP work is
  // done in the context of the Ethernet interrupt.
  MAP_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
  
  // Configure for ethernet LED function.
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  MAP_GPIOPinConfigure(GPIO_PF0_EN0LED0);
  MAP_GPIOPinConfigure(GPIO_PF4_EN0LED1);
  GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
  
  g_sConnection.sServerIP.addr = ipaddr_addr(ipAddress);
  
  RingBufInit(&g_rxBuffer, g_ui8rxBufferData, RX_BUFFER_SIZE);
  
  ethReset();
  
  // Initialize the lwIP library, using DHCP.
  lwIPInit(ui32SysClkFreq, g_sConnection.pui8MACArray, 0, 0, 0, IPADDR_USE_DHCP);
}

static err_t ethPoll_cb(void * pvArg, struct tcp_pcb *psPcb)
{
#ifdef DEBUG_UART
  UARTprintf("poll\n"); 
#endif
  // Check if server is alive
  
  return(ERR_OK);
}

static err_t ethReceive_cb(void *pvArg, struct tcp_pcb *psPcb, struct pbuf *psBuf, err_t iErr)
{
#ifdef DEBUG_UART
  UARTprintf("rcv\n");
#endif
  // The host closed the connection
  if(psBuf == 0)
  {
    tcp_close(psPcb);
    if (psPcb == g_sConnection.psTCP) g_sConnection.psTCP = 0;
    g_sConnection.eState = iEthIdle;
    
    return(ERR_OK);
  }

  struct pbuf *psBuf_head = psBuf;
  // Copy from tcp to ringbuffer
  while (psBuf_head != NULL)
  {
    if (RingBufFree(&g_rxBuffer) >= psBuf_head->len)
    {
      RingBufWrite(&g_rxBuffer, psBuf_head->payload, psBuf_head->len);
      psBuf_head = psBuf_head->next;
    } 
    else
      break;
  };
  
  // Inform lwIP we got length from buffer
  tcp_recved(psPcb, psBuf->tot_len);
  
  // Free the memory space allocated for this receive.
  pbuf_free(psBuf);
  
  // Return.
  return(ERR_OK);
}

static err_t ethSent_cb(void *pvArg, struct tcp_pcb *psPcb, u16_t ui16Len)
{
#ifdef DEBUG_UART
  UARTprintf("sent\n");
#endif
  
  // Return OK.
  return (ERR_OK);
}

static void ethError_cb(void *vPArg, err_t iErr)
{
#ifdef INFO_UART
  UARTprintf("Error %d, %s\n", iErr, lwip_strerr(iErr));
#endif

  if ( ERR_IS_FATAL(iErr) )
  {
    ethReset();
    return;
  }
  
  if ( ERR_TIMEOUT == iErr )
  {
    g_sConnection.eState = iEthIdle;
    return;
  }
}

static err_t ethConnected_cb(void *pvArg, struct tcp_pcb *psPcb, err_t iErr)
{
  // Check if there was a TCP error.
  if(iErr != ERR_OK)
  {
    // Clear out all of the TCP callbacks.
    tcp_sent(psPcb, NULL);
    tcp_recv(psPcb, NULL);
    tcp_err(psPcb, NULL);
    tcp_poll(psPcb, NULL, 10);

    // Close the TCP connection.
    tcp_close(psPcb);

    if(psPcb == g_sConnection.psTCP)
    {
      g_sConnection.psTCP = 0;
    }
#ifdef DEBUG_UART
    UARTprintf("tcp connected error\n");
#endif
    // And return.
    return (ERR_OK);
  }
#ifdef DEBUG_UART
  UARTprintf("tcp connected\n");
#endif
  // Setup the TCP receive function.
  tcp_recv(psPcb, ethReceive_cb);
  // Setup the TCP error function.
  tcp_err(psPcb, ethError_cb);
  // Setup the TCP sent callback function.
  //~ tcp_sent(psPcb, ethSent_cb);
  // Setup the TCP poll callback. 10 /2 = 5 seconds
  //~ tcp_poll(psPcb, ethPoll_cb, 10);
  // Connection is complete.
  g_sConnection.eState = iEthTCPConnectComplete;

  // Return a success code.
  return(ERR_OK);
}

int ethReadByte(void)
{
  if (!RingBufEmpty(&g_rxBuffer))
    return RingBufReadOne(&g_rxBuffer);
  else
    return -1;
}

void ethReset(void)
{
  g_sConnection.eState = iEthNoConnection;
  
  g_sConnection.sLocalIP.addr = 0;
  
  if(g_sConnection.psTCP)
  {
    tcp_sent(g_sConnection.psTCP, NULL);
    tcp_recv(g_sConnection.psTCP, NULL);
    tcp_err(g_sConnection.psTCP, NULL);
    tcp_poll(g_sConnection.psTCP, NULL, 10);

    tcp_close(g_sConnection.psTCP);
    g_sConnection.psTCP = 0;
  }
}

void ethWrite(uint8_t* data, int length)
{
  if (tcp_sndbuf(g_sConnection.psTCP) >= length)
  {
    err_t eError = tcp_write(g_sConnection.psTCP, data, length, TCP_WRITE_FLAG_COPY);
    
    if(eError == ERR_OK)
    {
      // Find out what we can send and send it
      tcp_output(g_sConnection.psTCP);
    }
  }
}

err_t ethConnect(void)
{
  err_t eTCPReturnCode;

  if(g_sConnection.psTCP)
  {
      // Initially clear out all of the TCP callbacks.
      tcp_sent(g_sConnection.psTCP, NULL);
      tcp_recv(g_sConnection.psTCP, NULL);
      tcp_err(g_sConnection.psTCP, NULL);
      tcp_poll(g_sConnection.psTCP, NULL, 10);

      // Make sure there is no lingering TCP connection.
      tcp_close(g_sConnection.psTCP);
  }
  
  // Create a new TCP socket.
  g_sConnection.psTCP = tcp_new();

  // Attempt to connect to the server
  eTCPReturnCode = tcp_connect(g_sConnection.psTCP, &g_sConnection.sServerIP, 11411, ethConnected_cb);
  // Setup the TCP error function.
  tcp_err(g_sConnection.psTCP, ethError_cb);
  
  g_sConnection.eState = iEthTCPConnectWait;

  return(eTCPReturnCode);
}

void lwIPHostTimerHandler(void) 
{
  uint32_t ui32NewIPAddress;

  // Get the current IP address.
  ui32NewIPAddress = lwIPLocalIPAddrGet();
  
  // See if the IP address has changed.
  if(ui32NewIPAddress != g_sConnection.sLocalIP.addr)
  {
    // See if there is an IP address assigned.
    if(ui32NewIPAddress == 0xffffffff)
    {
      ethReset();
      // No link yet
      g_sConnection.eState = iEthNoConnection;
    }
    else if(ui32NewIPAddress == 0)
    {
#ifdef INFO_UART
      UARTprintf("Waiting for IP\n");
#endif
      // There is no IP address, so indicate that the DHCP process is running
      g_sConnection.eState = iEthDHCPWait;
    }
    else
    {
      // Show IP after DHCP
      if (g_sConnection.eState == iEthDHCPWait)
      {
        g_sConnection.sLocalIP.addr = ui32NewIPAddress;
#ifdef INFO_UART
        UARTprintf("Got IP: %s\n", ipaddr_ntoa(&g_sConnection.sLocalIP));
#endif
      }
      g_sConnection.eState = iEthDHCPComplete;
      // Jump straight to Idle
      g_sConnection.eState = iEthIdle;
    }
    
    // Save the new IP address.
    g_sConnection.sLocalIP.addr = ui32NewIPAddress;
  }
  
  // If its not connected, try to connect to server
  if (g_sConnection.eState == iEthIdle)
  {
#ifdef DEBUG_UART
    UARTprintf("Try connect\n");
#endif
    ethConnect();
  } 
}
