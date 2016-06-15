/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Copyright (c) 2016
 * Author: Vitor Matos
 * Based on rosserial_arduino's ArduinoHardware.h with the intent to make it  work on TI's Energia.
 */

#ifndef TIVACHARDWARE_H
#define TIVACHARDWARE_H

#include <Energia.h>
#include <Ethernet.h>

class TivaCHardware {
  public:
    TivaCHardware()
    {
    }
    
    void setConnection(byte *mac, IPAddress &server)
    {
      this->server = server;
      for (int i=0; i<6; i++) {
        this->mac[i] = mac[i];
      }
    }
    
    IPAddress getLocalIP(){
      return Ethernet.localIP();
    }

    void init(){
      if (Ethernet.begin(mac) == 0) {
        // no point in carrying on, so do nothing forevermore:
        for(;;);
      }
      
      this->tcp.connect(this->server, 11411);
    }

    int read(){
      if (this->tcp.connected())
        return tcp.read();
      else
        this->tcp.connect(this->server, 11411);
        
      return -1;
    };
    
    void write(uint8_t* data, int length){
      tcp.write(data, length);
    }

    unsigned long time(){return millis();}

  protected:
    EthernetClient tcp;
    IPAddress server; 
    byte mac[6];
};

#endif  // TIVAHARDWARE_H
