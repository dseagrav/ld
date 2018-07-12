/* Copyright 2016-2017 
   Daniel Seagraves <dseagrav@lunar-tokyo.net>
   Barry Silverman <barry@disus.com>

   This file is part of LambdaDelta.

   LambdaDelta is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.

   LambdaDelta is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with LambdaDelta.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <strings.h>

#include "ld.h"
#include "nubus.h"

/* NUbus Interface */
volatile int NUbus_Busy;
volatile int NUbus_error;
volatile int NUbus_acknowledge;
volatile int NUbus_master;
volatile int NUbus_trace;
volatile int NUbus_Request;
volatile nuAddr NUbus_Address;
volatile nuData NUbus_Data;

void nubus_clock_pulse(){
  if(NUbus_Busy > 0){
    NUbus_Busy--;
    // Anyone take this request?
    if(NUbus_Busy == 1 && NUbus_acknowledge == 0){
      // No. Light the error bit for at least one cycle. 
      // Doing this before releasing the bus allows
      // the master a chance to learn their request timed out
      // because otherwise it would be clobbered if another master
      // grabs the bus first.
      NUbus_error = 1;
      logmsgf(LT_NUBUS,1,"NUBUS: Timed Out: Master 0x%X Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) Ack %o\n",
	     NUbus_master,NUbus_Request,NUbus_Address.raw,NUbus_Address.raw,
	     NUbus_Data.word,NUbus_Data.word,NUbus_acknowledge);
    }
  }
}

// Put a request on the bus
void nubus_io_request(int access, int master, uint32_t address, uint32_t data){
  // Clear flags
  NUbus_error = 0;
  NUbus_acknowledge = 0;
  // Log
  if(NUbus_trace == 1){
    logmsgf(LT_NUBUS,10,"NUBUS: Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) by dev 0x%X\n",
	   access,address,address,data,data,master);
  }
  // During the PROM run, it expects to read MD one instruction after the read request.
  NUbus_Busy = 3;
  NUbus_Address.raw = address;
  NUbus_master = master;
  NUbus_Request = access;
  NUbus_Data.word = data;
}
