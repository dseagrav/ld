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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <strings.h>
#include <pthread.h>

#include "ld.h"
#include "nubus.h"
#include "lambda_cpu.h"
#include "sdu.h"
#include "vcmem.h"
#include "mem.h"

/* NUbus Master Mutex */
volatile int bus_waiters = 0;
pthread_mutex_t nubus_master_mutex = PTHREAD_MUTEX_INITIALIZER;

/* NUbus Interface */
volatile int NUbus_Busy;
volatile int NUbus_error;
volatile int NUbus_acknowledge;
volatile int NUbus_master;
volatile int NUbus_trace;
volatile int NUbus_Request;
volatile nuAddr NUbus_Address;
volatile nuData NUbus_Data;
volatile nuData NUbus_Block[4]; // HACK FOR BLOCK TRANSFERS

void take_nubus_mastership(){
  // Increment waiters count
  // bus_waiters++;
  // Seize the nubus master mutex
  int result = pthread_mutex_lock(&nubus_master_mutex);
  if(result != 0){
    logmsgf(LT_NUBUS,0,"Unable to take nubus mastership: %s\n",strerror(result));
    exit(-1);
  }
  // Decrement waiters count
  // if(bus_waiters > 0){
  //   bus_waiters--;
  // }
}

void release_nubus_mastership(){
  // int waiters = bus_waiters;
  int result;
  // Free the bus if it isn't already.
  NUbus_Busy = 0;
  // Release the nubus master mutex
  result = pthread_mutex_unlock(&nubus_master_mutex);
  if(result != 0){
    logmsgf(LT_NUBUS,0,"Unable to release nubus mastership: %s\n",strerror(result));
    exit(-1);
  }
  /*
  if(waiters > 0){
    struct timespec sleep_time;
    // printf("%d waiters\n",waiters);
    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = 200;
    nanosleep(&sleep_time,NULL); // Let someone else take it
  }
  */
}

// SINGLE-OPERATION TRANSFER
void nubus_xfer(int access, int master, uint32_t address, uint32_t data){
  // Does a transfer as a single operation.
  // DEVICES CONSIDER REQUEST DATA VALID IF NUBUS-BUSY IS 2
  // Clear flags
  NUbus_error = 0;
  NUbus_acknowledge = 0;
  // Log
  if(NUbus_trace == 1){
    logmsgf(LT_NUBUS,10,"NUBUS: Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) by dev 0x%X\n",
	   access,address,address,data,data,master);
  }
  // Set up the request
  NUbus_Busy = 2;
  NUbus_Address.raw = address;
  NUbus_master = master;
  NUbus_Request = access;
  NUbus_Data.word = data;
  if((address&0x3) == 2){
    // Flush block
    NUbus_Block[0].word = 0;
    NUbus_Block[1].word = 0;
    NUbus_Block[2].word = 0;
    NUbus_Block[3].word = 0;
  }
  // Issue Lambda cache write checks
  if((access&0x01) == 1){
    if(master != 0xF0){ cache_write_check(access,0,address,data); }
    if(master != 0xF4){ cache_write_check(access,1,address,data); }
  }

  // Drive the target card
  switch(NUbus_Address.Card){
  case 0xF0: // LAMBDA 0
    lambda_nubus_pulse(0);
    break;

  case 0xF9: // MEMORY
  case 0xFA:
#ifdef CONFIG_2X2
  case 0xFD: // MORE MEMORY FOR 2X2
  case 0xFE:
#endif
    mem_clock_pulse();
    break;

  case 0xF8: // VCMEM 0
    vcmem_clock_pulse(0);
    break;

#ifdef CONFIG_2X2
  case 0xF4: // LAMBDA 1
    lambda_nubus_pulse(1);
    break;

  case 0xFC: // VCMEM 1
    vcmem_clock_pulse(1);
    break;
#endif

  case 0xFF: // SDU
    sdu_clock_pulse();
    break;
  }
  if(NUbus_acknowledge == 0){
    // Error. Time out immediately.
    NUbus_error = 1;
    logmsgf(LT_NUBUS,1,"NUBUS: Timed Out: Master 0x%X Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) Ack %o\n",
	    NUbus_master,NUbus_Request,NUbus_Address.raw,NUbus_Address.raw,
	    NUbus_Data.word,NUbus_Data.word,NUbus_acknowledge);
  }
  NUbus_Busy = 1; // Signal done.
}

void nubus_clock_pulse(){
  if(NUbus_Busy > 0){
    // Drive slaves here
    switch(NUbus_Address.Card){
    case 0xF0: // LAMBDA 0
      lambda_nubus_pulse(0);
      break;

#ifdef CONFIG_2X2
    case 0xF4: // LAMBDA 1
      lambda_nubus_pulse(1);
      break;
#endif

    case 0xF8: // VCMEM 0
      vcmem_clock_pulse(0);
      break;

#ifdef CONFIG_2X2
    case 0xFC: // VCMEM 1
      vcmem_clock_pulse(1);
#endif

    case 0xF9: // MEM 0
    case 0xFA: // MEM 1
#ifdef CONFIG_2X2
    case 0xFD: // MEM 2
    case 0xFE: // MEM 3
#endif
      mem_clock_pulse();
      break;

    case 0xFF: // SDU
      sdu_clock_pulse();
      break;
    }

    // Advance bus state
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
  // CONSISTENCY TEST
  if(NUbus_Busy != 0){ // && NUbus_master != master){
    logmsgf(LT_NUBUS,0,"NUBUS ACCESS CLASH: Busy %d\n",NUbus_Busy);
    logmsgf(LT_NUBUS,0,"NUBUS: Active Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) by dev 0x%X\n",
	   NUbus_Request,NUbus_Address.raw,NUbus_Address.raw,NUbus_Data.word,NUbus_Data.word,NUbus_master);
    logmsgf(LT_NUBUS,0,"NUBUS: New Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) by dev 0x%X\n",
	   access,address,address,data,data,master);
    exit(-1);
  }
  // Issue Lambda cache write checks
  if((access&0x01) == 1){
    if(master != 0xF0){ cache_write_check(access,0,address,data); }
    if(master != 0xF4){ cache_write_check(access,1,address,data); }
  }
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
  if((address&0x3) == 2){
    // Flush block
    NUbus_Block[0].word = 0;
    NUbus_Block[1].word = 0;
    NUbus_Block[2].word = 0;
    NUbus_Block[3].word = 0;
  }
}
