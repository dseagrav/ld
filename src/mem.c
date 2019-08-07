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

// Let's do 8MB
#define RAM_TOP 0x800000 // 8MB
// #define RAM_TOP 0x1000000 // 16MB

// Memory
#ifdef CONFIG_2X2
unsigned char MEM_RAM[4][RAM_TOP];
#else
unsigned char MEM_RAM[2][RAM_TOP];
#endif
// static uint8_t prom_string[0x12] = "LMI 8-MEGABYTE";
uint8_t MEM_ROM[2048];

// Externals
extern int ld_die_rq;

// Functions
void mem_init(){
  bzero(MEM_RAM[0],RAM_TOP);
  bzero(MEM_RAM[1],RAM_TOP);
#ifdef CONFIG_2X2
  bzero(MEM_RAM[2],RAM_TOP);
  bzero(MEM_RAM[3],RAM_TOP);
#endif
}

uint8_t debug_mem_read(uint32_t addr){
  return(MEM_RAM[0][addr]);
};

void debug_mem_write(uint32_t addr, uint8_t data){
  MEM_RAM[0][addr] = data;
};

void mem_clock_pulse(){
  // If the bus is busy and not acknowledged...
  if(NUbus_Busy == 2 && NUbus_acknowledge == 0){
    // Is it us?
    if(NUbus_Address.Card == 0xF9 || NUbus_Address.Card == 0xFA
#ifdef CONFIG_2X2
       || NUbus_Address.Card == 0xFD || NUbus_Address.Card == 0xFE
#endif
       ){
      int Card = 0;
      switch(NUbus_Address.Card){
      case 0xF9:
	Card = 0; break;
      case 0xFA:
	Card = 1; break;
#ifdef CONFIG_2X2
      case 0xFD:
	Card = 2; break;
      case 0xFE:
	Card = 3; break;
#endif
      }
      // Yes, answer
      switch(NUbus_Address.Addr){
      case 0x000000 ... RAM_TOP:
	if(NUbus_Request == VM_READ){ // Read four bytes
	  switch(NUbus_Address.Byte){
	  case 1: // Read Low Half
	    NUbus_Data.hword[0] = *(uint16_t *)(MEM_RAM[Card]+(NUbus_Address.Addr-1));
	    break;

	  case 2: // Block Transfer (ILLEGAL)
	    logmsgf(LT_MEM,0,"MEM: BLOCK READ REQUESTED\n");
	    ld_die_rq=1;
	    break;

	  case 3: // Read High Half
	    NUbus_Data.hword[1] = *(uint16_t *)(MEM_RAM[Card]+(NUbus_Address.Addr-1));
	    break;

	  case 0:
	    // Full word read
	    NUbus_Data.word = *(uint32_t *)(MEM_RAM[Card]+NUbus_Address.Addr);
	    break;
	  }
	  NUbus_acknowledge=1;
          return;
	}
	if(NUbus_Request == VM_WRITE){
	  switch(NUbus_Address.Byte){
	  case 1: // Write low half
	    *(uint16_t *)(MEM_RAM[Card]+(NUbus_Address.Addr-1)) = NUbus_Data.hword[0];
	    break;

	  case 2: // BLOCK TRANSFER (ILLEGAL)
	    logmsgf(LT_MEM,0,"MEM8: BLOCK TRANSFER REQUESTED\n");
	    ld_die_rq=1;
	    break;

	  case 3: // Write high half
	    *(uint16_t *)(MEM_RAM[Card]+(NUbus_Address.Addr-1)) = NUbus_Data.hword[1];
	    break;

	  case 0: // Full Word
	    *(uint32_t *)(MEM_RAM[Card]+NUbus_Address.Addr) = NUbus_Data.word;
	    break;
	  }
	  NUbus_acknowledge=1;
          return;
	}
	if(NUbus_Request == VM_BYTE_READ){
	  // BYTE READ
	  NUbus_Data.byte[NUbus_Address.Byte] = MEM_RAM[Card][NUbus_Address.Addr];
	  NUbus_acknowledge=1;
	  return;
	}	
	if(NUbus_Request == VM_BYTE_WRITE){
	  MEM_RAM[Card][NUbus_Address.Addr] = NUbus_Data.byte[NUbus_Address.Byte];
	  NUbus_acknowledge=1;
          return;
	}
	break;
	
	/*
      case 0xffdfe0: // ???
        if(NUbus_Request == VM_BYTE_READ){
	  logmsgf(LT_MEM,10,"MEM: CSR READ?\n");
	  NUbus_Data.word = 0;
          NUbus_acknowledge=1;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
	  logmsgf(LT_MEM,10,"MEM: CSR WRITE? DATA = 0x");
	  writeH8(NUbus_Data.byte[0]);
	  logmsgf(LT_MEM,10,"\n");
          NUbus_acknowledge=1;
          return;
        }
	break;
      case 0xffdfe7: // Read Check Bit, 8b, read-only
      case 0xffdfe8: // ECC output latch, 32b, read-only
      case 0xffdfeb: // ECC register, 8b, read-only
      case 0xffdfec: // ECC input latch, 32b, write-only
      case 0xffdff0: // ECC check bit, 8b, write-only
	*/

      case 0xffdfe5: // Mem control, 8b, read-write
        if(NUbus_Request == VM_BYTE_READ){
	  logmsgf(LT_MEM,10,"MEM: MEM CONTROL READ?\n");
	  NUbus_Data.word = 0;
          NUbus_acknowledge=1;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
	  logmsgf(LT_MEM,10,"MEM: MEM CONTROL WRITE? DATA = 0x%X\n",NUbus_Data.byte[0]);
          NUbus_acknowledge=1;
          return;
        }
	break;
	
	// Configuration ROM
      case 0xFFE000 ... 0xFFFFFF:
        if((NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ)
	   && NUbus_Address.Byte == 0){
          uint32_t rom_addr = (NUbus_Address.Addr-0xffe000)/4;
	  rom_addr ^= 0x400; // Hax
          NUbus_Data.word = MEM_ROM[rom_addr];
          NUbus_acknowledge=1;
          return;
        }
        break;

	/*
      case 0xfff800 ... 0xffff63:
	if(NUbus_Request == VM_READ){
	  uint8_t prom_addr = (NUbus_Address.Addr-0xfff800)/4;
	  if(prom_addr <= 0x12){	  
	    NUbus_Data.word = prom_string[prom_addr];
	  }else{
	    NUbus_Data.word = 0;
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_READ){
	  uint8_t prom_addr = (NUbus_Address.Addr-0xfff800)/4;
	  if(prom_addr <= 0x12){	  
	    NUbus_Data.word = prom_string[prom_addr];
	  }else{
	    NUbus_Data.word = 0;
	  }
	  NUbus_acknowledge=1;
	  return;
	}	
	break;
	
	// Serial number? Model number?
      case 0xFFFF64 ... 0xFFFFFF:
	if(NUbus_Request == VM_READ){
	  NUbus_Data.word = 0;
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_READ){
	  NUbus_Data.byte[NUbus_Address.Byte] = 0;
	  NUbus_acknowledge=1;
	  return;
	}
	break;
	*/
	
      // Uhoh!
      default:
	logmsgf(LT_MEM,0,"RAM: Unimplemented address 0x%X (0x%X)\n",
	       NUbus_Address.Addr,NUbus_Address.raw);
	lambda_dump(DUMP_ALL);
	ld_die_rq=1;      
	break;
      }
    }
    // Request finished or not ours
  }
}
