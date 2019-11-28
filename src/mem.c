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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <strings.h>
#include <pthread.h>

#include "ld.h"
#include "nubus.h"

// Board is 16MB
// WAIT A MINUTE! 16MB IS THE ENTIRE SLOT SPACE! WHAT GIVES?
// Easy - The board isn't really 16MB! The top 4K is omitted!
#define RAM_TOP   0xFFF000 // "16MB"

// Memory
#ifdef CONFIG_2X2
uint8_t MEM_RAM[2][RAM_TOP];
#else
uint8_t MEM_RAM[1][RAM_TOP];
#endif
static uint8_t rom_string[0x1B] = "LMI 16-MEGABYTE MEMORY V1.0";
// Don't use the ROM image anymore, no longer needed.
// uint8_t MEM_ROM[2048]; // TI ROMs are 2KB
// uint8_t MEM_ROM[512]; // LMI ROMs are 512 bytes

// Externals
extern int ld_die_rq;

// Functions
void mem_init(){
  bzero(MEM_RAM[0],RAM_TOP);
#ifdef CONFIG_2X2
  bzero(MEM_RAM[1],RAM_TOP);
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
    if(NUbus_Address.Card == 0xF9
#ifdef CONFIG_2X2
       || NUbus_Address.Card == 0xFC
#endif
       ){
      int Card = 0;
      switch(NUbus_Address.Card){
      case 0xF9:
	Card = 0; break;
#ifdef CONFIG_2X2
      case 0xFC:
	Card = 1; break;
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

	  case 2: // Block Transfer (USED ONLY BY LAMBDA WHEN FILLING CACHE)
	    {
	      uint32_t blockaddr = NUbus_Address.Addr&0xFFFFF0;
	      NUbus_Data.word = *(uint32_t *)(MEM_RAM[Card]+(NUbus_Address.Addr&0xFFFFFC)); // Fill as normal
	      NUbus_Block[0].word = *(uint32_t *)(MEM_RAM[Card]+(blockaddr));
	      NUbus_Block[1].word = *(uint32_t *)(MEM_RAM[Card]+(blockaddr+4));
	      NUbus_Block[2].word = *(uint32_t *)(MEM_RAM[Card]+(blockaddr+8));
	      NUbus_Block[3].word = *(uint32_t *)(MEM_RAM[Card]+(blockaddr+12));
	    }
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
	    logmsgf(LT_MEM,0,"MEM8: BLOCK WRITE REQUESTED\n");
	    exit(-1);
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

	// Some kind of configuration register
      case 0xFFF7FC ... 0xFFF7FF:
        if((NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ)){
	  NUbus_Data.word = 0;
	  NUbus_acknowledge=1;
	  return;
	}
	if((NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE)){
	  // Bit 0x08 is LED?
	  if(!(NUbus_Data.word == 0x08 && NUbus_Address.Byte == 0) && NUbus_Data.word != 0x00){
	    logmsgf(LT_MEM,0,"MEM: CONF REG WRITE: DATA = 0x%X\n",NUbus_Data.word);
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	break;

	// Configuration ROM
      case 0xFFF800 ... 0xFFFFFF:
        if((NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ)
	   && NUbus_Address.Byte == 0){
          uint32_t rom_addr = (NUbus_Address.Addr-0xfff800)/4;
	  if(rom_addr <= 0x1B){
	    NUbus_Data.word = rom_string[rom_addr];
	  }else{
	    NUbus_Data.word = 0;
	  }
          NUbus_acknowledge=1;
          return;
        }
        break;

      // Uhoh!
      default:
	logmsgf(LT_MEM,0,"RAM: Unimplemented address 0x%X (0x%X), op %X\n",
		NUbus_Address.Addr,NUbus_Address.raw,NUbus_Request);
	// lambda_dump(DUMP_ALL);
	ld_die_rq=1;
	break;
      }
    }
    // Request finished or not ours
  }
}
