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

/* VCMEM (console) Implementation */

#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <strings.h>

#include "ld.h"
#include "nubus.h"
#include "vcmem.h"

// State for two controllers
struct vcmemState vcS[2];
// PROM
uint8_t VCMEM_ROM[2048];
// static uint8_t prom_string[0x12] = "PROTOTYPE VCMEM";
// Externals
extern int ld_die_rq;
// Kernel interface items
extern int cp_state[2];
extern uint8_t keyboard_io_ring[2][0x100];
extern uint8_t keyboard_io_ring_top[2],keyboard_io_ring_bottom[2];
extern uint8_t mouse_io_ring[2][0x100];
extern uint8_t mouse_io_ring_top[2],mouse_io_ring_bottom[2];

// Functions
void vcmem_init(int vn,int slot){
  vcS[vn].Card = slot;
  vcS[vn].cycle_count = 0;
  // The SDU is supposed to have initialized the vcmem before we get here.
  vcS[vn].MemoryControl.MemCopy = 1;
  int x = 0;
  // Clobber A memory
  while(x < 1024*128){
    vcS[vn].AMemory[x] = 0;
    x++;
  }
}

void kb_put_char(int vn,uint8_t ch){
  keyboard_io_ring[vn][keyboard_io_ring_top[vn]] = ch;
  keyboard_io_ring_top[vn]++;
}

void vcmem_kb_int(int vn){
  // We should test the global enable in the function register first,
  // but it hasn't been touched yet
  if(vcS[vn].MemoryControl.InterruptEnabled != 0){
    vcS[vn].InterruptStatus.SerialFIFOFull = 1;
    if(NUbus_Busy == 0){    
      nubus_io_request(VM_WRITE,0xF4,vcS[vn].InterruptAddr,0xFFFFFFFF);
      // printf("VCMEM: Int generated\n");
    }else{
      printf("VCMEM: KB int while bus busy\n");
    }
  }
}

void vcmem_clock_pulse(int vn){
  // Time has passed...
  vcS[vn].cycle_count++;
  // There are 5000000 cycles per second, so 83335 per blank
  if(vcS[vn].cycle_count >= 83335){
    // We should test the global enable in the function register first, but it hasn't been touched yet
    if(vcS[vn].MemoryControl.InterruptEnabled != 0){
      // Vertical Blank
      vcS[vn].InterruptStatus.VerticalBlank = 1;
      if(NUbus_Busy == 0){
	// We can has bus
	nubus_io_request(VM_WRITE,0xF4,vcS[vn].InterruptAddr,0xFFFFFFFF);
	vcS[vn].cycle_count = 0;
	// printf("VCMEM: VB Int generated\n");
      }
    }else{
      vcS[vn].cycle_count = 0; // No interrupt, carry on
    }
  }
  // *** NUBUS SLAVE ***
  // If the bus is busy and not acknowledged...
  if(NUbus_Busy == 2 && NUbus_acknowledge == 0){
    // Is it us?
    if(NUbus_Address.Card == vcS[vn].Card){
      switch(NUbus_Address.Addr){
	// Registers
	// 00 = Function register
      case 0x00: // 00 = Function Register
	if(NUbus_Request == VM_BYTE_READ){
	  printf("VCMEM: Function Reg Byte Read\n");
	  NUbus_Data.byte[0] = vcS[vn].Function.byte[0];
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_WRITE){
	  vcS[vn].Function.byte[0] = NUbus_Data.byte[0];
	  if(NUbus_trace == 1){
	    printf("VCMEM: Function Reg Byte Write: ");
	    if(vcS[vn].Function.Reset != 0){ printf(" RESET"); }
	    if(vcS[vn].Function.BoardEnable != 0){ printf(" ENABLE"); }
	    if(vcS[vn].Function.LED != 0){ printf(" LED"); }
	    printf(" FUNCTION=%d\n",vcS[vn].Function.Function);
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	
	break;
      case 0x01: 
	if(NUbus_Request == VM_BYTE_READ){
	  printf("VCMEM: Function Reg Hi Byte Read\n");
	  NUbus_Data.byte[1] = vcS[vn].Function.byte[1];
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_WRITE){
	  printf("VCMEM: Function Reg Hi Byte Write\n");
	  vcS[vn].Function.byte[1] = NUbus_Data.byte[1];
	  NUbus_acknowledge=1;
	  return;
	}	
	break;
	
      case 0x04: // 04 = Memory Control register
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  printf("VCMEM: Memory Control Reg Read\n");
	  NUbus_Data.word = vcS[vn].MemoryControl.raw;
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  printf("VCMEM: Memory Control Reg Write: 0x%X\n",NUbus_Data.word);
	  vcS[vn].MemoryControl.raw = NUbus_Data.word;
	  set_bow_mode(vn,vcS[vn].MemoryControl.ReverseVideo ? 1 : 0); // Update black-on-white mode
	  NUbus_acknowledge=1;
	  return;
	}
	break;
      case 0x05:
	if(NUbus_Request == VM_BYTE_READ){
	  printf("VCMEM: Memory Control Reg Hi Byte Read\n");
	  NUbus_Data.byte[1] = vcS[vn].MemoryControl.byte[1];
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_WRITE){
	  printf("VCMEM: Memory Control Reg Hi Byte Write\n");
	  vcS[vn].MemoryControl.byte[1] = NUbus_Data.byte[1];
	  set_bow_mode(vn,vcS[vn].MemoryControl.ReverseVideo ? 1 : 0); // Update black-on-white mode
	  NUbus_acknowledge=1;
	  return;
	}	
	break;


      case 0x08 ... 0x0B: // Interrupt Address
	if(NUbus_Request == VM_WRITE && NUbus_Address.Byte == 0){
	  printf("VCMEM: Interrupt Address Reg Write: 0x%X\n",NUbus_Data.word);
	  vcS[vn].InterruptAddr = NUbus_Data.word;
	  NUbus_acknowledge=1;
	  return;
	}
        if(NUbus_Request == VM_BYTE_READ){
          uint32_t Word = 0;
          Word = vcS[vn].InterruptAddr;
          Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = (Word&0xFF);
          printf("VCMEM: Interrupt Address Reg Byte Read, returning 0x%X\n",
		 NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	if(NUbus_Request == VM_BYTE_WRITE){
          uint32_t Word = NUbus_Data.byte[NUbus_Address.Byte];
          uint32_t Mask = 0xFF;
	  printf("VCMEM: Interrupt Address Reg Byte Write: 0x%X\n",
		 NUbus_Data.byte[NUbus_Address.Byte]);
          Mask <<= (8*NUbus_Address.Byte);
          vcS[vn].InterruptAddr &= ~Mask;
          Word <<= (8*NUbus_Address.Byte);
          vcS[vn].InterruptAddr |= Word;
	  NUbus_acknowledge=1;
	  return;
	}
	break;

      case 0x0C ... 0x0F: // Interrupt Status Register
	if(NUbus_Request == VM_READ && NUbus_Address.Byte == 0){
	  if(NUbus_trace == 1){
	    printf("VCMEM: Interrupt Status Reg Read\n");
	  }
	  NUbus_Data.word = vcS[vn].InterruptStatus.raw;
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_READ){
	  if(NUbus_trace == 1){
	    printf("VCMEM: Interrupt Status Reg Byte Read\n");
	  }
	  NUbus_Data.byte[NUbus_Address.Byte] = vcS[vn].InterruptStatus.byte[NUbus_Address.Byte];
	  NUbus_acknowledge=1;
	  return;
	}
	break;
		
      case 0x10: // Serial Port Control Register
	if(NUbus_Request == VM_WRITE){
	  printf("VCMEM: Serial Port Control Reg Write: 0x%X\n",NUbus_Data.word);
	  vcS[vn].SerialControl.raw = NUbus_Data.word;
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_WRITE){
	  printf("VCMEM: Serial Port Control Reg Byte Write: 0x%X\n",NUbus_Data.word);
	  vcS[vn].SerialControl.byte[0] = NUbus_Data.byte[0];
	  NUbus_acknowledge=1;
	  return;
	}
        if(NUbus_Request == VM_BYTE_READ){
	  printf("VCMEM: Serial Port Control Reg Byte Read\n");
          NUbus_Data.byte[NUbus_Address.Byte] = vcS[vn].SerialControl.byte[NUbus_Address.Byte];
          NUbus_acknowledge=1;
          return;
        }
	break;
      case 0x11: // Serial Port Control Register (Hi)
	if(NUbus_Request == VM_BYTE_WRITE){
	  printf("VCMEM: Serial Port Control Reg (Hi) Byte Write: 0x%X\n",NUbus_Data.word);
	  vcS[vn].SerialControl.byte[1] = NUbus_Data.byte[1];
	  NUbus_acknowledge=1;
	  return;
	}
        if(NUbus_Request == VM_BYTE_READ){
	  printf("VCMEM: Serial Port Control Reg Byte Read\n");
          NUbus_Data.byte[NUbus_Address.Byte] = vcS[vn].SerialControl.byte[NUbus_Address.Byte];
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 0x14: // Serial Port Transmit (keytty?)
	if(NUbus_Request == VM_BYTE_WRITE){
	  printf("VCMEM: Serial Port Transmit Reg Byte Write: 0x%X\n",NUbus_Data.word);
	  // Discard data
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_READ){
	  printf("VCMEM: Serial Port Transmit Reg Read\n");
	  if(keyboard_io_ring_top[vn] != keyboard_io_ring_bottom[vn]){
	    NUbus_Data.word = 1; // We have a key
	  }else{
	    NUbus_Data.word = 0;
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	break;

      case 0x18: // Serial Port Recieve (keytty keyboard?)
	if(NUbus_Request == VM_BYTE_READ){
	  if(NUbus_trace == 1){
	    printf("VCMEM: Serial Port Recieve Reg Byte Read\n");
	  }
	  NUbus_Data.word = 0;
	  NUbus_acknowledge=1;
	  return;
	}
	break;

      case 0x30: // Serial Port A (Keyboard) Data
	// See vcmem-serial-set-up-port
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  if(NUbus_trace == 1){
	    printf("VCMEM: Serial Port A Data Read\n");
	  }
	  if(keyboard_io_ring_top[vn] != keyboard_io_ring_bottom[vn]){
	    NUbus_Data.word = keyboard_io_ring[vn][keyboard_io_ring_bottom[vn]];
	    // writeOct(keyboard_io_ring[keyboard_io_ring_bottom]);
	    // printf(" from ");
	    // writeDec(keyboard_io_ring_bottom);
	    keyboard_io_ring_bottom[vn]++;
	  }else{
	    // printf("0 from nowhere");
	    NUbus_Data.word = 0;
	  }
	  // printf("\n");
	  NUbus_acknowledge=1;
	  return;		
	}
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  if(NUbus_trace == 1){
	    printf("VCMEM: Serial Port A Data = 0x%X\n",NUbus_Data.byte[0]);
	  }
	  NUbus_acknowledge=1;
	  return;	
	}	
	break;

      case 0x34: // Serial Port A (Keyboard) Command
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  if(NUbus_trace == 1){
	    printf("VCMEM: Serial Port A Command = 0x%X\n",NUbus_Data.byte[0]);
	  }
	  NUbus_acknowledge=1;
	  return;	
	}
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  if(NUbus_trace == 1){
	    printf("VCMEM: Serial Port A Command Read: 0x");
	  }
	  if(keyboard_io_ring_top[vn] != keyboard_io_ring_bottom[vn]){
	    NUbus_Data.word = 0x5; // We have a key, tx ready
	  }else{
	    NUbus_Data.word = 0x4; // No key, tx ready
	  }
	  if(NUbus_trace == 1){
	    printf("%X\n",NUbus_Data.word);
	  }
	  NUbus_acknowledge=1;
	  return;	
	}
	break;

      case 0x38: // Serial Port B (Mouse) Data
	if(NUbus_Request == VM_READ){
	  // printf("VCMEM: Serial Port B Data Read: Taking code ");
	  if(mouse_io_ring_top[vn] != mouse_io_ring_bottom[vn]){
	    NUbus_Data.word = mouse_io_ring[vn][mouse_io_ring_bottom[vn]];
	    // writeOct(mouse_io_ring[mouse_io_ring_bottom]);
	    // printf(" from ");
	    // writeDec(mouse_io_ring_bottom);
	    mouse_io_ring_bottom[vn]++;
	  }else{
	    // printf("0 from nowhere");
	    NUbus_Data.word = 0;
	  }
	  // printf("\n");
	  NUbus_acknowledge=1;
	  return;
	}
	break;

      case 0x3C: // Serial Port B (Mouse) Command
	if(NUbus_Request == VM_WRITE){
	  if(NUbus_trace == 1){
	    printf("VCMEM: Serial Port B Command = 0x%X\n",NUbus_Data.byte[0]);
	  }
	  if(cp_state[vn] == 2){
	    cp_state[vn] = 3;
	    printf("MOUSE: INIT\n");
	  }
	  NUbus_acknowledge=1;
	  return;	
	}
	if(NUbus_Request == VM_READ){
	  if(NUbus_trace == 1){
	    printf("VCMEM: Serial Port B Command Read\n");
	  }
	  if(mouse_io_ring_top[vn] != mouse_io_ring_bottom[vn]){
	    NUbus_Data.word = 1; // We have a key
	  }else{
	    NUbus_Data.word = 0;
	  }
	  NUbus_acknowledge=1;
	  return;	
	}
	break;

	// VCMEM has two memories, A and B.
	// The A memory is attached to the nubus and the B memory is displayed.

	// The scanline table is at 0x6000-0x7FFC
	// Each entry is the offset of the start of that scanline.
      case 0x6000 ... 0x7FFF:
	if(NUbus_Request == VM_READ){
	  uint32_t Scanline = ((NUbus_Address.Addr-0x6000)>>2);
	  // This is a mono framebuffer, and our resolution is 800 x 1024.
	  // That's 80 bytes per line.
	  NUbus_Data.word = vcS[vn].SLT[Scanline]; // 0x20000+(80*Scanline);
	  if(NUbus_trace == 1){
	    printf("VCMEM: SLT Read: Line 0x%X, returning 0x%X\n",
		   Scanline,NUbus_Data.word);
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_WRITE){
	  uint32_t Scanline = ((NUbus_Address.Addr-0x6000)>>2);
	  if(NUbus_trace == 1){
	    printf("VCMEM: SLT Write: Line 0x%X w/ data 0x%X\n",
		   Scanline,NUbus_Data.word);
	  }
	  vcS[vn].SLT[Scanline] = NUbus_Data.word;
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_READ){
	  uint32_t Scanline = ((NUbus_Address.Addr-0x6000)>>2);
	  uint32_t Word = 0;
	  Word = vcS[vn].SLT[Scanline];
	  Word >>= (8*NUbus_Address.Byte);
	  NUbus_Data.byte[NUbus_Address.Byte] = (Word&0xFF);
	  if(NUbus_trace == 1){
	    printf("VCMEM: SLT Byte Read: Line 0x%X, returning 0x%X\n",
		   Scanline,NUbus_Data.byte[NUbus_Address.Byte]);
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_WRITE){
	  uint32_t Scanline = ((NUbus_Address.Addr-0x6000)>>2);
	  uint32_t Word = NUbus_Data.byte[NUbus_Address.Byte];
	  uint32_t Mask = 0xFF;
	  Mask <<= (8*NUbus_Address.Byte);
	  vcS[vn].SLT[Scanline] &= ~Mask;
	  Word <<= (8*NUbus_Address.Byte);
	  vcS[vn].SLT[Scanline] |= Word;
	  if(NUbus_trace == 1){
	    printf("VCMEM: SLT Byte Write: Line 0x%X w/ data 0x%X\n",
		   Scanline,NUbus_Data.byte[NUbus_Address.Byte]);
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	break;

	// Framebuffer is at 0x20000-0x3FFFF
      case 0x20000 ... 0x3FFFF:
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  uint32_t FBAddr = NUbus_Address.Addr-0x20000;
	  if(NUbus_Request == VM_READ){ // Read four bytes
	    switch(NUbus_Address.Byte){
	    case 1: // Read Low Half
	      NUbus_Data.hword[0] = *(uint16_t *)(vcS[vn].AMemory+(FBAddr-1));
	      break;
	      
	    case 2: // Block Transfer (ILLEGAL)
	      printf("VCMEM: BLOCK READ REQUESTED\n");
	      ld_die_rq=1;
	      break;
	      
	    case 3: // Read High Half
	      NUbus_Data.hword[1] = *(uint16_t *)(vcS[vn].AMemory+(FBAddr-1));
	      break;
	    
	    case 0:
	      // Full word read
	      NUbus_Data.word = *(uint32_t *)(vcS[vn].AMemory+FBAddr);
	      break;
	    }
	  }else{
	    // BYTE READ
	    NUbus_Data.byte[NUbus_Address.Byte] = vcS[vn].AMemory[FBAddr];
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  uint32_t FBAddr = NUbus_Address.Addr-0x20000;
	  if(NUbus_Request == VM_BYTE_WRITE){
	    switch(vcS[vn].Function.Function){
	    case 0: // XOR
	      vcS[vn].AMemory[FBAddr] ^= NUbus_Data.byte[NUbus_Address.Byte];
	      break;
	    case 1: // OR
	      vcS[vn].AMemory[FBAddr] |= NUbus_Data.byte[NUbus_Address.Byte];
	      break;
	    case 2: // AND
	      vcS[vn].AMemory[FBAddr] &= ~NUbus_Data.byte[NUbus_Address.Byte];
	      break;
	    case 3: // STORE
	      vcS[vn].AMemory[FBAddr] = NUbus_Data.byte[NUbus_Address.Byte];
	      break;
	    }
	    framebuffer_update_byte(vn,FBAddr,vcS[vn].AMemory[FBAddr]);
	  }else{
	    // WORD WRITE
	    switch(NUbus_Address.Byte){
	    case 1: // Write low half
	      switch(vcS[vn].Function.Function){
	      case 0: // XOR
		*(uint16_t *)(vcS[vn].AMemory+(FBAddr-1)) ^= NUbus_Data.hword[0]; 
		break;
	      case 1: // OR
		*(uint16_t *)(vcS[vn].AMemory+(FBAddr-1)) |= NUbus_Data.hword[0]; 
		break;
	      case 2: // AND
		*(uint16_t *)(vcS[vn].AMemory+(FBAddr-1)) &= ~NUbus_Data.hword[0]; 
		break;
	      case 3: // STORE
		*(uint16_t *)(vcS[vn].AMemory+(FBAddr-1)) = NUbus_Data.hword[0]; 
		break;
	      }
	      framebuffer_update_hword(vn,FBAddr-1,NUbus_Data.hword[0]);
	      break;
	      
	    case 2: // BLOCK TRANSFER (ILLEGAL)
	      printf("MEM8: BLOCK TRANSFER REQUESTED\n");
	      ld_die_rq=1;
	      break;
	      
	    case 3: // Write high half
	      switch(vcS[vn].Function.Function){
	      case 0: // XOR
		*(uint16_t *)(vcS[vn].AMemory+(FBAddr-1)) ^= NUbus_Data.hword[1]; 
		break;
              case 1: // OR
		*(uint16_t *)(vcS[vn].AMemory+(FBAddr-1)) |= NUbus_Data.hword[1]; 
		break;
              case 2: // AND
		*(uint16_t *)(vcS[vn].AMemory+(FBAddr-1)) &= ~NUbus_Data.hword[1]; 
		break;
              case 3: // STORE
		*(uint16_t *)(vcS[vn].AMemory+(FBAddr-1)) = NUbus_Data.hword[1]; 
		break;
	      }
	      framebuffer_update_hword(vn,FBAddr-1,NUbus_Data.hword[1]);
	      break;
	      
	    case 0: // Full Word
              switch(vcS[vn].Function.Function){
              case 0: // XOR
		*(uint32_t *)(vcS[vn].AMemory+FBAddr) ^= NUbus_Data.word;
		break;
              case 1: // OR
		*(uint32_t *)(vcS[vn].AMemory+FBAddr) |= NUbus_Data.word;
		break;
              case 2: // AND
		*(uint32_t *)(vcS[vn].AMemory+FBAddr) &= ~NUbus_Data.word;
		break;
              case 3: // STORE
		*(uint32_t *)(vcS[vn].AMemory+FBAddr) = NUbus_Data.word;
		break;
	      }
	      framebuffer_update_word(vn,FBAddr,NUbus_Data.word);
	      break;
	    }
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	break;

	// Configuration PROM
      case 0xFFE000 ... 0xFFFFFF:
	if((NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ) && NUbus_Address.Byte == 0){
	  uint32_t rom_addr = (NUbus_Address.Addr-0xffe000)/4;	  
	  NUbus_Data.word = VCMEM_ROM[rom_addr];
	  /*
	  printf("VCM: ROM READ 0x");
	  writeH32(NUbus_Address.Addr);
	  printf(" = ROM ADDR ");
	  writeH32(rom_addr);
	  printf("\n");
	  */
	  /*
	  uint8_t prom_addr = (NUbus_Address.Addr-0xfff800)/4;
	  if(prom_addr <= 0x12){
	    NUbus_Data.word = prom_string[prom_addr];
	  }else{
	    NUbus_Data.word = 0;
	  }
	  */	  
	  NUbus_acknowledge=1;
	  return;
	}
	break;

	// Serial number? Model number?
	/*
      case 0xFFFF64 ... 0xFFFFFF:
	if(NUbus_Request == VM_READ){
	  NUbus_Data.word = 0;
	  NUbus_acknowledge=1;
	  return;
	}
	break;
	*/
	
      default:      
	printf("VCMEM: Unimplemented address 0x%X\n",NUbus_Address.Addr);
	ld_die_rq = 1;
      }
    }
  }
}
