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
#include <unistd.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <strings.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

#include "ld.h"
#include "nubus.h"
#include "lambda_cpu.h"
#include "mem.h"
#include "sdu.h"
#include "syms.h"

// For this file only, PHYSKBD enables XBEEP
#ifdef CONFIG_PHYSKBD
#define XBEEP 1
#endif

#ifdef XBEEP
// BEEP support. The addresses of XBEEP and XFALSE are looked up in lambda_initialize.
uint32_t xbeep_addr = 0;
uint32_t xfalse_addr = 0;
#endif

// Configuration PROM message
static uint8_t prom_string[0x12] = "LMI LAMBDA V5.0";
static uint8_t prom_modelno_string[0x12] = "LAM001 V5.0";

// Emulator death flag
extern volatile int ld_die_rq;

// For mouse callback
extern uint32_t mouse_x_loc[2];
extern uint32_t mouse_y_loc[2];

#ifdef SHADOW
// Shadow Memory (Split per processor later)
Q ShadowMemory[0x2000000];
// Shadow Memory Page Map (Split per processor later)
ShadowMemoryPageEnt ShadowMemoryPageMap[0x20000];
#endif

// Processor states
struct lambdaState pS[2];

#ifdef CONFIG_CACHE
// Cache write-check status array
uint16_t cache_wc_status[2][0x00FFFFFF];

// Cache write-check mutexes
pthread_mutex_t cache_wc_mutex[2] = {PTHREAD_MUTEX_INITIALIZER,PTHREAD_MUTEX_INITIALIZER};
#endif

#ifdef SHADOW
// Shadow memory maintenance
void shadow_write(uint32_t addr,Q data){
  ShadowMemory[addr] = data;
}

Q shadow_read(uint32_t addr){
  return(ShadowMemory[addr]);
}
#endif

// Local Bus functions
void maybe_take_nubus_mastership(int I){
  if(pS[I].NUbus_Master == 0){
    if(pS[I].SM_Clock_Pulse > 0){
      logmsgf(LT_LAMBDA,0,"LCBUS: DANGER WILL ROBINSON! TAKING NUBUS MASTERSHIP IN SM CLOCK PULSE!\n");
    }
    take_nubus_mastership();
    pS[I].NUbus_Master = 1;
  }
}

// Utility functions
void debug_disassemble_IR(int I);
void sm_clock_pulse(int I,int clock,volatile Processor_Mode_Reg *oldPMR);

uint32_t ldb(uint64_t value, int size, int position){
  uint64_t mask,result;
  // Create mask
  mask = 0xFFFFFFFFFFFFFFFFLL; // Get all ones
  mask = mask >> (64 - size);  // Left-shift out the ones we don't need
  // Shift value into position
  result = (value >> position);
  return(result&mask);
}

uint32_t left_rotate(uint32_t data, int rot){
  uint32_t result=0;
  result  = (data >> (32-rot));
  result |= (data << rot);
  return(result);
}

uint32_t right_rotate(uint32_t data, int rot){
  uint32_t result=0;
  result  = (data << (32-rot));
  result |= (data >> rot);
  return(result);
}

// Debug trace items
#ifdef LAMBDA_DEBUGTRACE
#define MAX_DEBUG_TRACE 1000
uint64_t debugtrace_ir[2][MAX_DEBUG_TRACE];
uint32_t debugtrace_reg[2][MAX_DEBUG_TRACE][12];
int debugtrace_ptr[2] = { 0,0 };
#endif

// Barrel Shifter
void operate_shifter(int I){
  // uint32_t x=0;
  uint32_t Mask = 0;
  int left_mask_index;
  int right_mask_index;
  uint32_t R = pS[I].Mbus;

  // Shifter run
  // Rotate R
  if(pS[I].Iregister.Byte.Rotate_Source != 0){
    // Lambda doesn't have a rotate direction field
    R = left_rotate(R,pS[I].Iregister.Byte.Pos);
    if(pS[I].Iregister.Byte.Len == 040 && pS[I].Iregister.Byte.Rotate_Mask == 0){
      pS[I].Iregister.Byte.Len = 0;
    }
  }
  // Create mask
  // Get Right mask
  if(pS[I].Iregister.Byte.Rotate_Mask != 0){
    right_mask_index = pS[I].Iregister.Byte.Pos;
  }else{
    right_mask_index = 0;
  }
  right_mask_index &= 037;
  // Get Left mask
  // left_mask_index = (right_mask_index + (pS[I].Iregister.Byte.Len)) % 32;
  left_mask_index = (right_mask_index + (pS[I].Iregister.Byte.Len)) % 33;
  /*
  left_mask_index = (right_mask_index + (pS[I].Iregister.Byte.Len));
  if (left_mask_index == 040) left_mask_index = 037;
  left_mask_index %= 32;
  */

  // Shifter mask ROMs
  // THE MASKER CONTENTS AREN"T BEING TRASHED,
  // THE LEFT MASK IS COMING UP ONE ELEMENT SHORT!
  const uint32_t shift_left_mask[041] = {
    0x00000001,0x00000003,0x00000007,0x0000000F,
    0x0000001F,0x0000003F,0x0000007F,0x000000FF,
    0x000001FF,0x000003FF,0x000007FF,0x00000FFF,
    0x00001FFF,0x00003FFF,0x00007FFF,0x0000FFFF,
    0x0001FFFF,0x0003FFFF,0x0007FFFF,0x000FFFFF,
    0x001FFFFF,0x003FFFFF,0x007FFFFF,0x00FFFFFF,
    0x01FFFFFF,0x03FFFFFF,0x07FFFFFF,0x0FFFFFFF,
    0x1FFFFFFF,0x3FFFFFFF,0x7FFFFFFF,0xFFFFFFFF,0xFFFFFFFF
  };
  const uint32_t shift_right_mask[040] = {
    0xFFFFFFFF,0xFFFFFFFE,0xFFFFFFFC,0xFFFFFFF8,
    0xFFFFFFF0,0xFFFFFFE0,0xFFFFFFC0,0xFFFFFF80,
    0xFFFFFF00,0xFFFFFE00,0xFFFFFC00,0xFFFFF800,
    0xFFFFF000,0xFFFFE000,0xFFFFC000,0xFFFF8000,
    0xFFFF0000,0xFFFE0000,0xFFFC0000,0xFFF80000,
    0xFFF00000,0xFFE00000,0xFFC00000,0xFF800000,
    0xFF000000,0xFE000000,0xFC000000,0xF8000000,
    0xF0000000,0xE0000000,0xC0000000,0x80000000
  };

  /*
  if(left_mask_index == 040){
    logmsgf(LT_LAMBDA,10,"SHIFTER: LMI = 040, GENERATED LEFT MASK 0x");
    writeH32(shift_left_mask[left_mask_index]);
    logmsgf(LT_LAMBDA,10,"\n");
  }
  */

  // Final mask
  Mask = shift_left_mask[left_mask_index]&shift_right_mask[right_mask_index];

  if(pS[I].microtrace){
    logmsgf(LT_LAMBDA,10,"SHIFTER: LMI = %d RMI = %d SLM = 0x%X SRM = 0x%X MASK = 0x%X\n",
	   left_mask_index,right_mask_index,
	   shift_left_mask[left_mask_index],shift_right_mask[right_mask_index],
	   Mask);
  }

  // Merge A with R, using bits from R if the mask bit is 1
  pS[I].Obus = 0;
  /*
  // Bottom cycle
  if((Mask&0x01) == 0x01){
    // Bit set, pull this one from R
    pS[I].Obus |= (R&0x01);
  }else{
    // Bit clear, pull this one from A
    pS[I].Obus |= (pS[I].Abus&0x01);
  }
  // Continue
  x=0x01;
  while(x != 0x80000000){
    x = x << 1;
    if((Mask&x) == x){
      // Bit set, pull this one from R
      pS[I].Obus |= (R&x);
    }else{
      // Bit clear, pull this one from A
      pS[I].Obus |= (pS[I].Abus&x);
    }
  }
  // Let's do this faster!
  {
    uint32_t Output = 0;
  */
  // Bits set in mask come from R
  pS[I].Obus = R&Mask;
  // Bits clear in mask come from A
  pS[I].Obus |= pS[I].Abus&(~Mask);
  // SAVE R FOR LATER ABUSE
  pS[I].Rbus = R;
  if(pS[I].microtrace){
    logmsgf(LT_LAMBDA,10,"SHIFTER: COMPLETED! O = 0x%X\n",pS[I].Obus);
  }
}

/*
   MICROINSTRUCTION BITS

   ALL OPS
   LAM-IR-PARITY-FIELD (BYTE 4. 60.)
   LAM-IR-HALT (BYTE 1. 59.)
   LAM-IR-CLOBBERS-MEM-SUBR-BIT (byte 1 58.)
   LAM-IR-STAT-BIT (byte 1 57.)
   LAM-IR-ILONG (byte 1 56.)
   LAM-IR-SLOW-DEST (BYTE 1 55.)
   LAM-IR-MACRO-STREAM-ADVANCE (BYTE 1 54.)
   LAM-IR-SOURCE-TO-MACRO-IR (BYTE 1 53.)
   LAM-IR-MACRO-IR-DISPATCH (BYTE 1 52.)
   LAM-IR-POPJ-AFTER-NEXT (byte 1 51.)
   LAM-IR-A-SRC (byte 12. 39.)
   LAM-IR-M-SRC (byte 7 32.)
   LAM-IR-OP (byte 2 30.)

   ALU-OP
   WHAT ARE BITS 27-29?
   LAM-IR-DEST (BYTE 13. 14.)
   LAM-IR-MF (BYTE 2 12.) ;MISCELLANEOUS FUNCTION
   LAM-IR-OB (BYTE 3 9.)  ;note bit 2 of this field conditions sel.0 of the output selector
   LAM-IR-ALUF (BYTE 7 2)        ;INCLUDING CARRY
   LAM-IR-Q (BYTE 2 0)

   BYTE-OP (OTHERWISE SAME AS ALU-OP?)
   LAM-IR-BYTE-FUNC (BYTE 2 28.)

   LAM-BYTE-SPEC (BYTE 12. 0)


   JUMP-OP
   LAM-IR-JUMP-ADDR (BYTE 16. 14.)
   LAM-IR-JUMP-PAGE-NUMBER-IN-ADR (BYTE 12. 18.)
   LAM-IR-JUMP-LC-INCREMENT (BYTE 1 11.)
   LAM-IR-RPN (BYTE 3 8.)
   LAM-IR-JUMP-COND (BYTE 8. 0)

   DISP-OP
   LAM-IR-DISP-ENABLE-META (BYTE 2 28.)          ;enable bits from map.
   LAM-IR-DISP-WRITE-VMA (BYTE 1 27.)
   LAM-IR-DISP-LPC (BYTE 1 26.)                  ;"decrement" return address on call
   LAM-IR-DISP-DISPATCH-CONSTANT (BYTE 12. 14.)
   WHAT ARE BITS 11-13?
   LAM-IR-DISP-BYTL (BYTE 5 6)
   LAM-IR-MROT (BYTE 6 0)





 */


// Disassembly tables
// Jump R-P-N operations. On Lambda, the NOP and SKIP are "undefined".
const char *jump_op_str[10] = {
  "Jump-XCT-Next",
  "JUMP",
  "Call-XCT-Next",
  "CALL",
  "Return-XCT-Next",
  "RETURN",
  "R+P",
  "R+P+N" };

const char *alu_op_str[040] = {
  "SETZ",
  "AND",
  "ANDCA",
  "SETM",
  "04",
  "SETA",
  "XOR",
  "IOR",
  "10",
  "11",
  "12",
  "13",
  "14",
  "15",
  "16",
  "SETO",
  "MSTEP",
  "MSTEP-LAST",
  "DFSTEP",
  "DSTEP",
  "RSTEP",
  "25",
  "M-A-1",
  "27",
  "30",
  "ADD",
  "32",
  "33",
  "34",
  "35",
  "36",
  "M+M" };

const char *jump_cond_str[040] = {
  "00",
  "M<A",
  "M<=A",
  "M!=A",
  "NOT-PAGE-FAULT",
  "PAGE-FAULT-OR-INTERRUPT",
  "PAGE-FAULT-OR-INTERRUPT-OR-SEQUENCE-BREAK",
  "ALWAYS",
  "10",
  "DATA-TYPE-NOT-EQUAL",
  "12",
  "13",
  "14",
  "15",
  "16",
  "17",
  "20",
  "21",
  "22",
  "23",
  "24",
  "25",
  "26",
  "27",
  "30",
  "31",
  "32",
  "33",
  "34",
  "35",
  "36",
  "37"
};

// Minus 0100
const char *mf_src_str[0100] = {
  "INTERRUPT-POINTER",
  "MACRO.IR.DISPLACEMENT",
  "STAT-COUNTER",
  "MACRO.IR",
  "MACRO.IR.DECODE.RAM",
  "SPY-REG",
  "MULTIPLIER-FT",
  "RG-MODE",
  "DISP-CONST",
  "MICRO-STACK",
  "MICRO-STACK-POP",
  "CRAM-ADR-MAP",
  "114",
  "115",
  "116",
  "117",
  "CACHE-ADDRESS",
  "MD-NO-HOLD",
  "VMA",
  "L1-MAP",
  "L2-MAP-CONTROL",
  "L2-MAP-PHYSICAL-PAGE",
  "LC",
  "127",
  "PDL-BUFFER-INDEX",
  "Q",
  "PDL-BUFFER-POINTER",
  "133",
  "134",
  "DP-MODE",
  "C-PDL-BUFFER-POINTER-POP",
  "C-PDL-BUFFER-INDEX",
  "140",
  "141",
  "STAT-COUNTER-AUX",
  "143",
  "MID-MISC",
  "145",
  "MULTIPLIER",
  "147",
  "150",
  "151",
  "152",
  "153",
  "154",
  "155",
  "156",
  "157",
  "160",
  "MD",
  "162",
  "163",
  "164",
  "165",
  "166",
  "167",
  "170",
  "171",
  "172",
  "173",
  "174",
  "175",
  "C-PDL-BUFFER-POINTER",
  "177"
};

const char *mf_dest_str[0100] = {
  "000",
  "LC",
  "DP-MODE",
  "003",
  "STAT-COUNTER-AUX",
  "MID",
  "CRAM-HI",
  "CRAM-LO",
  "C-PDL-BUFFER-POINTER",
  "C-PDL-BUFFER-POINTER-PUSH",
  "C-PDL-BUFFER-INDEX",
  "PDL-BUFFER-POINTER",
  "014",
  "MICRO-STACK-PUSH",
  "IMOD-LO",
  "IMOD-HI",
  "VMA",
  "VMA-START-READ",
  "VMA-START-WRITE",
  "L1-MAP",
  "L2-MAP-CONTROL",
  "025",
  "CRAM-MAP",
  "027",
  "MD",
  "MD-START-READ",
  "MD-START-WRITE",
  "C-PDL-INDEX-INC",
  "MICRO-STACK-DATA",
  "MICRO-STACK-POINTER-IF-POP",
  "C-PDL-INDEX-DEC",
  "MULTIPLIER",
  "040",
  "INTERRUPT-CLEAR",
  "RG-MODE",
  "043",
  "STAT-COUNTER",
  "045",
  "046",
  "047",
  "050",
  "051",
  "052",
  "PDL-BUFFER-INDEX",
  "054",
  "055",
  "056",
  "057",
  "060",
  "VMA-START-READ-FORCE",
  "VMA-START-WRITE-FORCE",
  "063",
  "L2-MAP-PHYSICAL-PAGE",
  "065",
  "066",
  "067",
  "070",
  "MD-START-READ-FORCE",
  "MD-START-READ-FORCE",
  "073",
  "074",
  "075",
  "076",
  "077"
};

// Micro disassembler

void disassembleWork(void* f, uint64_t inst, int debLog){
  char *location;
  char symloc[100];
  int offset;
  UInst ui;

  // Disassemble the instruction in inst.
  ui.raw = inst;

  // Global fields
  if(debLog == 0){
    fprintf(f, "P: %o",ui.Parity);
  }
  if(ui.Halt != 0){ fprintf(f, " HALT"); }
  if(ui.Clobbers_Mem_Subr_Bit != 0){ fprintf(f, " CMSB"); }
  if(ui.Stat_Bit != 0){ fprintf(f, " STAT"); }
  if(ui.ILong != 0){ fprintf(f, " ILong"); }
  if(ui.Slow_Dest != 0){ fprintf(f, " SLOW-DEST"); }
  if(ui.Macro_Stream_Advance != 0){ fprintf(f, " MSA"); }
  if(ui.Src_to_Macro_IR != 0){ fprintf(f, " S2MIR"); }
  if(ui.Macro_IR_Disp != 0){ fprintf(f, " MIR-DISP"); }
  if(ui.PopJ_After_Next != 0){ fprintf(f, " PJAN"); }
  // sym_find_by_val(int mcr, int t, int v)
  if(ui.ASource > 0){
    location = sym_find_by_val(1, 4, ui.ASource);
    if(location != 0){
      fprintf(f, " %s(%o)",location,ui.ASource);
    }else{
      fprintf(f, " A:%o",ui.ASource);
    }
  }
  if(ui.MSource > 0){
    if(ui.MSource > 077){
      fprintf(f, " MF-%s",mf_src_str[ui.MSource-0100]);
    }else{
      location = sym_find_by_val(1, 5, ui.MSource);
      if(location != 0){
        fprintf(f, " %s(%o)",location,ui.MSource);
      }else{
        fprintf(f, " M:%o",ui.MSource);
      }
    }
  }
  fprintf(f, " ");

  // Next get opcode and switch
  switch(ui.Opcode){
  case 0: // ALU
    fprintf(f, "ALU Carry %o Operation %s Mask %o Output %o Dest ",ui.ALU.Carry,alu_op_str[ui.ALU.Operation],ui.ALU.Mask,ui.ALU.Output);
    // Destination
    if(ui.Destination.A.Flag != 0){
      location = sym_find_by_val(1, 4, ui.Destination.A.Addr);
      if(location != 0){
        fprintf(f, "%s(A-%o)",location,ui.Destination.A.Addr);
      }else{
        fprintf(f, "A:%o",ui.Destination.A.Addr);
      }
    }else{
      location = sym_find_by_val(1, 5, ui.Destination.M.Addr);
      if(location != 0){
        fprintf(f, "%s(AM-%o)",location,ui.Destination.M.Addr);
      }else{
        fprintf(f, "M:%o",ui.Destination.M.Addr);
      }
      if(ui.Destination.F.Dest > 0){
        fprintf(f, " MF-%s",mf_dest_str[ui.Destination.F.Dest]);
      }
    }
    if(debLog == 0){
      fprintf(f, " Misc %o Spare %o QControl %o",ui.ALU.Misc,ui.ALU.Spare,ui.ALU.QControl);
    }
    break;
  case 1: // BYTE
    fprintf(f, "BYTE RMask %o RSrc %o Pos %o Len %o Dest ",ui.Byte.Rotate_Mask,ui.Byte.Rotate_Source,ui.Byte.Pos,ui.Byte.Len);
    // writeOct(ui.Byte.Dest);
    if(ui.Destination.A.Flag != 0){
      location = sym_find_by_val(1, 4, ui.Destination.A.Addr);
      if(location != 0){
        fprintf(f, "%s(%o)",location,ui.Destination.A.Addr);
      }else{
        fprintf(f, "A:%o",ui.Destination.A.Addr);
      }
    }else{
      location = sym_find_by_val(1, 5, ui.Destination.M.Addr);
      if(location != 0){
        fprintf(f, "%s(%o)",location,ui.Destination.M.Addr);
      }else{
        fprintf(f, "M:%o",ui.Destination.M.Addr);
      }
      if(ui.Destination.F.Dest > 0){
        fprintf(f, " MF-%s",mf_dest_str[ui.Destination.F.Dest]);
      }
    }
    if(debLog == 0){
      fprintf(f, " Misc %o Spare %o",ui.Byte.Misc,ui.Byte.Spare);
    }
    break;
  case 2: // JUMP
    {
      location = "";
      offset = 0;
      location = sym_find_last(1, ui.Jump.Address, &offset);
      if(location != 0){
        if(offset != 0){
          sprintf(symloc, "%s+%o", location, offset);
        }else{
          sprintf(symloc, "%s", location);
        }
      }else{
        sprintf(symloc, "Addr");
      }
      // Operation and new-micro-PC
      // sprintf(disasm,"%s %d ",jump_op_str[ldb(inst,3,5)], ldb(inst,14,18));
      fprintf(f, "%s %s(%o) Invert %o Test %o Cond ",jump_op_str[ui.Jump.RPN],symloc,ui.Jump.Address,ui.Jump.Invert,ui.Jump.Test);
      if(ui.Jump.Test != 0){
        fprintf(f, "%s", jump_cond_str[ui.Jump.Cond]);
      }else{
	fprintf(f, "%o", ui.Jump.Cond);
      }
      fprintf(f, " LCInc %o",ui.Jump.LC_Increment);
      if(debLog == 0){
        fprintf(f, " Spare %o Spare2 %o",ui.Jump.Spare,ui.Jump.Spare2);
      }
    }
    break;
  case 3: // DISP
    fprintf(f, "DISPATCH Pos %o Len %o Constant %o LPC %o WriteVMA %o EnableGCV %o EnableOldspace %o",
	    ui.Dispatch.Pos,ui.Dispatch.Len,ui.Dispatch.Constant,ui.Dispatch.LPC,ui.Dispatch.Write_VMA,
	    ui.Dispatch.Enable_GC_Volatility_Meta,ui.Dispatch.Enable_Oldspace_Meta);
    if(debLog == 0){
      fprintf(f, " Spare %o",ui.Dispatch.Spare);
    }
    break;
  }
}

void disassemble(int level,uint64_t inst){
  extern uint8_t loglevel[MAX_LOGTYPE];
  if(loglevel[LT_LAMBDA] >= level){
    disassembleWork(stdout, inst, 0);
  }
}

void disassemble_IR(int I){
  char *location;
  char symloc[100];
  int offset;

  location = "";
  offset = 0;
  location = sym_find_last(1, pS[I].loc_ctr_cnt, &offset);
  if(location != 0){
    if(offset != 0){
      sprintf(symloc, "%s+%o", location, offset);
    }else{
      sprintf(symloc, "%s", location);
    }
  }else{
    symloc[0] = 0;
  }
  logmsgf(LT_LAMBDA,10,"[%s (%o)]\n\t",symloc,pS[I].loc_ctr_cnt);
  disassemble(10,pS[I].Iregister.raw);
  logmsgf(LT_LAMBDA,10,"\n");
}

void debug_disassemble_IR(int I){
  char *location;
  char symloc[100];
  int offset;

  location = "";
  offset = 0;
  location = sym_find_last(1, pS[I].loc_ctr_cnt, &offset);
  if(location != 0){
    if(offset != 0){
      sprintf(symloc, "%s+%o", location, offset);
    }else{
      sprintf(symloc, "%s", location);
    }
  }else{
    symloc[0] = 0;
  }
  logmsgf(LT_LAMBDA,10,"[%s (%o) A=%X M=%X O=%X]\n\t",symloc,
	 pS[I].loc_ctr_cnt,pS[I].Abus,pS[I].Mbus,pS[I].Obus);
  disassemble(10,pS[I].Iregister.raw);
  logmsgf(LT_LAMBDA,10,"\n");
}

#ifdef LAMBDA_DEBUGTRACE
// Write out debug trace entry
void write_debugtrace_ent(int I,int x){
  // udisassemble(debugtrace_reg[x][0], debugtrace_ir[I][x], disasm);
  char *location;
  char symloc[100];
  int offset;

  location = "";
  offset = 0;
  location = sym_find_last(1, debugtrace_reg[I][x][0], &offset);
  if(location == 0){ location = ""; }
  if(offset != 0){
      sprintf(symloc, "%s+%o", location, offset);
  }else{
      sprintf(symloc, "%s", location);
  }
  logmsgf(LT_LAMBDA,10,"[%s (%o) A=%X M=%X O=%X]\n\t",symloc,
	 debugtrace_reg[I][x][0],
	 debugtrace_reg[I][x][2],
	 debugtrace_reg[I][x][3],
	 debugtrace_reg[I][x][4]);
  disassemble(10,debugtrace_ir[I][x]);
  logmsgf(LT_LAMBDA,10,"\n");

  /*
  fprintf(trace,"%.4X/%.4X [A=%.8X M=%.8X O=%.8X uSP=%.2X VMA=%.8X MD=%.8X LC=%.8X] %s\n",
          debugtrace_reg[I][x][0], debugtrace_reg[I][x][1], debugtrace_reg[I][x][2], debugtrace_reg[I][x][3], debugtrace_reg[I][x][4], debugtrace_reg[I][x][5],
          debugtrace_reg[I][x][6], debugtrace_reg[I][x][7], debugtrace_reg[I][x][8], disasm);
  if((debugtrace_reg[I][x][9]&1)==1){ fprintf(trace,"** NOP-NEXT\n"); }
  if((debugtrace_reg[I][x][9]&2)==2){ fprintf(trace,"** DISP-NEXT\n"); }
  if((debugtrace_reg[I][x][9]&4)==4){ fprintf(trace,"** DISP-AFTER-NEXT %d\n",debugtrace_reg[I][x][10]); }
  if((debugtrace_reg[I][x][9]&8)==8){ fprintf(trace,"** ISTREAM-FETCH\n"); }
  if(debugtrace_msg[x][0] != 0){ fprintf(trace,"%s\n",debugtrace_msg[x]); }
  fflush(trace);
  */
}
#endif

// Reset
// If ID is zero, we are reinitializing.
void lambda_initialize(int I,int ID){
  int x=0;
  // Unlike Raven, Lambda does not have a microcode PROM of its own.
  // The SDU loads the bootstrap from PROM into WCS.
  // Stop clock
  pS[I].cpu_die_rq = 1;
  pS[I].SM_Clock = 0;
  // Reset flags
  pS[I].popj_after_nxt = -1;
  pS[I].slow_dest = false;
  pS[I].long_inst = false;
  pS[I].macro_dispatch_inst = -1;
  pS[I].exec_hold = false;
  pS[I].uI_Clock_Pulse = false;
  pS[I].wrote_uPC = false;
  pS[I].NOP_Next = false;
  pS[I].mirInvalid = 0;
  // Reset LCbus
  pS[I].NUbus_Master = 0;
  pS[I].LCbus_Busy = 0;
  pS[I].LCbus_acknowledge = 0;
  pS[I].LCbus_error = 0;
  // Reset SM clocks
  pS[I].SM_Clock_Pulse = 0;
  // Reinitialize performance counter
  pS[I].delta_time = 0;
  // If this is an initial setup...
  if(ID != 0){
    // Clobber erasable memories
    bzero((uint8_t *)pS[I].WCS,(64*1024)*8);
    // Reset PC
    pS[I].loc_ctr_reg.raw = 0;
    pS[I].loc_ctr_cnt = 0;
    pS[I].loc_ctr_nxt = -1;
    // Clobber uPCS pointer
    pS[I].uPCS_ptr_reg = 0;
    // Clobber TRAM PC
    pS[I].TRAM_PC = 03000;
    pS[I].ConReg.uinst_clock_l = 1;
    // Reset more flags
    pS[I].spy_wrote_ireg = false;
    pS[I].microtrace = false;
    pS[I].macrotrace = false;
    // Set up slot assignments
    pS[I].NUbus_ID = ID;
    pS[I].RG_Mode.NUbus_ID = (pS[I].NUbus_ID&0x0F);
#ifdef CONFIG_CACHE
    // Clobber cache
    x=0;
    while(x < 0x1000000){
      cache_wc_status[0][x] = cache_wc_status[1][x] = 0;
      x++;
    }
    x = 0;
    while(x < 256){
      int y;
      pS[I].Cache_Sector_Addr[x] = 0;
      y=0;
      while(y < 4){
        pS[I].Cache_Status[x][y] = 0;
        y++;
      }
      x++;
    }
    pS[I].Cache_Full = 0;
    pS[I].Cache_Oldest_Sector = 0;
#endif
    // Let's experimentally reset bits in the LV1 and LV2 maps
    x=0;
    while(x < 4096){
      // All these fields are inverted.
      pS[I].vm_lv1_map[x].MB = 03;
      pS[I].vm_lv1_map[x].MB_Valid = 1;
      pS[I].vm_lv2_ctl[x].Meta = 077;
      x++;
    }
    // Also clobber interrupts on the RG board
    x=0;
    pS[I].InterruptPending = 0;
    while(x<0x100){
      pS[I].InterruptStatus[x] = 0;
      x++;
    }
  }

#ifdef XBEEP
  // Should this be done every time, in case user changes microcode?
  // But then they'd have to change symbols file too...
  if (xbeep_addr == 0) {
    if (sym_find(1, "XBEEP", (int *)&xbeep_addr) != 0)
      logmsgf(LT_LAMBDA,0,"can't find XBEEP ucode address\n");
    else
      logmsgf(LT_LAMBDA,10,"found XBEEP at %#o\n", xbeep_addr);
  }
  if (xfalse_addr == 0) {
    if (sym_find(1, "XFALSE", (int *)&xfalse_addr) != 0)
      logmsgf(LT_LAMBDA,0,"can't find XFALSE ucode address\n");
    else
      logmsgf(LT_LAMBDA,10,"found XFALSE at %#o\n", xfalse_addr);
  }
#endif
}

// ALU items
// ALU operation M-A or (M-A-1)
// Used by JUMP
void alu_sub_stub(int I,int Carry){
  pS[I].ALU_Result = pS[I].Mbus - pS[I].Abus - (Carry ? 0 : 1);
  // FIXNUM Overflow Check
  if((((pS[I].Mbus^pS[I].Abus)&(pS[I].Mbus^pS[I].ALU_Result))&0x01000000)==0x01000000){
    pS[I].ALU_Fixnum_Oflow=1;
  }
}

// Sanitize results
void alu_cleanup_result(int I){
  // Reduce carry-out to a flag without use of boolean type
  if((pS[I].ALU_Result&0xFFFFFFFF00000000LL) != 0){ pS[I].ALU_Carry_Out = 1; }
  // Clean Output (ALU is 32 bits wide)
  pS[I].ALU_Result &= 0xFFFFFFFF;
  // Result to O-bus
  pS[I].Obus = pS[I].ALU_Result;
}

// Arrange for carry-out
void fix_alu_carry_out(int I){
  int cout = ((pS[I].ALU_Result < pS[I].Mbus ? 1 : 0) + ((pS[I].Mbus>>31)&1) + ((pS[I].Abus>>31)&1)) & 1;
  pS[I].ALU_Result &= 0xffffffff;
  if (cout){
    pS[I].ALU_Result |= 0x100000000LL; // Arrange for carry-out
  }
}

void operate_alu(int I){
  switch(pS[I].Iregister.ALU.Operation){
  case 0: // LAM-ALU-SETZ
    pS[I].ALU_Result = 0;
    break;
  case 1: // LAM-ALU-AND
    pS[I].ALU_Result = pS[I].Mbus&pS[I].Abus;
    // If bit sign is set, carry-out.
    if(pS[I].ALU_Result&0x80000000){
      pS[I].ALU_Result |= 0x100000000LL;
    }
    break;
  case 2: // LAM-ALU-ANDCA
    pS[I].ALU_Result = pS[I].Mbus&(~pS[I].Abus);
    break;
  case 3: // LAM-ALU-SETM
    pS[I].ALU_Result = pS[I].Mbus;
    break;
  case 4: // No symbol, used in ANDCM, see imicro/lambda-components.lisp
    pS[I].ALU_Result = (~pS[I].Mbus)&pS[I].Abus;
    break;
  case 5: // LAM-ALU-SETA
    pS[I].ALU_Result = pS[I].Abus;
    break;
  case 6: // LAM-ALU-XOR
    pS[I].ALU_Result = pS[I].Mbus^pS[I].Abus;
    break;
  case 7: // LAM-ALU-IOR
    pS[I].ALU_Result = pS[I].Mbus|pS[I].Abus;
    break;
  case 010: // No symbol, used in ANDCB, see imicro/lambda-components.lisp
    pS[I].ALU_Result = (~pS[I].Mbus)&(~pS[I].Abus);
    break;
  case 012: // No symbol, used in SETCA, see imicro/lambda-components.lisp
    pS[I].ALU_Result = ~pS[I].Abus;
    break;
  case 014: // No symbol, used in SETCM, see imicro/lambda-components.lisp
    pS[I].ALU_Result = ~pS[I].Mbus;
    break;
  case 017: // LAM-ALU-SETO
    pS[I].ALU_Result = 0xFFFFFFFF;
    break;
  case 020: // LAM-ALU-MSTEP
    if((pS[I].Qregister&0x01)==0x01){
      pS[I].ALU_Result = pS[I].Mbus + pS[I].Abus;
      fix_alu_carry_out(I);
    }else{
      pS[I].ALU_Result = pS[I].Mbus;
      pS[I].ALU_Result &= 0xffffffff;
      if(pS[I].ALU_Result & 0x80000000){
	pS[I].ALU_Result |= 0x100000000LL; // Arrange for carry-out
      }
    }
    break;
  case 022: // LAM-ALU-DFSTEP
    // Divide First Step
    if(pS[I].Abus&0x80000000){
      // logmsgf(LT_LAMBDA,10,"DFSTEP: NEGATIVE DIVISOR\n");
      pS[I].ALU_Result = pS[I].Mbus - (-pS[I].Abus);
    }else{
      pS[I].ALU_Result = pS[I].Mbus - pS[I].Abus;
    }
    // Set divisor sign
    // slow destination required to allow divisor sign to load.
    if(pS[I].Abus&0x80000000){ pS[I].DP_Mode.Divisor_Sign = 1; }else{ pS[I].DP_Mode.Divisor_Sign = 0; }
    break;
  case 023: // LAM-ALU-DSTEP
    // Divide Step
    if(pS[I].DP_Mode.Divisor_Sign == 1){
      // logmsgf(LT_LAMBDA,10,"DSTEP: NEGATIVE DIVISOR\n");
      if((pS[I].Qregister&0x01)==0){
	pS[I].ALU_Result = pS[I].Mbus + (-pS[I].Abus);
      }else{
	pS[I].ALU_Result = pS[I].Mbus - (-pS[I].Abus);
      }
    }else{
      if((pS[I].Qregister&0x01)==0){
	pS[I].ALU_Result = pS[I].Mbus + pS[I].Abus;
      }else{
	pS[I].ALU_Result = pS[I].Mbus - pS[I].Abus;
      }
    }
    break;
  case 024: // LAM-ALU-RSTEP
    // Divide Remainder Correction Step
    if(pS[I].DP_Mode.Divisor_Sign == 1){
      // logmsgf(LT_LAMBDA,10,"RSTEP: NEGATIVE DIVISOR\n");
      if((pS[I].Qregister&0x01)==0x00){
	pS[I].ALU_Result = pS[I].Mbus + (-pS[I].Abus);
      }else{
	pS[I].ALU_Result = pS[I].Mbus;
      }
    }else{
      if((pS[I].Qregister&0x01)==0x00){
	pS[I].ALU_Result = pS[I].Mbus + pS[I].Abus;
      }else{
	pS[I].ALU_Result = pS[I].Mbus;
      }
    }
    break;
  case 026: // LAM-ALU-M-A-1
    // alu_sub_stub(I,1);
    alu_sub_stub(I, pS[I].Iregister.ALU.Carry);
    break;
  case 031: // LAM-ALU-ADD
    pS[I].ALU_Result = pS[I].Mbus + pS[I].Abus + (pS[I].Iregister.ALU.Carry ? 1 : 0);
    if((((pS[I].Mbus^pS[I].ALU_Result)&(pS[I].Abus^pS[I].ALU_Result))&0x01000000)==0x01000000){
      pS[I].ALU_Fixnum_Oflow=1;
    }
    fix_alu_carry_out(I);
    break;
  case 034: // LAM-??? (Raven ALU-Opcode-M)
    // M or M+1
    pS[I].ALU_Result = pS[I].Mbus;
    if(pS[I].Iregister.ALU.Carry != 0){
      pS[I].ALU_Result++;
    }
    fix_alu_carry_out(I);
    break;
  case 037: // LAM-ALU-M+M
    pS[I].ALU_Result = pS[I].Mbus + pS[I].Mbus + (pS[I].Iregister.ALU.Carry ? 1 : 0);
    if((((pS[I].Mbus^pS[I].ALU_Result)&(pS[I].Mbus^pS[I].ALU_Result))&0x01000000)==0x01000000){
      pS[I].ALU_Fixnum_Oflow=1;
    }
    fix_alu_carry_out(I);
    break;
  default:
    logmsgf(LT_LAMBDA,0,"Unknown ALU Operation %o\n",pS[I].Iregister.ALU.Operation);
    pS[I].cpu_die_rq = 1;
  }

  // Reduce carry-out to a flag
  pS[I].ALU_Carry_Out = (pS[I].ALU_Result&0xFFFFFFFF00000000LL) ? 1 : 0;
  // Clean Output (ALU is 32 bits wide)
  pS[I].ALU_Result &= 0xFFFFFFFF;

  // For logical operations the carry-out flag is the integer sign bit.
  if((pS[I].Iregister.ALU.Operation < 030) && (pS[I].Iregister.ALU.Operation != 020)){
    pS[I].ALU_Carry_Out = (pS[I].ALU_Result & 0x80000000) ? 1 : 0;
  }

  // The mask bit selects where the high bits of the result come from.
  // If it's set, they come from the A-bus.
  if(pS[I].Iregister.ALU.Mask > 0){
    pS[I].ALU_Result &= 0x01FFFFFF;
    pS[I].ALU_Result |= (pS[I].Abus&0xFE000000);
  }
}

void handle_o_bus(int I){
  // Determine output location
  pS[I].Obus_Input = pS[I].ALU_Result;

  // Load pS[I].Obus
  switch(pS[I].Iregister.ALU.Output){

  case 0: // LAM-OB-MSK
    // Run things through the MASKER PROM.
    // Bits feeding the masker:
    // Bit 8: "feeds MASKER proms on ALU inst, which, if set, selects:
    //  alu.output.bus.control.2 =0, bits 31-25 ->1, 24-0-> 0.  ie, if alu.output.bus.control.1
    //   =0 and alu.output.bus.control.0=1 (normal case), then data type bits come from
    //   A source, pointer bits from ALU.
    //  alu.output.bus.control.2 =1, " (comment ended)
    // Bit 11: "this bit goes to masker prom on alu's, allowing access to masked add hacks, etc.
    // Bit 29: "in alu inst, this same bit allows q-control bits to reach masker proms, allowing access to masked-add hacks."

    // "if the m source is all ones and the a source is all zeroes, then the output should be identical to the contents of the masker prom"
    // (But that was for byte operations!)

    if(pS[I].Iregister.ALU.Mask){ logmsgf(LT_LAMBDA,0,"MASKER: BIT 8 SET\n"); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.ALU.Output&0x040){ logmsgf(LT_LAMBDA,0,"MASKER: BIT 11 SET\n"); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.ALU.Spare&0x040){ logmsgf(LT_LAMBDA,0,"MASKER: BIT 29 SET\n"); pS[I].cpu_die_rq = 1; }

    if(pS[I].Iregister.ALU.Operation != 0){
      // logmsgf(LT_LAMBDA,10,"MASKER: Masker PROM unimplemented\n");
      // pS[I].cpu_die_rq = 1;
    }
    pS[I].Obus = pS[I].Obus_Input;
    break;

  case 1: // LAM-OB-ALU
    pS[I].Obus = pS[I].Obus_Input;
    break;
  case 2: // LAM-OB-ALU-RIGHT-1
    pS[I].Obus = pS[I].Obus_Input >> 1;
    // Different method
    if(pS[I].ALU_Carry_Out != 0){
      pS[I].Obus |= 0x80000000;
    }else{
      pS[I].Obus &= ~0x80000000;
    }
    break;
  case 3: // LAM-OB-ALU-EXTEND-25
    // Raven Output-Bus-Sign-Extend
    if((pS[I].Obus_Input&0x01000000)==0){
      pS[I].Obus = pS[I].Obus_Input&0x00FFFFFF;
    }else{
      pS[I].Obus = pS[I].Obus_Input|0xFF000000;
    }
    break;
  case 5: // See imicro/lambda-components.lisp - Does not have a symbol. Used for OUTPUT-SELECTOR-MASK-11
    pS[I].Obus = (pS[I].Obus_Input & 0x7FF);
    break;
  case 6: // LAM-OB-ALU-LEFT-1
    pS[I].Obus = (pS[I].Obus_Input << 1);
    if(pS[I].Qregister&0x80000000){ pS[I].Obus |= 1; }
    break;
  case 7: // LAM-OB-ALU-MIRROR
    // Reverse the bits in the ALU output
    pS[I].Obus = pS[I].Obus_Input;
    pS[I].Obus = ((pS[I].Obus >>  1) & 0x55555555) | ((pS[I].Obus <<  1) & 0xaaaaaaaa);
    pS[I].Obus = ((pS[I].Obus >>  2) & 0x33333333) | ((pS[I].Obus <<  2) & 0xcccccccc);
    pS[I].Obus = ((pS[I].Obus >>  4) & 0x0f0f0f0f) | ((pS[I].Obus <<  4) & 0xf0f0f0f0);
    pS[I].Obus = ((pS[I].Obus >>  8) & 0x00ff00ff) | ((pS[I].Obus <<  8) & 0xff00ff00);
    pS[I].Obus = ((pS[I].Obus >> 16) & 0x0000ffff) | ((pS[I].Obus << 16) & 0xffff0000);
    break;
  default:
    logmsgf(LT_LAMBDA,0,"Unknown output select %o\n",pS[I].Iregister.ALU.Output);
    pS[I].cpu_die_rq = 1;
  }
}

// Q register
void handle_q_register(int I){
  if(pS[I].Iregister.ALU.QControl > 0){
    switch(pS[I].Iregister.ALU.QControl){
    case 1: // LAM-Q-LEFT
      // Shift
      pS[I].Qregister = pS[I].Qregister << 1;
      // Carry-In inverse of ALU sign bit
      if((pS[I].ALU_Result&0x80000000) == 0){
	pS[I].Qregister |= 1;
      }
      break;
    case 2: // LAM-Q-RIGHT
      pS[I].Qregister = pS[I].Qregister >> 1;
      // Carry-In ALU result
      if((pS[I].ALU_Result&01) == 0x01){
	pS[I].Qregister |= 0x80000000;
      }
      break;
    case 3: // LAM-Q-LOAD
      pS[I].Qregister=pS[I].ALU_Result;
      break;
    default:
      logmsgf(LT_LAMBDA,0,"Unknown Q control %o\n",pS[I].Iregister.ALU.QControl);
      pS[I].cpu_die_rq = 1;
    }
  }
}

// Virtual memory mapping process
void VM_resolve_address(int I,int access,int force){
  if(pS[I].microtrace){
    logmsgf(LT_LAMBDA,10,"VM: Access %o Force %o: VMA = 0x%X\n",
	   access,force,pS[I].VMAregister.raw);
    logmsgf(LT_LAMBDA,10,"VM: LV1 ENT = 0x%X (LV2 Block %o MB %o MB-Validity %o)\n",
	   pS[I].vm_lv1_map[pS[I].VMAregister.VM.VPage_Block].raw,
	   pS[I].vm_lv1_map[pS[I].VMAregister.VM.VPage_Block].LV2_Block,
	   pS[I].vm_lv1_map[pS[I].VMAregister.VM.VPage_Block].MB,
	   pS[I].vm_lv1_map[pS[I].VMAregister.VM.VPage_Block].MB_Valid);
  }
  // Reset page fault
  pS[I].Page_Fault = 0;
  // Lambda doesn't have a LV1 valid bit, so LV1 better be valid!

  // LV1 Meta Bits:
  // "For hardware convenience, all three L1 map meta bits are stored in COMPLEMENTED form."
  // 0 (3) = Static Region (OLDEST)
  // 1 (2) = Dynamic Region
  // 2 (1) = Active Consing Region
  // 3 (0) = Extra PDL Region (NEWEST)

  // Obtain LV2 index
  pS[I].vm_lv2_index.raw = 0;
  pS[I].vm_lv2_index.VPage_Offset = pS[I].VMAregister.VM.VPage_Offset;
  pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].VMAregister.VM.VPage_Block].LV2_Block;

#ifdef CONFIG_CACHE
  // Propagate cache stuff
  pS[I].Cache_Permit = pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Cache_Permit;
  pS[I].Packet_Code = pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packet_Code;
  pS[I].Packetize_Writes = pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packetize_Writes;
  pS[I].Cache_Sector_Hit = 0;
  pS[I].Cache_Sector = 0;
#endif

  // Print LV2 data
  if(pS[I].microtrace){
    logmsgf(LT_LAMBDA,10,"VM: LV2 CTL ENT = 0x%X (Meta %o Status %o Access %o Force-Allowed %o Packet_Code %o Packetize-Writes %o Enable-Cache %o Lock-Nubus %o)\n",
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].raw,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Access,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Force_Allowed,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packet_Code,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packetize_Writes,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Cache_Permit,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Lock_NUbus);
    logmsgf(LT_LAMBDA,10,"VM: LV2 ADR ENT 0x%X = 0x%X (PPN %o Byte_Code %o)\n",
	   pS[I].vm_lv2_index.raw,pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].raw,
	   pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].PPN,
	   pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].Byte_Code);
  }
  // Here's our chance to screw up.
  // LV2 Meta:
  //

  // Update cached GCV
  pS[I].cached_gcv = pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta&03;

  // LV2 Access:
  // 1 = Write Only
  // 2 = Read Only
  // 3 = R/W

  // Access overlaps with Status such that Status bit 2 implies write access.
  // That is, if the high bit of Status is set, Write access is permitted.

  // Page fault / halt tests
  if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Access == 0){
    if(pS[I].microtrace){
      logmsgf(LT_LAMBDA,10,"VM: No-access page fault\n");
    }
    pS[I].Page_Fault = 1;
    return;
  }

  // Access 1 = Write Only
  // Does write access imply read access?
  if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Access == 1){ // && (access == VM_READ || access == VM_BYTE_READ)){
    // If we did not use force or force is disabled...
    if(force == 0 || pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Force_Allowed == 0){
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"VM: No-read-access page fault\n");
      }
      pS[I].Page_Fault = 1;
      return;
    }
  }

  // Access 2 = Read Only
  // if((pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Access&2) == 0 && (access == VM_WRITE || access == VM_BYTE_WRITE)){
  if((pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Access) == 2 && (access == VM_WRITE || access == VM_BYTE_WRITE)){
    // If we did not use force or force is disabled...
    if(force == 0 || pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Force_Allowed == 0){
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"VM: No-write-access page fault\n");
      }
      pS[I].Page_Fault = 1;
      return;
    }
  }

  #if 0
  // Otherwise, halt for inspection on these
  if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status != 00 &&
     pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status != 02 &&
     pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status != 03 &&
     pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status != 04 &&
     pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status != 05 &&
     pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status != 06){
    logmsgf(LT_LAMBDA,0,"VM: Unknown LV2 status bits: Investigate!\n");
    pS[I].cpu_die_rq = 1;
  }
  #endif

  // Index LV2
  pS[I].vm_phys_addr.Offset = pS[I].VMAregister.VM.Offset;
  pS[I].vm_phys_addr.PPN = pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].PPN;

  // Byte code is 2 bits, setting which byte is written by accesses to this page.
  // Basically it clobbers the low bits of the nubus address and forces byte mode.
  // There's one of these in LV2 CTL as well, called "packet code"
  // Testing says it's not supposed to mess with this stuff, but don't know how they're related
  // UPDATE: PACKET CODE INTERACTION!
  // It appears that Packet Code sets the byte flag, but Byte Code modifies the address as expected.
  // The bootstrap will set Packet Code to 1 with the Packetize flag set to 0.
  // This seems to indicate byte-ness.
  // Packet Code nonzero with Byte Code 0 means word!

  // FAR NEWER UPDATE
  // IF BYTE CODE IS NONZERO AND PACKET CODE IS SET, BYTE ACCESS
  // IF BYTE CODE IS NONZERO AND PACKET CODE IS CLEAR, HALFWORD ACCESS
  // IF BYTE CODE IS ZERO AND PACKET CODE IS CLEAR, WORD ACCESS
  // IF BYTE CODE IS ZERO AND PACKET CODE IS SET, BLOCK XFER IF CACHE ENABLED

  // Drive low address bits.
  pS[I].vm_phys_addr.Byte = pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].Byte_Code;
  // Assume not byte mode.
  pS[I].vm_byte_mode = 0;
  // Packet code set?
  if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packet_Code != 0){
    // Yes. Byte access?
    if(pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].Byte_Code > 0){
      // Byte access
      pS[I].vm_byte_mode = 2;
    }
    // If it wasn't, and cache is enabled, this will become a block xfer.
  }
  // If the packet code wasn't set, we have either a word or halfword access.

  /*
  pS[I].vm_byte_mode = 0;
  switch(pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].Byte_Code){
  case 0:
    pS[I].vm_phys_addr.Byte = 0;
    break;
  case 1:
    pS[I].vm_phys_addr.Byte = 1;
    break;
  case 2:
    pS[I].vm_phys_addr.Byte = 2;
    break;
  case 3:
    pS[I].vm_phys_addr.Byte = 3;
    break;
  }
  // Packet code set?
  if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packet_Code != 0){
    // Is the byte code enabled?
    if(pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].Byte_Code != 0){
      // If packet code is nonzero and byte access is set, this is a byte access.
      pS[I].vm_byte_mode = 2; // Byte access
    }else{
      // If byte access is not set, packet code drives cache transfer size.
    }
  }
  */

#ifdef SHADOW
  // Shadow memory maintenance if writing and not page fault.
  if(access == VM_WRITE || access == VM_BYTE_WRITE){
    // Update page table bits
    ShadowMemoryPageMap[pS[I].VMAregister.SM.VPage].Resident = 1;
    ShadowMemoryPageMap[pS[I].VMAregister.SM.VPage].Written = 1;
    // Store value
    shadow_write(pS[I].VMAregister.ptr,pS[I].MDregister);
  }
#endif

  // Log result!
  if(pS[I].microtrace){
    logmsgf(LT_LAMBDA,10,"VM: Resulting PA 0x%X\n",pS[I].vm_phys_addr.raw);
  }
}

// Source fetch handling
// source_mode is 1 if we are being clocked by an IMOD write.
void handle_source(int I,int source_mode){
  // Handle A Bus Input
  pS[I].Abus = pS[I].Amemory[pS[I].Iregister.ASource];
  // Handle M Bus Input
  if(pS[I].Iregister.MSource < 0100){
    pS[I].MFObus = pS[I].Mbus = pS[I].Mmemory[pS[I].Iregister.MSource];
    return;
  }else{
    switch(pS[I].Iregister.MSource){
    case 0100: // LAM-M-SRC-INTERRUPT-POINTER
      pS[I].Mbus = pS[I].InterruptVector;
      break;
    case 0101: // LAM-M-SRC-MACRO.IR.DISPLACEMENT
      if(((pS[I].LCregister.raw>>1)&0x01) == 0x01){
        // Left half
        pS[I].Mbus = pS[I].MIregister.mi[0].raw&077;
      }else{
        // Right half
        pS[I].Mbus = pS[I].MIregister.mi[1].raw&077;
      }
      if(pS[I].microtrace){
        logmsgf(LT_LAMBDA,10,"MACRO-IR-DISPLACEMENT: LC = 0x%X, MIR = 0x%X, Fetched 0x%X\n",
	       pS[I].LCregister.raw,pS[I].MIregister.raw,pS[I].Mbus);
      }
      break;
    case 0102: // LAM-M-SRC-STAT-COUNTER
      pS[I].Mbus = pS[I].stat_counter_main;
      break;
    case 0103: // LAM-M-SRC-MACRO.IR
      if(((pS[I].LCregister.raw>>1)&0x01) == 0x01){
        // Left half
        pS[I].Mbus = pS[I].MIregister.mi[0].raw;
      }else{
        // Right half
        pS[I].Mbus = pS[I].MIregister.mi[1].raw;
      }
      if(pS[I].microtrace){
        logmsgf(LT_LAMBDA,10,"MACRO-IR: LC = 0x%X, MIR = 0x%X, Fetched 0x%X\n",
	       pS[I].LCregister.raw,pS[I].MIregister.raw,pS[I].Mbus);
      }
      break;
    case 0104: // LAM-M-SRC-MACRO.IR.DECODE.RAM
      {
        // If Enable_Misc_MID is set, we dispatch on the whole MISC field of the instruction, if it is a MISC instruction to destination ignore (0).
        // MID memory 6000 - 7777 is used to hold dispatch addresses for MISC (6000-6777) and MISC1 (7000 - 7777).
        // So what we are doing is conditionally setting the high bits if this is an appropriate MISC op.
        if(pS[I].RG_Mode.Enable_Misc_MID != 0){ logmsgf(LT_LAMBDA,0,"EMM\n"); pS[I].cpu_die_rq = 1; }
        if(pS[I].MIregister.mi[0].raw != pS[I].MIregister.mi[1].raw){ logmsgf(LT_LAMBDA,0,"MIR\n"); pS[I].cpu_die_rq = 1; }
        // Generate address
        pS[I].MIDAddr.Opcode = pS[I].MIregister.mi[0].Misc.Opcode;
        pS[I].MIDAddr.Hi = pS[I].RG_Mode.MID_Hi_Adr;
        // Perform read
        pS[I].Mbus = pS[I].MIDmemory[pS[I].MIDAddr.raw];
        if(pS[I].microtrace){
          logmsgf(LT_LAMBDA,10,"MI: LAM-M-SRC-MACRO.IR.DECODE.RAM\n");
          logmsgf(LT_LAMBDA,10,"MID READ: MIR = 0x%X LC = 0x%X RG.Hi = %o RG.Enable_Misc_MID = %o (OPCODE %o) GENERATED ADDR %o DATA = 0x%X\n",
		 pS[I].MIregister.raw,pS[I].LCregister.raw,pS[I].RG_Mode.MID_Hi_Adr,pS[I].RG_Mode.Enable_Misc_MID,
		 pS[I].MIregister.mi[0].Misc.Opcode,pS[I].MIDAddr.raw,pS[I].Mbus);
        }
      }
      break;
    case 0105: // LAM-M-SRC-SPY-REG
      pS[I].Mbus = pS[I].SPY_Register;
      break;
    case 0106: // LAM-M-SRC-MULTIPLIER-FT
      pS[I].Mbus = pS[I].Multiplier_FT;
      break;
    case 0107: // LAM-M-SRC-RG-MODE
      pS[I].Mbus = pS[I].RG_Mode.raw;
      break;
    case 0110: // LAM-M-SRC-DISP-CONST
      pS[I].Mbus = pS[I].disp_constant_reg;
      break;
    case 0111: // LAM-M-SRC-MICRO-STACK
      // USP BITS 31-24, US BITS 18-0
      pS[I].Mbus = (pS[I].uPCS_ptr_reg&0xFF);
      pS[I].Mbus <<= 24;
      pS[I].Mbus |= (pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0x3FFFF);
      break;
    case 0112: // LAM-M-SRC-MICRO-STACK-POP
      if(source_mode == 1){ break; }
      // Same as above, but pops stack.
      if(pS[I].popj_after_nxt == 0){
        if(pS[I].microtrace){
          logmsgf(LT_LAMBDA,10,"PJAN armed with a MICRO-STACK-POP\n");
        }
        // PJAN is armed. We want this call to return to my caller instead of here.
        // So we'll pop this return address now.
        pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
        pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
        pS[I].popj_after_nxt = -1;
      }
      // Continue
      pS[I].Mbus = (pS[I].uPCS_ptr_reg&0xFF);
      pS[I].Mbus <<= 24;
      pS[I].Mbus |= (pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0x3FFFF);
      pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
      break;
    case 0121: // LAM-M-SRC-MD-NO-HOLD
      // Wait, so does this mean MD is supposed to be waiting for something to happen
      // before it returns?
      pS[I].Mbus = pS[I].MDregister.raw;
      break;
    case 0122: // LAM-M-SRC-VMA
      pS[I].Mbus = pS[I].VMAregister.raw;
      break;
    case 0123: // LAM-M-SRC-L1-MAP
      // Addressed by MD!
      pS[I].Mbus = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].raw;
      break;
    case 0124: // LAM-M-SRC-L2-MAP-CONTROL
      // Addressed by MD!
      pS[I].vm_lv2_index.raw = 0;
      pS[I].vm_lv2_index.VPage_Offset = pS[I].MDregister.VM.VPage_Offset;
      pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block;
      pS[I].Mbus = pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].raw;
      if(pS[I].microtrace){
        logmsgf(LT_LAMBDA,10,"VM: READ LV2 CTL ENT 0x%X\n",pS[I].vm_lv2_index.raw);
      }
      break;
    case 0125: // LAM-M-SRC-L2-MAP-PHYSICAL-PAGE
      // Addressed by MD!
      pS[I].vm_lv2_index.raw = 0;
      pS[I].vm_lv2_index.VPage_Offset = pS[I].MDregister.VM.VPage_Offset;
      pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block;
      pS[I].Mbus = pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].raw;
      if(pS[I].microtrace){
        logmsgf(LT_LAMBDA,10,"VM: READ LV2 ADR ENT 0x%X\n",pS[I].vm_lv2_index.raw);
      }
      break;
    case 0126: // LAM-M-SRC-LC
      pS[I].Mbus = pS[I].LCregister.raw;
      if(pS[I].microtrace || pS[I].macrotrace){
        logmsgf(LT_LAMBDA,10,"MI: LC READ: LC = 0x%X (0%o)\n",pS[I].LCregister.raw,pS[I].LCregister.raw);
      }
      break;
    case 0130: // LAM-M-SRC-PDL-BUFFER-INDEX
      pS[I].Mbus = pS[I].pdl_index_reg;
      break;
    case 0131: // LAM-M-SRC-Q
      pS[I].Mbus = pS[I].Qregister;
      break;
    case 0132: // LAM-M-SRC-PDL-BUFFER-POINTER
      pS[I].Mbus = pS[I].pdl_ptr_reg;
      break;
    case 0135: // LAM-M-SRC-DP-MODE
      pS[I].Mbus = pS[I].DP_Mode.raw;
      break;
    case 0136: // LAM-M-SRC-C-PDL-BUFFER-POINTER-POP
      if(source_mode == 1){ break; }
      // Read documentation for pS[I].DP_Mode
      if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
        pS[I].Mbus = pS[I].Mmemory[pS[I].pdl_ptr_reg];
        pS[I].pdl_ptr_reg--; pS[I].pdl_ptr_reg &= 0xFFF;
      }else{
        pS[I].Mbus = pS[I].Mmemory[0x800+pS[I].pdl_ptr_reg];
        pS[I].pdl_ptr_reg--; pS[I].pdl_ptr_reg &= 0x7FF;
      }
      if(pS[I].microtrace || pS[I].macrotrace){
        logmsgf(LT_LAMBDA,10,"MI: C-PDL-BUFFER-POINTER-POP: Addr Hi = 0x%X, NEW PTR = %o, DATA = %o\n",
	       pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_ptr_reg,pS[I].Mbus);
      }
      break;
    case 0137: // LAM-M-SRC-C-PDL-BUFFER-INDEX
      if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
        pS[I].Mbus = pS[I].Mmemory[pS[I].pdl_index_reg];
      }else{
        pS[I].Mbus = pS[I].Mmemory[0x800+pS[I].pdl_index_reg];
      }
      if(pS[I].microtrace || pS[I].macrotrace){
        logmsgf(LT_LAMBDA,10,"MI: C-PDL-BUFFER-INDEX: Addr Hi = %X, INDEX = %o, DATA = %o\n",
	       pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_index_reg,pS[I].Mbus);
      }
      break;
    case 0142: // LAM-M-SRC-STAT-COUNTER-AUX
      pS[I].Mbus = pS[I].stat_counter_aux;
      break;
    case 0144: // LAM-M-SRC-MACRO-IR-DECODE-MISC-ENABLE
      {
        DispatchWord disp_word;
        uint16_t inst;
        uint16_t addr;

        if(pS[I].RG_Mode.Enable_Misc_MID == 0){ logmsgf(LT_LAMBDA,0,"EMM-OFF\n"); pS[I].cpu_die_rq = 1; }
        // Generate address
        if(((pS[I].LCregister.raw>>1)&0x01) == 0x01){
          // Left half
          inst = pS[I].MIregister.mi[0].raw;
        }else{
          // Right half
          inst = pS[I].MIregister.mi[1].raw;
        }
	addr = 0;
        pS[I].MIDAddr.Hi = pS[I].RG_Mode.MID_Hi_Adr;
        if((inst&035000) == 015000){
          addr = 06000 + (inst&0777);
        }
        if((inst&035000) == 035000){
          addr = 07000 + (inst&0777);
        }

        disp_word.raw = pS[I].MIDmemory[addr];

        // Log it
        if(pS[I].microtrace){
          logmsgf(LT_LAMBDA,10,"MI: MACRO-IR-DISPATCH-MISC\n");
          logmsgf(LT_LAMBDA,10,"MID: MIR = 0x%X LC = 0x%X RG.Hi = %o RG.Enable_Misc_MID = %o: GENERATED ADDR %o DATA = 0x%X\n",
		 pS[I].MIregister.raw,pS[I].LCregister.raw,pS[I].RG_Mode.MID_Hi_Adr,
		 pS[I].RG_Mode.Enable_Misc_MID,addr,disp_word.raw);
          logmsgf(LT_LAMBDA,10," DEST ");
          {
            char *location;
            char symloc[100];
            int offset;
            location = "";
            offset = 0;
            location = sym_find_last(1, disp_word.PC, &offset);
            if(location != 0){
              if(offset != 0){
                sprintf(symloc, "%s+%o", location, offset);
              }else{
                sprintf(symloc, "%s", location);
              }
              logmsgf(LT_LAMBDA,10,"%s",symloc);
            }
            logmsgf(LT_LAMBDA,10," (%o)",disp_word.PC);
          }
          logmsgf(LT_LAMBDA,10,"\n");
        }
#ifdef XBEEP
	// Hack to capture calls to SYS:%BEEP. Rather than doing it at microcode level,
	// needing to interpret writes to toggle the loudspeaker,
	// do it at macrocode level (sort of), using SDL Audio functionality to make noise.
	// @@@@ what timing and other accounting needs to be taken care of?
	// @@@@ clock seems to catch up, magically?

	// Only use this code if the symbols can be found (too dangerous otherwise?).
	// Generalize to handle other functionality?
	if (xbeep_addr != 0 && xfalse_addr != 0 && disp_word.PC == xbeep_addr) { // XBEEP
	  // The beep params are on the call stack (first pop duration, then wavelength)
	  // See the case for LAM-M-SRC-C-PDL-BUFFER-POINTER-POP above
	  // Read documentation for pS[I].DP_Mode
	  uint32_t duration, wavelength;
	  if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
	    // pop duration
	    duration = pS[I].Mmemory[pS[I].pdl_ptr_reg];
	    pS[I].pdl_ptr_reg--; pS[I].pdl_ptr_reg &= 0xFFF;
	    // pop wavelength
	    wavelength = pS[I].Mmemory[pS[I].pdl_ptr_reg];
	    pS[I].pdl_ptr_reg--; pS[I].pdl_ptr_reg &= 0xFFF;
	  }else{
	    // pop duration
	    duration = pS[I].Mmemory[0x800+pS[I].pdl_ptr_reg];
	    pS[I].pdl_ptr_reg--; pS[I].pdl_ptr_reg &= 0x7FF;
	    // pop wavelength
	    wavelength = pS[I].Mmemory[0x800+pS[I].pdl_ptr_reg];
	    pS[I].pdl_ptr_reg--; pS[I].pdl_ptr_reg &= 0x7FF;
	  }
	  // Check datatypes and de-tag
	  if (((duration >> 031) & 037) != 5) // DTP-FIX
	    duration = 0;	// @@@@ maybe log an error?
	  else
	    duration &= 0177777777;  // 25 bit fixnum
	  if (((wavelength >> 031) & 037) != 5) // DTP-FIX
	    wavelength = 0;	// @@@@ maybe log an error?
	  else
	    wavelength &= 0177777777;  // 25 bit fixnum
	  // printf("XBEEP wavelength %d. duration %d.\n", wavelength, duration);
	  xbeep(wavelength, duration);
	  // printf("MID: patch XBEEP => XFALSE\n");
	  // just return false, never call the XBEEP routine
	  disp_word.PC = xfalse_addr; // XFALSE
	}
	// end hack
#endif
        pS[I].Mbus = disp_word.PC;
      }
      break;
    case 0146: // LAM-M-SRC-MULTIPLIER
      pS[I].Mbus = pS[I].Multiplier_Output;
      break;
    case 0153: // ???
      // This is apparently a typo in the microcode.
      pS[I].Mbus = 0;
      break;
    case 0161: // LAM-M-SRC-MD
      pS[I].Mbus = pS[I].MDregister.raw;
      break;
    case 0176: // LAM-M-SRC-C-PDL-BUFFER-POINTER
      if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
        pS[I].Mbus = pS[I].Mmemory[pS[I].pdl_ptr_reg];
      }else{
        pS[I].Mbus = pS[I].Mmemory[0x800+pS[I].pdl_ptr_reg];
      }
      if(pS[I].microtrace || pS[I].macrotrace){
        logmsgf(LT_LAMBDA,10,"MI: C-PDL-BUFFER-POINTER: Addr Hi = 0x%X, INDEX = %o, DATA = %o\n",
	       pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_index_reg,pS[I].Mbus);
      }
      break;
    case 0177: // Unknown, used by lambda-diag
      pS[I].Mbus = 0;
      break;
    default:
      logmsgf(LT_LAMBDA,0,"Unknown MF-Source %o\n",pS[I].Iregister.MSource);
      pS[I].cpu_die_rq = 1;
    }
    // Load MFO bus from M bus (for source cycle)
    pS[I].MFObus = pS[I].Mbus;
    return;
  }
}

#ifdef CONFIG_CACHE
// Age all cache sectors
// Vectorization wasn't particularly useful here and is generally disabled with -Og anyway.
void Age_All_Cache_Sectors(int I, int sector){
  int x = 0;
  // Ensure our sector doesn't become 255, to save one test inside the loop.
  pS[I].Cache_Sector_Age[sector] = 0;
  // Age all sectors, tracking the oldest.
  while(x < 256){
    pS[I].Cache_Sector_Age[x]++;
    if(pS[I].Cache_Sector_Age[x] == 255){ pS[I].Cache_Oldest_Sector = x; }
    x++;
  }
  // Reset our sector's age to zero.
  pS[I].Cache_Sector_Age[sector] = 0;
}

// Age cache sectors newer than ours.
// We aren't aging the whole cache, and we aren't aging the oldest sector.
void Age_Cache_Sectors(int I,int sector){
  int x = 0;
  int age_above = pS[I].Cache_Sector_Age[sector];
  int sectors_aged = 0;
  // Loop sectors.
  while(x < 256){
    // If this sector is newer than ours, age it.
    if(pS[I].Cache_Sector_Age[x] <= age_above){
      // logmsgf(LT_LAMBDA,3,"CACHE: AGING SECTOR %d: %d -> %d\n",x,pS[I].Cache_Sector_Age[x],pS[I].Cache_Sector_Age[x]+1);
      pS[I].Cache_Sector_Age[x]++;
      sectors_aged++;
      // If we aged our sector and everything newer, we are done.
      if(sectors_aged == age_above+1){
	// Reset age of our sector
	pS[I].Cache_Sector_Age[sector] = 0;
	// We are done!
	return;
      }
    }
    x++;
  }
}

// Take and release cache write-check mutex
void take_cache_wc(int I){
  int result = pthread_mutex_lock(&cache_wc_mutex[I]);
  if(result != 0){
    logmsgf(LT_LAMBDA,0,"Unable to take cache writecheck mutex %d: %s\n",I,strerror(result));
    exit(-1);
  }
}

void release_cache_wc(int I){
  int result = pthread_mutex_unlock(&cache_wc_mutex[I]);
  if(result != 0){
    logmsgf(LT_LAMBDA,0,"Unable to release cache writecheck mutex %d: %s\n",I,strerror(result));
    exit(-1);
  }
}

// Called by nubus when something else updates the cache
void cache_write_check(int access, int I, uint32_t address, uint32_t data){
  // Is this in the cache?
  int sector = 0;
  uint32_t Sector_Addr = address&0xFFFFFFF0;
  uint32_t WCStatus_Offset = ((address&0x0FFFFFF0)>>4);
  uint8_t Sector_Offset = (address&0xC)>>2;
  // If this sector is allocated...
  while(cache_wc_status[I][WCStatus_Offset] != 0){
    // Check its address
    sector = (cache_wc_status[I][WCStatus_Offset]&0xFF);
    // If it matched...
    if(pS[I].Cache_Sector_Addr[sector] == Sector_Addr){
      // HIT
      take_cache_wc(I);
      // Do we still have it?
      if(pS[I].Cache_Sector_Addr[sector] == Sector_Addr){
	// Yes
	// logmsgf(LT_LAMBDA,2,"CACHE: WRITE CHECK HIT: Request %o Addr 0x%X w/ data 0x%X\n",access,address,data);
	switch(access){
	case VM_WRITE:
	  switch(address&0x03){
	  case 0:
	    // FULL WORD
	    pS[I].Cache_Data[sector][Sector_Offset].word = data;
	    pS[I].Cache_Status[sector][Sector_Offset] = 0x0F;
	    break;
	  case 1:
	    // Low Half
	    pS[I].Cache_Data[sector][Sector_Offset].word &= 0xFFFF0000;
	    pS[I].Cache_Data[sector][Sector_Offset].word |= (data&0xFFFF);
	    pS[I].Cache_Status[sector][Sector_Offset] |= 0x03;
	    break;
	  case 3:
	    // High half
	    pS[I].Cache_Data[sector][Sector_Offset].word &= 0x0000FFFF;
	    pS[I].Cache_Data[sector][Sector_Offset].word |= (data&0xFFFF0000);
	    pS[I].Cache_Status[sector][Sector_Offset] |= 0x0C;
	    break;
	  case 2:
	    // Block (Invalid)
	  default:
	    logmsgf(LT_LAMBDA,0,"CACHE: WRITE CHECK HIT W/ UI MODE %o\n",address&0x03);
	    exit(-1);
	  }
	  break;
	case VM_BYTE_WRITE:
	default:
	  logmsgf(LT_LAMBDA,0,"CACHE: WRITE CHECK HIT W/ UI OP %o\n",access);
	  exit(-1);
	}
	release_cache_wc(I);
	return; // Bail out
      }else{
	// We lost it.
	logmsgf(LT_LAMBDA,2,"CACHE: WRITE CHECK HIT LOST BEFORE CACHE WC MUTEX COULD BE OBTAINED\n");
	release_cache_wc(I);
	continue; // Try again
      }
    }else{
      // We lost the match before we got here.
      logmsgf(LT_LAMBDA,2,"CACHE: WRITE CHECK HIT LOST BEFORE FIRST TEST\n");
      continue;
    }
    // We should not get here, but in case we do...
    break;
  }
}
#endif

// Local Bus Interface
void lcbus_io_request(int access, int I, uint32_t address, uint32_t data){
#ifdef CONFIG_CACHE
  int nubus_required = 1;
#endif
  // Hrm...
  if(pS[I].SM_Clock_Pulse > 0){
    logmsgf(LT_LAMBDA,0,"LCBUS: DANGER WILL ROBINSON! LCBUS ACCESS IN SM CLOCK PULSE!\n");
  }
  // Clear flags
  pS[I].LCbus_error = 0;
  pS[I].LCbus_acknowledge = 0;
  // Log
  /*
  if(NUbus_trace == 1){
    logmsgf(LT_NUBUS,10,"NUBUS: Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) by dev 0x%X\n",
	    access,address,address,data,data,master);
  }
  */
  // Put request on bus
  pS[I].LCbus_Busy = 3;
  pS[I].LCbus_Address.raw = address;
  pS[I].LCbus_Request = access;
  pS[I].LCbus_Data.word = data;
#ifdef CONFIG_CACHE
  // CACHE TEST
  if(pS[I].Cache_Permit != 0){
    int sector = 0;
    uint32_t Sector_Addr = pS[I].LCbus_Address.raw&0xFFFFFFF0;
    uint32_t WCStatus_Offset = ((pS[I].LCbus_Address.raw&0x0FFFFFF0)>>4);
    uint8_t Sector_Offset = (pS[I].LCbus_Address.raw&0xC)>>2;
    /*
    logmsgf(LT_LAMBDA,2,"CACHE: Request %o Addr 0x%X w/ data 0x%X\n",
	    pS[I].LCbus_Request,pS[I].LCbus_Address.raw,pS[I].LCbus_Data.word);
    logmsgf(LT_LAMBDA,2,"CACHE: Packet Code %o, Packetize Writes %o\n",pS[I].Packet_Code,pS[I].Packetize_Writes);
    */
    // Is this in the cache already?
    if(cache_wc_status[I][WCStatus_Offset] == 0){
      // No, don't bother searching.
      sector = 256;
    }else{
      // Otherwise, where is it?
      /*
      while(sector < 256){
	if(pS[I].Cache_Sector_Addr[sector] == Sector_Addr){
	  // FOUND IT
	  break;
	}
	sector++;
      }
      */
      sector = cache_wc_status[I][WCStatus_Offset]&0xFF;
    }
    if(sector == 256){
      // SECTOR MISS
      // logmsgf(LT_LAMBDA,2,"CACHE: SECTOR MISS\n");
      // Find an open sector
      if(pS[I].Cache_Full == 1){
	// CACHE FULL
	int y = 0;
	sector = pS[I].Cache_Oldest_Sector;
	// logmsgf(LT_LAMBDA,0,"CACHE FULL: EVICTING SECTOR %d\n",sector);
	// This is now a sector hit, etc.
	pS[I].Cache_Sector_Hit = 1;
	pS[I].Cache_Sector = sector;
	// Rewrite address and invalidate data
	take_cache_wc(I);
	cache_wc_status[I][(uint32_t)((pS[I].Cache_Sector_Addr[sector]&0x0FFFFFF0)>>4)] = 0;
	pS[I].Cache_Sector_Addr[sector] = Sector_Addr;
	cache_wc_status[I][WCStatus_Offset] = sector|0x100;
	release_cache_wc(I);
	while(y < 4){
	  pS[I].Cache_Status[sector][y] = 0;
	  pS[I].Cache_Data[sector][y].word = 0;
	  y++;
	}
      }else{
	// CACHE NOT FULL
	// For now, the oldest sector is simply the most recently allocated.
	sector = pS[I].Cache_Oldest_Sector;
	pS[I].Cache_Oldest_Sector++;
	take_cache_wc(I);
	cache_wc_status[I][WCStatus_Offset] = sector|0x100;
	pS[I].Cache_Sector_Addr[sector] = Sector_Addr;
	release_cache_wc(I);
	pS[I].Cache_Sector_Age[sector] = 0; // New sector
	// If we just allocated the last sector
	if(sector == 255){
	  // Cache is full, oldest sector is now sector 0.
	  pS[I].Cache_Oldest_Sector = 0;
	  pS[I].Cache_Full = 1;
	}
	// Set up hit
	pS[I].Cache_Sector_Hit = 1;
	pS[I].Cache_Sector = sector;
      }
      // We either allocated a new sector or aged the oldest sector, so age the whole cache.
      Age_All_Cache_Sectors(I,sector);
      // Now handle the access.
      // If this is a word write, we can store directly
      switch(pS[I].LCbus_Request){
      case VM_WRITE:
	switch(pS[I].LCbus_Address.raw&0x3){
	case 0:
	  // FULL WORD
	  // logmsgf(LT_LAMBDA,2,"CACHE: WORD WRITE - STORED DIRECTLY\n");
	  pS[I].Cache_Data[sector][Sector_Offset].word = pS[I].LCbus_Data.word;
	  pS[I].Cache_Status[sector][Sector_Offset] = 0x0F;
	  break;
	case 1: // LOW HALF
	case 3: // HI HALF
	case 2: // BLOCK (invalid)
	default:
	  logmsgf(LT_LAMBDA,0,"CACHE: WORD WRITE - UI MODE %o\n",pS[I].LCbus_Address.raw&0x3);
	  exit(-1);
	}
	// Still needs bus for backing write
	break;
      case VM_READ:
	if((pS[I].LCbus_Address.raw&0x3) == 0 && pS[I].Packet_Code != 0){
	  // BLOCK TRANSFER
	  pS[I].LCbus_Address.raw |= 0x2; // Set block transfer mode
	  // logmsgf(LT_LAMBDA,2,"CACHE: WORD READ - DOING BLOCK TRANSFER FROM 0x%.8X\n",pS[I].LCbus_Address.raw);
	}else{
	  // logmsgf(LT_LAMBDA,2,"CACHE: WORD READ - DOING BUS READ\n");
	}
	// Needs bus to fill cache
	break;
      default:
	logmsgf(LT_LAMBDA,0,"CACHE: SECTOR MISS W/ UI OP %o\n",pS[I].LCbus_Request);
	exit(-1);
	break;
      }
    }else{
      // SECTOR HIT
      // logmsgf(LT_LAMBDA,2,"CACHE: SECTOR HIT, SECTOR %d\n",sector);
      pS[I].Cache_Sector_Hit = 1;
      pS[I].Cache_Sector = sector;
      // Are we aging it?
      if(pS[I].Cache_Sector_Age[sector] != 0){
	// Yes. Does it happen to be the oldest sector?
	if(pS[I].Cache_Sector_Age[sector] == 255){
	  // Yes
	  Age_All_Cache_Sectors(I,sector);
	}else{
	  // Otherwise do conditional aging.
	  Age_Cache_Sectors(I,sector);
	}
      }
      // Data hit?
      switch(pS[I].LCbus_Request){
      case VM_WRITE:
	// Store the word and carry on for the backing write.
	switch(pS[I].LCbus_Address.raw&0x3){
	case 0:
	  // FULL WORD
	  // logmsgf(LT_LAMBDA,2,"CACHE: WORD WRITE - STORED DIRECTLY\n");
	  pS[I].Cache_Data[sector][Sector_Offset].word = pS[I].LCbus_Data.word;
	  pS[I].Cache_Status[sector][Sector_Offset] = 0x0F;
	  break;
	case 1: // LOW HALF
	case 3: // HI HALF
	case 2: // BLOCK (invalid)
	default:
	  logmsgf(LT_LAMBDA,0,"CACHE: WORD WRITE - UI MODE %o\n",pS[I].LCbus_Address.raw&0x3);
	  exit(-1);
	}
	break;

      case VM_READ:
	// Do we have it?
	switch(pS[I].LCbus_Address.raw&0x3){
	case 0: // FULL WORD
	  // Requires the whole word.
	  if(pS[I].Cache_Status[sector][Sector_Offset] == 0x0F){
	    // CACHE HIT
	    // logmsgf(LT_LAMBDA,2,"CACHE: CACHE HIT - WORD READ\n");
	    nubus_required = 0; // Don't need the bus
	  }else{
	    // CACHE MISS
	    if(pS[I].Packet_Code != 0){
	      // BLOCK TRANSFER
	      pS[I].LCbus_Address.raw |= 0x2; // Set block transfer mode
	      // logmsgf(LT_LAMBDA,2,"CACHE: CACHE MISS - DOING BLOCK TRANSFER FROM 0x%.8X\n",pS[I].LCbus_Address.raw);
	    }else{
	      // logmsgf(LT_LAMBDA,2,"CACHE: CACHE MISS - WORD READ\n");
	    }
	  }
	  break;
	case 1: // LOW HALF
	case 3: // HI HALF
	case 2: // BLOCK (invalid)
	default:
	  logmsgf(LT_LAMBDA,0,"CACHE: WORD READ - UI MODE %o\n",pS[I].LCbus_Address.raw&0x3);
	  exit(-1);
	}
	break;

      default:
	logmsgf(LT_LAMBDA,0,"CACHE: SECTOR HIT W/ UI OP %o\n",pS[I].LCbus_Request);
	pS[I].cpu_die_rq = 1;
	break;
      }
    }
    // We need the NUbus.
    if(nubus_required == 1){
      // logmsgf(LT_LAMBDA,2,"CACHE: Taking nubus mastership and issuing request\n");
      maybe_take_nubus_mastership(I);
      nubus_io_request(pS[I].LCbus_Request,pS[I].NUbus_ID,pS[I].LCbus_Address.raw,pS[I].LCbus_Data.word);
    }else{
      if(pS[I].NUbus_Master == 1){
	logmsgf(LT_LAMBDA,0,"CACHE: NO NEED FOR BUS BUT WE STILL HAVE IT?\n");
	exit(-1);
      }
    }
    return;
    // End of cache test
  }
#endif
  maybe_take_nubus_mastership(I);
  nubus_io_request(access,pS[I].NUbus_ID,address,data);
}

// Destination selector handling
void handle_destination(int I){
  if(pS[I].Iregister.Destination.A.Flag != 0){
    // A-Memory store
    pS[I].Amemory[pS[I].Iregister.Destination.A.Addr] = pS[I].Obus;
    if(pS[I].Iregister.Destination.A.Addr == mouse_x_loc[I] || pS[I].Iregister.Destination.A.Addr == mouse_y_loc[I]){
      warp_mouse_callback(I);
    }
  }else{
    // A+M-Memory and/or functional destination store
    pS[I].Amemory[pS[I].Iregister.Destination.M.Addr] = pS[I].Obus;
    pS[I].Mmemory[pS[I].Iregister.Destination.M.Addr] = pS[I].Obus;
    if(pS[I].Iregister.Destination.F.Dest > 0){
      switch(pS[I].Iregister.Destination.F.Dest){
      case 001: // LAM-FUNC-DEST-LC
	// On Raven, writing LC also sets needfetch.
	// Let's do that here too.
	pS[I].RG_Mode.Need_Macro_Inst_Fetch = 1;
	if((pS[I].Obus&1) == 1){
	  pS[I].Obus--;
	}
	pS[I].LCregister.raw = pS[I].Obus;
	if(pS[I].microtrace || pS[I].macrotrace){
	  logmsgf(LT_LAMBDA,10,"MI: LC SET: NEW LC = 0x%X (0%o), need-fetch set\n",
		 pS[I].LCregister.raw,pS[I].LCregister.raw);
	}
	break;
      case 002: // LAM-FUNC-DEST-DP-MODE
	// Only the low 6 bits are writable
	pS[I].DP_Mode.raw &= 0xC0;
	pS[I].DP_Mode.raw |= pS[I].Obus&0x3F;
	break;
      case 004: // LAM-FUNC-DEST-STAT-COUNTER-AUX
	pS[I].stat_counter_aux = pS[I].Obus;
	break;
      case 005: // LAM-FUNC-DEST-MID
	// Macro Instruction Dispatch RAM
	// Addressed by MIR contents
	// Stop conditions (for investigation)
	// if(pS[I].RG_Mode.MID_Hi_Adr != 03){ pS[I].cpu_die_rq = 1; }
	// if(pS[I].RG_Mode.MID_Hi_Adr != 0 && pS[I].RG_Mode.MID_Hi_Adr != 1){ logmsgf(LT_LAMBDA,10,"MIHI\n"); pS[I].cpu_die_rq = 1; }
	if(pS[I].RG_Mode.Enable_Misc_MID != 0){ logmsgf(LT_LAMBDA,0,"EMM\n"); pS[I].cpu_die_rq = 1; }
	if(pS[I].MIregister.mi[0].raw != pS[I].MIregister.mi[1].raw){ logmsgf(LT_LAMBDA,0,"MIR\n"); pS[I].cpu_die_rq = 1; }
	// Generate address
	pS[I].MIDAddr.Opcode = pS[I].MIregister.mi[0].Misc.Opcode;
	pS[I].MIDAddr.Hi = pS[I].RG_Mode.MID_Hi_Adr;
	// Perform write
	pS[I].MIDmemory[pS[I].MIDAddr.raw] = pS[I].Obus;
	// Log it
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"MID WRITE: MIR = 0x%X RG.Hi = %o RG.Enable_Misc_MID = %o (OPCODE %o) GENERATED ADDR %o DATA = 0x%X\n",
		 pS[I].MIregister.raw,pS[I].RG_Mode.MID_Hi_Adr,pS[I].RG_Mode.Enable_Misc_MID,
		 pS[I].MIregister.mi[0].Misc.Opcode,pS[I].MIDAddr.raw,pS[I].Obus);
	}
	break;
      case 006: // LAM-FUNC-DEST-CRAM-HIGH
	// Same trick as LAM-FUNC-DEST-CRAM-MAP?
	// Addressed by the previous instruction, so loc_ctr_reg
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"CRAM WRITE HI: Addr %o w/ data %o\n",pS[I].loc_ctr_reg.raw,pS[I].Obus);
	}
        {
          int addr = pS[I].loc_ctr_reg.raw;
          int paddr = pS[I].CRAM_map[addr>>4]&03777;
          uint64_t tmp = (pS[I].Obus&0xFFFFFFFF);

          paddr <<= 4;
          paddr |= (addr&0xf);
          tmp <<= 32;
          pS[I].WCS[paddr].raw &= 0xFFFFFFFF;
          pS[I].WCS[paddr].raw |= tmp;
          pS[I].cram_write_cyc = true;
        }
	break;
      case 007: // LAM-FUNC-DEST-CRAM-LOW
	// Same trick as LAM-FUNC-DEST-CRAM-MAP?
	// Addressed by the previous instruction, so pS[I].loc_ctr_reg
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"CRAM WRITE LO: Addr %o w/ data %o\n",pS[I].loc_ctr_reg.raw,pS[I].Obus);
	}
        {
          int addr = pS[I].loc_ctr_reg.raw;
          int paddr = pS[I].CRAM_map[addr>>4]&03777;
          paddr <<= 4;
          paddr |= (addr&0xf);

          pS[I].WCS[paddr].raw &= 0xFFFFFFFF00000000LL;
          pS[I].WCS[paddr].raw |= (pS[I].Obus&0xFFFFFFFF);
          pS[I].cram_write_cyc = true;
        }
	break;
      case 010: // LAM-FUNC-DEST-C-PDL-BUFFER-POINTER
	// See documentation for DP_Mode register
	if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
	  pS[I].pdl_ptr_reg &= 0xFFF;
	  pS[I].Mmemory[pS[I].pdl_ptr_reg] = pS[I].Obus;
	}else{
	  pS[I].pdl_ptr_reg &= 0x7FF;
	  pS[I].Mmemory[0x800+pS[I].pdl_ptr_reg] = pS[I].Obus;
	}
        if(pS[I].microtrace || pS[I].macrotrace){
	  logmsgf(LT_LAMBDA,10,"MI: C-PDL-BUFFER-POINTER: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
		 pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_ptr_reg,pS[I].Obus);
	}
	break;
      case 011: // LAM-FUNC-DEST-C-PDL-BUFFER-POINTER-PUSH
	// See documentation for DP_Mode register
	pS[I].pdl_ptr_reg++;
	if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
	  pS[I].pdl_ptr_reg &= 0xFFF;
	  pS[I].Mmemory[pS[I].pdl_ptr_reg] = pS[I].Obus;
	}else{
	  pS[I].pdl_ptr_reg &= 0x7FF;
	  pS[I].Mmemory[0x800+pS[I].pdl_ptr_reg] = pS[I].Obus;
	}
        if(pS[I].microtrace || pS[I].macrotrace){
	  logmsgf(LT_LAMBDA,10,"MI: C-PDL-BUFFER-POINTER-PUSH: Addr Hi = 0x%X, NEW PTR = %o, DATA = %o\n",
		 pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_ptr_reg,pS[I].Obus);
	}
	break;
      case 012: // LAM-FUNC-DEST-C-PDL-BUFFER-INDEX
        // See documentation for DP_Mode register
        if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
          pS[I].pdl_index_reg &= 0xFFF;
          pS[I].Mmemory[pS[I].pdl_index_reg] = pS[I].Obus;
        }else{
          pS[I].pdl_ptr_reg &= 0x7FF;
          pS[I].Mmemory[0x800+pS[I].pdl_index_reg] = pS[I].Obus;
        }
        if(pS[I].microtrace || pS[I].macrotrace){
          logmsgf(LT_LAMBDA,10,"MI: C-PDL-BUFFER-INDEX: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
		 pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_index_reg,pS[I].Obus);
        }
        break;
      case 013: // LAM-FUNC-DEST-PDL-BUFFER-POINTER
        // See documentation for pS[I].DP_Mode register
	// I assume this is right.
	if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
	  pS[I].pdl_ptr_reg = pS[I].Obus&0xFFF;
	}else{
	  pS[I].pdl_ptr_reg = pS[I].Obus&0x7FF;
	}
        if(pS[I].microtrace || pS[I].macrotrace){
	  logmsgf(LT_LAMBDA,10,"MI: PDL-BUFFER-POINTER: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
		 pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_ptr_reg,pS[I].Obus);
	}
	break;
      case 015: // LAM-FUNC-DEST-MICRO-STACK-PUSH
	pS[I].uPCS_ptr_reg++; pS[I].uPCS_ptr_reg &= 0xFF;
	pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].Obus;
	if(pS[I].microtrace){
          char *location;
          char symloc[100];
          int offset;

          location = "";
          offset = 0;
          location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	  if(location != 0){
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	  }else{
	    // No symbol
	    symloc[0] = 0;
	  }
	  logmsgf(LT_LAMBDA,10,"uStack[%o] = %s (%o)\n",pS[I].uPCS_ptr_reg,symloc,pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
	}
	break;
      case 016: // LAM-FUNC-DEST-IMOD-LOW
	pS[I].imod_lo = pS[I].Obus;
	pS[I].imod_en = 1;
	break;
      case 017: // LAM-FUNC-DEST-IMOD-HIGH
	pS[I].imod_hi = pS[I].Obus;
	pS[I].imod_en = 1;
	break;
      case 020: // LAM-FUNC-DEST-VMA
	pS[I].VMAregister.raw = pS[I].Obus; // Load VMA
	break;
      case 021: // LAM-FUNC-DEST-VMA-START-READ
	// Load VMA from pS[I].Obus and initiate a read.
	pS[I].VMAregister.raw = pS[I].Obus; // Load VMA
	VM_resolve_address(I,VM_READ,0);
	if(pS[I].Page_Fault == 0 && pS[I].ConReg.Enable_NU_Master == 1){
	  // Do it
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control == 01){
	    pS[I].stat_counter_aux++;
	  }
	  if(pS[I].RG_Mode.Main_Stat_Count_Control == 01){
	    pS[I].stat_counter_main++;
	  }
	  lcbus_io_request(pS[I].vm_byte_mode|VM_READ,I,pS[I].vm_phys_addr.raw,0);
	}
	break;
      case 022: // LAM-FUNC-DEST-VMA-START-WRITE
	// Load VMA from pS[I].Obus and initiate a write.
	pS[I].VMAregister.raw = pS[I].Obus; // Load VMA
	VM_resolve_address(I,VM_WRITE,0);
	if(pS[I].Page_Fault == 0 && pS[I].ConReg.Enable_NU_Master == 1){
	  // Do it
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control == 01){
	    pS[I].stat_counter_aux++;
	  }
	  if(pS[I].RG_Mode.Main_Stat_Count_Control == 01){
	    pS[I].stat_counter_main++;
	  }
	  lcbus_io_request(pS[I].vm_byte_mode|VM_WRITE,I,pS[I].vm_phys_addr.raw,pS[I].MDregister.raw);
	}
	break;
      case 023: // LAM-FUNC-DEST-L1-MAP
	// Requires SLOW-DEST
	// Addressed by MD
	// 20000 per page
	// pS[I].vm_lv1_map[ldb(pS[I].MDregister.raw,12,13)].raw = pS[I].Obus;
	pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].raw = pS[I].Obus;
	// cached_lv1 = pS[I].VMAregister;
	if(pS[I].microtrace){
          logmsgf(LT_LAMBDA,10,"VM: WRITE LV1 ENT 0x%X DATA 0x%X RESULT 0x%X (Meta %o Validity %o LV2_Block %o)\n",
		 pS[I].MDregister.VM.VPage_Block,
		 pS[I].Obus,
		 pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].raw,
		 pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB,
		 pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid,
		 pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block);
        }
        break;
      case 024: // LAM-FUNC-DEST-L2-MAP-CONTROL
	// Requires SLOW-DEST
	// Addressed by MD
        // 400 per page
	pS[I].vm_lv2_index.raw = 0;
	pS[I].vm_lv2_index.VPage_Offset = pS[I].MDregister.VM.VPage_Offset;
	pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block;
	pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].raw = pS[I].Obus;
	// If we wrote something we don't understand, stop.
	/*
	if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Byte_Code != 0){
	  logmsgf(LT_LAMBDA,10,"LV2 CTL BYTE CODE WRITE\n");
	  pS[I].cpu_die_rq = 1;
	}
	*/
	if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packetize_Writes){ logmsgf(LT_LAMBDA,0,"LV2 CTL PACKETIZED WRITE\n"); pS[I].cpu_die_rq = 1; }
	if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Lock_NUbus){ logmsgf(LT_LAMBDA,0,"LV2 CTL LOCK NUBUS\n"); pS[I].cpu_die_rq = 1; }
	if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Unused != 0){
	  logmsgf(LT_LAMBDA,0,"LV2 CTL UNUSED WRITE\n");
	  pS[I].cpu_die_rq = 1;
	}
	// Invert the meta bits?
        if(pS[I].microtrace || pS[I].cpu_die_rq == 1){
	  logmsgf(LT_LAMBDA,10,"VM: WRITE LV2 CTL ENT 0x%X (Meta %o Status %o Access %o Force-Allowed %o Packet-Code %o Packetize-Writes %o Enable-Cache %o Lock-Nubus %o Unused %o)\n",
		 pS[I].vm_lv2_index.raw,pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta,pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status,
		 pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Access,pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Force_Allowed,
		 pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packet_Code,pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packetize_Writes,
		 pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Cache_Permit,pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Lock_NUbus,
		 pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Unused);
	}
	break;
      case 026: // LAM-FUNC-DEST-CRAM-MAP
	// WWII?
	// Addressed by the previous instruction, so pS[I].loc_ctr_reg
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"CRAM MAP WRITE: Addr %o w/ data %o\n",
		 pS[I].loc_ctr_reg.Page,pS[I].Obus);
	}
	pS[I].CRAM_map[pS[I].loc_ctr_reg.Page] = pS[I].Obus;
	pS[I].cram_write_cyc = true;
	// if(pS[I].loc_ctr_reg.Page < 07770){ pS[I].cpu_die_rq = 1; }
	break;
      case 030: // LAM-FUNC-DEST-MD
	pS[I].MDregister.raw = pS[I].Obus;
	// cached_lv1 = pS[I].vm_lv1_map[ldb(pS[I].Obus,12,13)];
	break;
      case 032: // LAM-FUNC-DEST-MD-START-WRITE
	pS[I].MDregister.raw = pS[I].Obus;
	VM_resolve_address(I,VM_WRITE,0);
	if(pS[I].Page_Fault == 0 && pS[I].ConReg.Enable_NU_Master == 1){
	  // Do it
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control == 01){
	    pS[I].stat_counter_aux++;
	  }
	  if(pS[I].RG_Mode.Main_Stat_Count_Control == 01){
	    pS[I].stat_counter_main++;
	  }
	  lcbus_io_request(pS[I].vm_byte_mode|VM_WRITE,I,pS[I].vm_phys_addr.raw,pS[I].MDregister.raw);
	}
	break;
      case 033: // LAM-FUNC-DEST-C-PDL-INDEX-INC
        // See documentation for DP_Mode register
	pS[I].pdl_index_reg++;
        if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
          pS[I].pdl_index_reg &= 0xFFF;
          pS[I].Mmemory[pS[I].pdl_index_reg] = pS[I].Obus;
        }else{
          pS[I].pdl_ptr_reg &= 0x7FF;
          pS[I].Mmemory[0x800+pS[I].pdl_index_reg] = pS[I].Obus;
        }
        if(pS[I].microtrace || pS[I].macrotrace){
          logmsgf(LT_LAMBDA,10,"MI: C-PDL-INDEX-INC: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
		 pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_index_reg,pS[I].Obus);
        }
        break;
      case 035: // LAM-FUNC-DEST-MICRO-STACK-POINTER-IF-POP (used by LAM)
	// "COMPLICATED. Works only on POP"
	if(pS[I].Iregister.MSource == 0112){
	  pS[I].pdl_index_reg = (pS[I].Obus&0xFFF);
	}
	break;
      case 036: // LAM-FUNC-DEST-C-PDL-INDEX-DEC
        // See documentation for DP_Mode register
	pS[I].pdl_index_reg--;
        if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
          pS[I].pdl_index_reg &= 0xFFF;
          pS[I].Mmemory[pS[I].pdl_index_reg] = pS[I].Obus;
        }else{
          pS[I].pdl_ptr_reg &= 0x7FF;
          pS[I].Mmemory[0x800+pS[I].pdl_index_reg] = pS[I].Obus;
        }
        if(pS[I].microtrace || pS[I].macrotrace){
          logmsgf(LT_LAMBDA,10,"MI: C-PDL-INDEX-DEC: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
		 pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_index_reg,pS[I].Obus);
        }
        break;
      case 037: // LAM-FUNC-DEST-MULTIPLIER
	{
	  uint16_t x,y;
	  // This writes the 16x16 multipler chip
	  pS[I].Multiplier_Input = pS[I].Obus;
	  // Operate multiplier
	  x = pS[I].Multiplier_Input&0xFFFF;
	  y = ((pS[I].Multiplier_Input>>16)&0xFFFF);
	  pS[I].Multiplier_FT = x*y;
	  // Generate output too
	  // x = pS[I].Multiplier_Input&0x7FFF;
	  // y = ((pS[I].Multiplier_Input>>15)&0x7FFF);
	  pS[I].Multiplier_Output = pS[I].Multiplier_FT;
	  if(pS[I].microtrace){
	    logmsgf(LT_LAMBDA,10,"MULTIPLIER: INPUT = 0x%X, X = 0x%X, Y = 0x%X, FT = 0x%X, OUT = 0x%X\n",
		   pS[I].Multiplier_Input,x,y,pS[I].Multiplier_FT,pS[I].Multiplier_Output);
	  }
	}
	break;
      case 041: // LAM-FUNC-DEST-INTERRUPT-CLEAR
	// Clear the active interrupt
	if(pS[I].InterruptPending > 0){
	  pS[I].InterruptPending--;
	}
	if(pS[I].InterruptVector != 0){
	  pS[I].InterruptStatus[pS[I].InterruptVector] = 0;
	  pS[I].InterruptVector = 0;
	}
	break;
      case 042: // LAM-FUNC-DEST-RG-MODE
	// THE WHOLE RG MODE REGISTER NOW LOOKS LIKE 12 READ/WRITE BITS, 8 READ-ONLY BITS, AND THEN 12 MORE READ/WRITE BITS
	pS[I].RG_Mode.raw &= 0x000FF000;
	pS[I].RG_Mode.raw |= (pS[I].Obus&0xFFF00FFF);
	// Investigation stops
	// if(pS[I].RG_Mode.Sequence_Break != 0){ pS[I].cpu_die_rq = 1; }
	// if(pS[I].RG_Mode.Interrupt_Enable != 0){ pS[I].cpu_die_rq = 1; }
	if(pS[I].microtrace || pS[I].cpu_die_rq == 1){
	  logmsgf(LT_LAMBDA,10,"RG Mode = %o (",pS[I].RG_Mode.raw);
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control != 0){ logmsgf(LT_LAMBDA,10,"Aux_Stat_Count_Control:%o ",pS[I].RG_Mode.Aux_Stat_Count_Control); }
	  if(pS[I].RG_Mode.Aux_Stat_Count_Clock != 0){ logmsgf(LT_LAMBDA,10,"Aux_Stat_Count_Clock:%o ",pS[I].RG_Mode.Aux_Stat_Count_Clock); }
	  if(pS[I].RG_Mode.NUbus_ID != 0){ logmsgf(LT_LAMBDA,10,"NUbus_ID:%o ",pS[I].RG_Mode.NUbus_ID); }
	  if(pS[I].RG_Mode.Need_Macro_Inst_Fetch != 0){ logmsgf(LT_LAMBDA,10,"Need_Macro_Inst_Fetch:%o ",pS[I].RG_Mode.Need_Macro_Inst_Fetch); }
	  if(pS[I].RG_Mode.Main_Stat_Count_Control != 0){ logmsgf(LT_LAMBDA,10,"Main_Stat_Count_Control:%o ",pS[I].RG_Mode.Main_Stat_Count_Control); }
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control_2 != 0){ logmsgf(LT_LAMBDA,10,"Aux_Stat_Count_Control_2:%o ",pS[I].RG_Mode.Aux_Stat_Count_Control_2); }
	  if(pS[I].RG_Mode.Main_Stat_Clock_Control != 0){ logmsgf(LT_LAMBDA,10,"Main_Stat_Clock_Control:%o ",pS[I].RG_Mode.Main_Stat_Clock_Control); }
	  if(pS[I].RG_Mode.Enable_Misc_MID != 0){ logmsgf(LT_LAMBDA,10,"Enable_Misc_MID:%o ",pS[I].RG_Mode.Enable_Misc_MID); }
	  if(pS[I].RG_Mode.Sequence_Break != 0){ logmsgf(LT_LAMBDA,10,"Sequence_Break:%o ",pS[I].RG_Mode.Sequence_Break); }
	  if(pS[I].RG_Mode.Interrupt_Enable != 0){ logmsgf(LT_LAMBDA,10,"Interrupt_Enable:%o ",pS[I].RG_Mode.Interrupt_Enable); }
	  if(pS[I].RG_Mode.MID_Hi_Adr != 0){ logmsgf(LT_LAMBDA,10,"MID_Hi_Adr:%o ",pS[I].RG_Mode.MID_Hi_Adr); }
	  if(pS[I].RG_Mode.VA_Mode != 0){ logmsgf(LT_LAMBDA,10,"VA_Mode:%o ",pS[I].RG_Mode.VA_Mode); }
	  if(pS[I].RG_Mode.Single_Step_Macro_Inst != 0){ logmsgf(LT_LAMBDA,10,"Single_Step_Macro_Inst:%o ",pS[I].RG_Mode.Single_Step_Macro_Inst); }
	  logmsgf(LT_LAMBDA,10,")\n");
	}
	break;
      case 044: // LAM-FUNC-DEST-STAT-COUNTER
	pS[I].stat_counter_main = pS[I].Obus;
	break;
      case 053: // LAM-FUNC-DEST-PDL-BUFFER-INDEX
	if((pS[I].DP_Mode.PDL_Addr_Hi) == 0){
	  pS[I].pdl_index_reg = pS[I].Obus&0xFFF;
	}else{
	  pS[I].pdl_index_reg = pS[I].Obus&0x7FF;
	}
        if(pS[I].microtrace || pS[I].macrotrace){
	  logmsgf(LT_LAMBDA,10,"MI: PDL-BUFFER-INDEX: Addr Hi = 0x%X, INDEX = %o, DATA = %o\n",
		 pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_index_reg,pS[I].Obus);
	}
	break;
      case 061: // LAM-FUNC-DEST-VMA-START-READ-FORCE
	// Load VMA from pS[I].Obus and initiate a read with force.
	pS[I].VMAregister.raw = pS[I].Obus; // Load VMA
	VM_resolve_address(I,VM_READ,1);
	if(pS[I].Page_Fault == 0 && pS[I].ConReg.Enable_NU_Master == 1){
	  // Do it
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control == 01){
	    pS[I].stat_counter_aux++;
	  }
	  if(pS[I].RG_Mode.Main_Stat_Count_Control == 01){
	    pS[I].stat_counter_main++;
	  }
	  lcbus_io_request(pS[I].vm_byte_mode|VM_READ,I,pS[I].vm_phys_addr.raw,0);
	}
	break;
      case 064: // LAM-FUNC-DEST-L2-MAP-PHYSICAL-PAGE
        // Requires SLOW-DEST
        // Addressed by MD
        // 400 per page
	pS[I].vm_lv2_index.raw = 0;
        pS[I].vm_lv2_index.VPage_Offset = pS[I].MDregister.VM.VPage_Offset;
        pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block;
        pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].raw = pS[I].Obus;
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"VM: LV2 ADR ENT 0x%X = 0x%X\n",pS[I].vm_lv2_index.raw,pS[I].Obus);
	}
        break;
      case 072: // LAM-FUNC-DEST-MD-START-WRITE-FORCE
	pS[I].MDregister.raw = pS[I].Obus;
	VM_resolve_address(I,VM_WRITE,1);
	if(pS[I].Page_Fault == 0 && pS[I].ConReg.Enable_NU_Master == 1){
	  // Do it
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control == 01){
	    pS[I].stat_counter_aux++;
	  }
	  if(pS[I].RG_Mode.Main_Stat_Count_Control == 01){
	    pS[I].stat_counter_main++;
	  }
	  lcbus_io_request(pS[I].vm_byte_mode|VM_WRITE,I,pS[I].vm_phys_addr.raw,pS[I].MDregister.raw);
	}
	break;
      default:
	logmsgf(LT_LAMBDA,0,"Unknown F-Dest %o\n",pS[I].Iregister.Destination.F.Dest);
	pS[I].cpu_die_rq = 1;
      }
    }
  }
}

// NUbus Slave
void lambda_nubus_slave(int I){
  switch(NUbus_Address.Addr){
    // SPY interface
  case 0x000 ... 0x3FF:
    {
      uint8_t spy_reg = ((NUbus_Address.Addr&0x3FF)>>2);
      switch(spy_reg){
      case 000: // CSMRAM
	if(NUbus_Request == VM_WRITE){
          pS[I].CSMRAM[pS[I].CSM_Adr.Addr].raw = NUbus_Data.word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: CSM Write, addr 0x%X, data 0x%X\n",pS[I].CSM_Adr.Addr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].CSMRAM[pS[I].CSM_Adr.Addr].raw;
          logmsgf(LT_LAMBDA,10,"RG: SPY: CSM Read, addr 0x%X, data 0x%X\n",pS[I].CSM_Adr.Addr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
	if(NUbus_Request == VM_BYTE_WRITE){
          pS[I].CSMRAM[pS[I].CSM_Adr.Addr].byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: CSM Byte Write, addr 0x%X, data 0x%X\n",pS[I].CSM_Adr.Addr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].CSMRAM[pS[I].CSM_Adr.Addr].byte[NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: CSM Byte Read, addr 0x%X, data 0x%X\n",pS[I].CSM_Adr.Addr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge = 1;
          return;
        }
	break;

      case 001: // CSM.ADR
	// special address register for debug read/write of (It says TRAM, but meant CSMRAM?), low 12 bits, read and write
	// & CACHED.PHY.ADR (currently cached physical address, high 18 bits, read only)
	// & MEMORY CYCLE STATUS (memory.cycle.pending, bit 31, and memory.cycle.active, bit 30; read only)
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
          uint32_t Word = (NUbus_Data.word&0x0FFF);
          if(NUbus_Request == VM_BYTE_WRITE){
            uint32_t Mask = 0xFF;
            uint32_t Data = NUbus_Data.byte[NUbus_Address.Byte];
            Word = pS[I].CSM_Adr.raw;
            Mask <<= (8*NUbus_Address.Byte);
            Data <<= (8*NUbus_Address.Byte);
            Word &= ~Mask;
            Word |= Data;
          }
          pS[I].CSM_Adr.Addr = Word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: CSM.ADR Write, data 0x%X\n",pS[I].CSM_Adr.Addr);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].CSM_Adr.raw;
          logmsgf(LT_LAMBDA,10,"RG: SPY: CSM.ADR Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          uint32_t Word = pS[I].CSM_Adr.raw;
          Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          logmsgf(LT_LAMBDA,10,"RG: SPY: CSM.ADR Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 002: // CSM.REG
	// Output register of CSM. Read-only.
	if(NUbus_Request == VM_WRITE){
          logmsgf(LT_LAMBDA,10,"RG: SPY: CSM.REG Write, data discarded\n");
          NUbus_acknowledge = 1;
          return;
        }
	if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].CSM_Output;
          logmsgf(LT_LAMBDA,10,"RG: SPY: CSM.REG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        break;

      case 004: // HIGH IREG
	if(NUbus_Request == VM_WRITE){
          pS[I].Iregister.word[1] = NUbus_Data.word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: HI IREG Write, data 0x%X\n",NUbus_Data.word);
	  logmsgf(LT_LAMBDA,10,"DISASSEMBLY OF WRITTEN UI:\n");
	  disassemble_IR(I);
          NUbus_acknowledge = 1;
	  pS[I].spy_wrote_ireg = true;
	  // Writing IREG explicitcly clocks the A and M busses without a SM clock!
	  // See the Q-to-SPY tests of the DP diagnostic for proof
	  handle_source(I,1);
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          pS[I].Iregister.byte[4+NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: HI IREG Byte Write, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
	  pS[I].spy_wrote_ireg = true;
	  if(NUbus_Address.Byte == 3){
	    logmsgf(LT_LAMBDA,10,"DISASSEMBLY OF WRITTEN UI:\n");
	    disassemble_IR(I);
	    // Writing IREG explicitcly clocks the A and M busses without a SM clock!
	    // See the Q-to-SPY tests of the DP diagnostic for proof
	    handle_source(I,1);
	  }
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].Iregister.word[1];
          logmsgf(LT_LAMBDA,10,"RG: SPY: HI IREG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].Iregister.byte[4+NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: HI IREG Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 005: // LO IREG
	if(NUbus_Request == VM_WRITE){
          pS[I].Iregister.word[0] = NUbus_Data.word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: LO IREG Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
	  pS[I].spy_wrote_ireg = true;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          pS[I].Iregister.byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: LO IREG Byte Write, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
	  pS[I].spy_wrote_ireg = true;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].Iregister.word[0];
          logmsgf(LT_LAMBDA,10,"RG: SPY: LO IREG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].Iregister.byte[NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: HI IREG Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 006: // HI CRAM
        if(NUbus_Request == VM_WRITE){
	  int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
	  int paddr = pS[I].CRAM_map[addr>>4] & 03777;
	  paddr <<= 4;
	  paddr |= (addr&0xF);
          pS[I].WCS[paddr].word[1] = NUbus_Data.word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: HI CRAM Write, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          NUbus_Data.word = pS[I].WCS[paddr].word[1];
          logmsgf(LT_LAMBDA,10,"RG: SPY: HI CRAM Read, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          pS[I].WCS[paddr].byte[4+NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: HI CRAM Byte Write, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].WCS[paddr].byte[4+NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: HI CRAM Byte Read, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 007: // LOW CRAM
        if(NUbus_Request == VM_WRITE){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          pS[I].WCS[paddr].word[0] = NUbus_Data.word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: LO CRAM Write, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          NUbus_Data.word = pS[I].WCS[paddr].word[0];
          logmsgf(LT_LAMBDA,10,"RG: SPY: LO CRAM Read, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          pS[I].WCS[paddr].byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: LO CRAM Byte Write, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].WCS[paddr].byte[NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: LO CRAM Byte Read, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        break;

      case 010: // TRAM.ADR
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  uint32_t Word = (NUbus_Data.word&0x0FFF);
	  if(NUbus_Request == VM_BYTE_WRITE){
	    uint32_t Mask = 0xFF;
	    uint32_t Data = NUbus_Data.byte[NUbus_Address.Byte];
	    Word = (pS[I].TRAM_Adr&0x0FFF);
	    Mask <<= (8*NUbus_Address.Byte);
	    Data <<= (8*NUbus_Address.Byte);
	    Word &= ~Mask;
	    Word |= Data;
	  }
	  pS[I].TRAM_Adr = Word;
	  logmsgf(LT_LAMBDA,10,"RG: SPY: TRAM.ADR Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = (pS[I].TRAM_Adr&0x0FFF);
          logmsgf(LT_LAMBDA,10,"RG: SPY: TRAM.ADR Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
	  uint32_t Word = (pS[I].TRAM_Adr&0x0FFF);
          Word >>= (8*NUbus_Address.Byte);
	  NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          logmsgf(LT_LAMBDA,10,"RG: SPY: TRAM.ADR Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 011: // TRAM
        if(NUbus_Request == VM_WRITE){
          pS[I].TRAM[pS[I].TRAM_Adr].word = NUbus_Data.word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: TRAM Write, addr 0x%X, data 0x%X\n",pS[I].TRAM_Adr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          pS[I].TRAM[pS[I].TRAM_Adr].byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
	  logmsgf(LT_LAMBDA,10,"RG: SPY: TRAM Byte Write, addr 0x%X, data 0x%X\n",pS[I].TRAM_Adr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].TRAM[pS[I].TRAM_Adr].word;
	  logmsgf(LT_LAMBDA,10,"RG: SPY: TRAM Read, addr 0x%X, data 0x%X\n",pS[I].TRAM_Adr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].TRAM[pS[I].TRAM_Adr].byte[NUbus_Address.Byte];
	  logmsgf(LT_LAMBDA,10,"RG: SPY: TRAM Byte Read, addr 0x%X, data 0x%X\n",pS[I].TRAM_Adr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 012: // HRAM
        if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  uint32_t Word = NUbus_Data.word;
          if(NUbus_Request == VM_BYTE_WRITE){
            uint32_t Mask = 0xFF;
            uint32_t Data = NUbus_Data.byte[NUbus_Address.Byte];
            Word = pS[I].History_RAM[pS[I].History_Pointer&0xFFF];
            Mask <<= (8*NUbus_Address.Byte);
            Data <<= (8*NUbus_Address.Byte);
            Word &= ~Mask;
            Word |= Data;
          }
          pS[I].History_RAM[pS[I].History_Pointer&0xFFF] = Word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: HRAM Write, addr 0x%X, data 0x%X\n",pS[I].History_Pointer&0xFFF,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  uint32_t Word = pS[I].History_RAM[pS[I].History_Pointer&0xFFF];
	  if(NUbus_Request == VM_READ){
	    NUbus_Data.word = Word;
	  }else{
	    NUbus_Data.byte[NUbus_Address.Byte] = ((Word>>(8*NUbus_Address.Byte))&0xFF);
	  }
	  logmsgf(LT_LAMBDA,10,"RG: SPY: HRAM Read, addr 0x%X, data 0x%X\n",pS[I].History_Pointer&0xFFF,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
	break;

      case 013: // HPTR
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  uint32_t Word = (NUbus_Data.word&0x0FFF);
          if(NUbus_Request == VM_BYTE_WRITE){
            uint32_t Mask = 0xFF;
            uint32_t Data = NUbus_Data.byte[NUbus_Address.Byte];
            Word = (pS[I].History_Pointer&0x0FFF);
            Mask <<= (8*NUbus_Address.Byte);
            Data <<= (8*NUbus_Address.Byte);
            Word &= ~Mask;
            Word |= Data;
          }
	  pS[I].History_Pointer = Word;
	  pS[I].Next_History_Pointer = Word;
	  logmsgf(LT_LAMBDA,10,"RG: SPY: HPTR Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = (pS[I].History_Pointer&0x0FFF);
          logmsgf(LT_LAMBDA,10,"RG: SPY: HPTR Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
	if(NUbus_Request == VM_BYTE_READ){
	  uint32_t Word = (pS[I].History_Pointer&0x0FFF);
          Word >>= (8*NUbus_Address.Byte);
	  NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
	  logmsgf(LT_LAMBDA,10,"RG: SPY: HPTR Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
	  NUbus_acknowledge=1;
	  return;
	}
	break;

      case 014: // PC (read-only)
	if(NUbus_Request == VM_WRITE){
          logmsgf(LT_LAMBDA,10,"RG: SPY: uPC Write, data discarded");
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
	  // If we have a pending PC, take that instead
          logmsgf(LT_LAMBDA,10,"RG: SPY: uPC Read, ");
	  if(pS[I].loc_ctr_nxt != -1){
	    logmsgf(LT_LAMBDA,10,"Nxt");
	    NUbus_Data.word = pS[I].loc_ctr_nxt;
	  }else{
	    logmsgf(LT_LAMBDA,10,"Reg");
	    NUbus_Data.word = pS[I].loc_ctr_reg.raw;
	  }
	  logmsgf(LT_LAMBDA,10," = 0x%X (0%o)\n",NUbus_Data.word,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
	  uint32_t Word = 0;
          logmsgf(LT_LAMBDA,10,"RG: SPY: uPC Byte Read, ");
          if(pS[I].loc_ctr_nxt != -1){
            logmsgf(LT_LAMBDA,10,"Nxt");
            Word = pS[I].loc_ctr_nxt;
          }else{
	    logmsgf(LT_LAMBDA,10,"Reg");
	    Word = pS[I].loc_ctr_reg.raw;
          }
	  logmsgf(LT_LAMBDA,10," = 0%o",Word);
	  Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          logmsgf(LT_LAMBDA,10,", data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        break;

      case 015: // TREG (Output register of TRAM, read-only)
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].TREG.word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: TREG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].TREG.byte[NUbus_Address.Byte];
          logmsgf(LT_LAMBDA,10,"RG: SPY: TREG Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 016: // MFO ENABLE (reading MFO bus)
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].MFObus;
          logmsgf(LT_LAMBDA,10,"RG: SPY: MFO BUS Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          uint32_t Word = pS[I].MFObus;
          Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          logmsgf(LT_LAMBDA,10,"RG: SPY: MFO BUS Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 017: // Spy Register
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
          uint32_t Word = NUbus_Data.word;
          if(NUbus_Request == VM_BYTE_WRITE){
            uint32_t Mask = 0xFF;
            uint32_t Data = NUbus_Data.byte[NUbus_Address.Byte];
            Word = pS[I].SPY_Register;
            Mask <<= (8*NUbus_Address.Byte);
            Data <<= (8*NUbus_Address.Byte);
            Word &= ~Mask;
            Word |= Data;
          }
	  pS[I].SPY_Register = Word;
	  logmsgf(LT_LAMBDA,10,"RG: SPY: SPY REG Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].SPY_Register;
          logmsgf(LT_LAMBDA,10,"RG: SPY: SPY REG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          uint32_t Word = pS[I].SPY_Register;
          Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          logmsgf(LT_LAMBDA,10,"RG: SPY: SPY REG Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 020: // PMR
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  uint32_t Word = (NUbus_Data.word&0xFFFFFF00);
	  Processor_Mode_Reg oldPMR = pS[I].PMR;
	  if(NUbus_Request == VM_BYTE_WRITE){
	    // Make Word
	    uint32_t Mask = 0xFF, NewWord = NUbus_Data.byte[NUbus_Address.Byte];
	    Word = ((pS[I].PMR.raw&0x00FFFFFF)<<8);
	    Mask <<= 8*NUbus_Address.Byte;
	    NewWord <<= 8*NUbus_Address.Byte;
	    Word &= ~Mask;
	    Word |= NewWord;
	  }
	  pS[I].PMR.raw = Word;
	  pS[I].PMR.raw >>= 8;
	  logmsgf(LT_LAMBDA,10,"RG: SPY: PMR Write, data 0x%X\n",Word);
	  NUbus_acknowledge = 1;
	  // HANDLE BITS
	  if(pS[I].PMR.Clear_NOP != 0){
	    logmsgf(LT_LAMBDA,10,"RG: PMR: Clear NOP\n");
	    pS[I].ConReg.nop = 0;
	  }
	  if(pS[I].PMR.Reset_Interrupt_Counter != 0){
	    int x=0;
	    logmsgf(LT_LAMBDA,10,"RG: PMR: Reset Interrupt Counter\n");
	    pS[I].InterruptPending = 0;
	    while(x<0x100){
	      pS[I].InterruptStatus[x] = 0;
	      x++;
	    }
	    pS[I].InterruptVector = 0;
	  }
	  if(pS[I].PMR.Force_T_Hold != 0){
	    if(oldPMR.Force_T_Hold == 0){
	      logmsgf(LT_LAMBDA,10,"RG: PMR: FORCE-T-HOLD SET\n");
	      pS[I].exec_hold = true;
	      pS[I].ConReg.t_hold_l = 0; // Holding
	    }
	  }else{
	    if(oldPMR.Force_T_Hold != 0){
	      logmsgf(LT_LAMBDA,10,"RG: PMR: FORCE-T-HOLD CLEARED\n");
	      pS[I].exec_hold = false;
	      pS[I].ConReg.t_hold_l = 1; // Not Holding
	    }
	  }
	  // If CONREG Enable_SM_Clock is clear, the SM clock is controlled by PMR Debug_Clock
	  if(pS[I].ConReg.Enable_SM_Clock == 0){
	    // FIXME: HAVE THE TARGET PROESSOR THREAD EXECUTE THIS
	    // OR WE WILL FAIL WHEN ACCESSING THE BUS (LAM DOES THIS)
	    pS[I].SM_Old_PMR = &oldPMR;
	    pS[I].SM_Clock_Pulse++;
	    // logmsgf(LT_LAMBDA,0,"LAMBDA %d: SM CLOCK PULSE REQUEST\n",I);
	    // sm_clock_pulse(I,pS[I].PMR.Debug_Clock,&oldPMR);
	    while(pS[I].SM_Clock_Pulse > 0){
	      struct timespec sleep_time;
	      sleep_time.tv_sec = 0;
	      sleep_time.tv_nsec = 200;
	      nanosleep(&sleep_time,NULL); // Wait for it to complete
	    }
	  }
	  // UInst Advance
	  if(pS[I].PMR.Advance_UInst_Request != 0 && oldPMR.Advance_UInst_Request == 0){
	    // a transition from 0 to 1 causes a single uinst step, or tries to restart the lambda if it is in a halted state
	    logmsgf(LT_LAMBDA,10,"RG %d: PMR: ADVANCE UINST REQUEST\n",I);
	    // pS[I].microtrace = 1;
	    pS[I].cpu_die_rq = 0;
	  }
	  return;
	}
	if(NUbus_Request == VM_READ){
	  NUbus_Data.word = (pS[I].PMR.raw&0x00FFFFFF);
	  NUbus_Data.word <<= 8;
	  NUbus_Data.word |= 0xFF;
	  // Low 8 bits come from configuration PROM, but I don't know how to address it
	  // Newboot seems to indicate they should be all Fs
	  logmsgf(LT_LAMBDA,10,"RG: SPY: PMR Read, data 0x%X\n",NUbus_Data.word);
	  NUbus_acknowledge = 1;
	  return;
	}
	if(NUbus_Request == VM_BYTE_READ){
	  uint32_t Word = (((pS[I].PMR.raw&0x00FFFFFF)<<8)|0xFF);
	  Word >>= (8*NUbus_Address.Byte);
	  NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
	  logmsgf(LT_LAMBDA,10,"RG: SPY: PMR Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
	  NUbus_acknowledge=1;
	  return;
	}
	break;

      case 021: // PARITY VECTOR
        if(NUbus_Request == VM_WRITE){
          pS[I].Parity_Vector = NUbus_Data.word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: Parity Vector Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].Parity_Vector;
          logmsgf(LT_LAMBDA,10,"RG: SPY: Parity Vector Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
	break;

      case 022: // CRAM ADR MAP
	// Addressed by PC
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
          int map_addr = ((pS[I].loc_ctr_reg.raw&0xFFFF)>>4);
	  uint32_t Word = NUbus_Data.word;
          if(NUbus_Request == VM_BYTE_WRITE){
            uint32_t Mask = 0xFF;
            uint32_t Data = NUbus_Data.byte[NUbus_Address.Byte];
            Word = pS[I].CRAM_map[map_addr];
            Mask <<= (8*NUbus_Address.Byte);
            Data <<= (8*NUbus_Address.Byte);
            Word &= ~Mask;
            Word |= Data;
          }
          pS[I].CRAM_map[map_addr] = Word;
          logmsgf(LT_LAMBDA,10,"RG: SPY: CRAM ADR MAP Write, addr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          int map_addr = ((pS[I].loc_ctr_reg.raw&0xFFFF)>>4);
          NUbus_Data.word = pS[I].CRAM_map[map_addr];
          logmsgf(LT_LAMBDA,10,"RG: SPY: CRAM ADR MAP Read, addr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          int map_addr = ((pS[I].loc_ctr_reg.raw&0xFFFF)>>4);
          uint32_t Word = pS[I].CRAM_map[map_addr];
          Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          logmsgf(LT_LAMBDA,10,"RG: SPY: CRAM ADR MAP Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        break;

      }
      logmsgf(LT_LAMBDA,0,"RG: Unimplemented SPY Request %o Addr 0x%X (Reg 0%o) w/ data 0x%X (0%o)\n",
	     NUbus_Request,NUbus_Address.raw,spy_reg,NUbus_Data.word,NUbus_Data.word);
      pS[I].cpu_die_rq = 1;
    }
    break;

  case 0x400 ... 0x7FF:
    if(NUbus_Request == VM_WRITE){
      // Interrupt!
      uint8_t Vector = ((NUbus_Address.Addr>>2)&0xFF);
      if(NUbus_trace == 1){
	logmsgf(LT_LAMBDA,10,"RG: INTERRUPT RQ: Vector %o",Vector);
      }
      // Light vector bit if enabled
      if(pS[I].RG_Mode.Interrupt_Enable != 0){
	if(pS[I].InterruptStatus[Vector] != 1){
	  pS[I].InterruptStatus[Vector] = 1;
	  pS[I].InterruptPending++;
	}
	if(NUbus_trace == 1){
	  logmsgf(LT_LAMBDA,10," ACCEPTED");
	}
      }
      if(NUbus_trace == 1){
	logmsgf(LT_LAMBDA,10,"\n");
      }
      // We got it
      NUbus_acknowledge = 1;
      // Die for now
      // pS[I].cpu_die_rq = 1;
      // All done
      return;
    }
    break;

    // Configuration Register
  case 0xFFF7FC:
    if(NUbus_Request == VM_BYTE_WRITE || NUbus_Request == VM_WRITE){
      pS[I].ConReg.writable_bits = (NUbus_Data.byte[0]&0x0F);
      logmsgf(LT_LAMBDA,10,"RG %d: CON REG write, data 0x%X, result 0x%X\n",
	     I,NUbus_Data.byte[0],pS[I].ConReg.word);
      // HANDLE THOSE BITS
      if(pS[I].ConReg.Init == 1){
	logmsgf(LT_LAMBDA,10,"CONREG: INIT\n");
	lambda_initialize(I,0);
	pS[I].ConReg.Init = 0; // This makes newboot happy
      }
      if(pS[I].ConReg.Enable_SM_Clock == 1){
	logmsgf(LT_LAMBDA,10,"CONREG %d: ENABLE SM CLOCK\n",I);
	if(pS[I].cpu_die_rq != 0){
	  if(pS[I].PMR.Allow_UInst_Clocks != 0){
	    extern int cp_state[2];
	    logmsgf(LT_LAMBDA,10,"CONREG %d: LAMBDA ENABLED\n",I);
	    pS[I].cpu_die_rq = 0;
	    // Update state
	    if(pS[I].loc_ctr_reg.raw == 036000 || pS[I].loc_ctr_reg.raw == 036001){
	      // PROM START
	      cp_state[I] = 1;
	      logmsgf(LT_LAMBDA,1,"LAMBDA %d COLD BOOT START\n",I);
	    }else{
	      if(pS[I].loc_ctr_reg.raw == 000002){
		// LISP START
		cp_state[I] = 2;
		logmsgf(LT_LAMBDA,1,"LAMBDA %d LISP START\n",I);
		// Patch microcode bug
		logmsgf(LT_LAMBDA,10,"Patching loaded microcode...\r\n");
		pS[I].WCS[016047].ASource = 01114;    // Fix to XGCD1+2 - Add1 to Rotate Field Sub1 from Length
		// At this point, we should capture Lisp's starting state
		dump_lisp_start_state(I);
	      }else{
		logmsgf(LT_LAMBDA,10,"LAMBDA %d UNKNOWN START POINT %o\n",I,pS[I].loc_ctr_reg.raw);
	      }
	    }
	  }else{
	    logmsgf(LT_LAMBDA,10,"CONREG: Lambda not enabled, PMR.Allow_UInst_Clocks is off\n");
	  }
	}
      }else{
	logmsgf(LT_LAMBDA,10,"CONREG %d: DISABLE SM CLOCK\n",I);
	pS[I].cpu_die_rq = 1; // Ensure stopped
	pS[I].ConReg.LED = 1; // Not inverted - Does this force LED?
	// pS[I].ConReg.uinst_clock_l = 1; // HACK HACK
      }
      NUbus_acknowledge=1;
      return;
    }
    if(NUbus_Request == VM_READ){
      NUbus_Data.word = pS[I].ConReg.word; // 0x80; // UINST-CLOCK-L-BIT
      logmsgf(LT_LAMBDA,10,"RG %d: CON REG read, data 0x%X\n",I,NUbus_Data.word);
      NUbus_acknowledge=1;
      return;
    }
  case 0xFFF7FD:
  case 0xFFF7FE:
  case 0xFFF7FF:
    if(NUbus_Request == VM_BYTE_READ){
      NUbus_Data.byte[NUbus_Address.Byte] = pS[I].ConReg.byte[NUbus_Address.Byte];
      logmsgf(LT_LAMBDA,10,"RG %d: CON REG byte read, data 0x%X\n",I,NUbus_Data.byte[NUbus_Address.Byte]);
      NUbus_acknowledge=1;
      return;
    }
    break;

    // Configuration PROM
  case 0xfff800 ... 0xffff63:
    if((NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ)
       && NUbus_Address.Byte == 0){
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

  // Model # PROM
  case 0xFFFF64 ... 0xFFFFFF:
    if((NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ)
       && NUbus_Address.Byte == 0){
      uint8_t prom_addr = (NUbus_Address.Addr-0xFFFF64)/4;
      if(prom_addr <= 0x12){
        NUbus_Data.word = prom_modelno_string[prom_addr];
      }else{
        NUbus_Data.word = 0;
      }
      NUbus_acknowledge=1;
      return;
    }
    break;
  }

  // Otherwise die
  logmsgf(LT_LAMBDA,0,"RG: Unimplemented address: 0x%X\n",NUbus_Address.Addr);
  pS[I].cpu_die_rq = 1;
}

// State Machine Clock Pulse
void sm_clock_pulse(int I,int clock,volatile Processor_Mode_Reg *oldPMR){
  int old_clock = pS[I].SM_Clock;
  pS[I].SM_Clock = clock;
  if(clock == 0){
    if(old_clock != 0){
      logmsgf(LT_LAMBDA,10,"RG %d: PMR: SM CLOCK FALLING EDGE: TRAM_PC 0%o\n",I,pS[I].TRAM_PC);
      // If we are writing TRAM_PC, do so
      if(pS[I].PMR.Spy_Address_TRAM_L == 0 && oldPMR->Spy_Address_TRAM_L != 0){
	pS[I].TRAM_PC = (pS[I].TRAM_Adr&0xFFF);
	logmsgf(LT_LAMBDA,10,"TREG %d: PMR.Spy_Address_TRAM_L: %o\n",I,pS[I].TRAM_PC);
      }
      // Do UI pulse stuff
      if(pS[I].ConReg.uinst_clock_l == 1 && pS[I].uI_Clock_Pulse == true){
	pS[I].uI_Clock_Pulse = false;
	logmsgf(LT_LAMBDA,10,"TREG %d: Advance HPTR\n",I);
	pS[I].History_Pointer = pS[I].Next_History_Pointer;
	if(pS[I].wrote_uPC == false){
	  logmsgf(LT_LAMBDA,10,"TREG %d: UI CLOCK: ADVANCE uPC\n",I);
	  // ADVANCE LOCATION COUNTER
	  if(pS[I].loc_ctr_nxt != -1){
	    pS[I].loc_ctr_reg.raw = pS[I].loc_ctr_nxt; // Load next instruction
	    pS[I].loc_ctr_nxt = -1;          // Disarm
	  }else{
	    pS[I].loc_ctr_reg.raw++; // Otherwise get next instruction
	  }
	  pS[I].last_loc_ctr = pS[I].loc_ctr_reg.raw;    // Save This
	}else{
	  pS[I].wrote_uPC = false;
	}
	// LOAD NOP IF NOT FORCED?
	if(pS[I].NOP_Next != 0 && pS[I].PMR.Clear_NOP == 0){
	  pS[I].ConReg.nop = 1;
	}
	logmsgf(LT_LAMBDA,10,"TREG %d: New uPC: CNT %.4o REG %.4o NXT %.o\n",
		I,pS[I].loc_ctr_cnt,pS[I].loc_ctr_reg.raw,pS[I].loc_ctr_nxt);
      }
      // Update UI clock.
      if(pS[I].ConReg.uinst_clock_l != 1){
	pS[I].ConReg.uinst_clock_l = 1;
	logmsgf(LT_LAMBDA,10,"TREG %d: UI CLOCK FALLING EDGE\n",I);
	pS[I].uI_Clock_Pulse = true;
	// FETCH IREG?
	/*
	  if(pS[I].spy_wrote_ireg == false){
	  int addr = pS[I].loc_ctr_cnt&0xFFFF;
	  int paddr = pS[I].CRAM_map[addr>>4] & 03777;
	  paddr <<= 4;
	  paddr |= (addr&0xF);
	  if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"UI: MAP 0x%X -> 0x%X\n",pS[I].loc_ctr_cnt&0xFFFF,paddr);
	  }
	  pS[I].Iregister.raw = pS[I].WCS[paddr].raw;
	  }else{
	  pS[I].spy_wrote_ireg = false;
	  }
	*/
	// Process HRAM
	logmsgf(LT_LAMBDA,10,"TREG %d: Load History RAM\n",I);
	pS[I].History_RAM[pS[I].History_Pointer&0xFFF] = pS[I].loc_ctr_cnt;
	pS[I].Next_History_Pointer = pS[I].History_Pointer+1;
	if(pS[I].Next_History_Pointer > 0xFFF){ pS[I].Next_History_Pointer = 0; }
      }
      // If Force-Hold is on, make sure hold stays lit
      if(pS[I].PMR.Force_T_Hold != 0 && pS[I].ConReg.t_hold_l != 0){ pS[I].ConReg.t_hold_l = 0; }
      // Operate ALU and shifter, load O bus
      if(pS[I].TREG.A_clock_next == 1 && pS[I].TREG.M_clock_next == 1 &&
	 pS[I].TREG.A_WE_L == 1 && pS[I].TREG.M_WE_L == 1){
	// This is probably after the source read
	logmsgf(LT_LAMBDA,10,"TREG %d: Operating ALU/Shifter\n",I);
	// Switch opcode
	switch(pS[I].Iregister.Opcode){
	case 0: // ALU-OP
	  // Perform ALU operation
	  operate_alu(I);

	  // Operate O bus
	  handle_o_bus(I);

	  if(pS[I].Iregister.ALU.Misc > 0){ logmsgf(LT_LAMBDA,0,"ALU-MISC "); pS[I].cpu_die_rq = 1; }
	  if(pS[I].Iregister.ALU.Spare > 0){ logmsgf(LT_LAMBDA,0,"ALU-SPARE "); pS[I].cpu_die_rq = 1; }

	  // Load MFO from O-bus
	  pS[I].MFObus = pS[I].Obus;
	  logmsgf(LT_LAMBDA,10,"TREG %d: ALU RESULT = 0x%.8lX OBus = 0x%.8X\n",I,pS[I].ALU_Result,pS[I].Obus);
	  break;

	case 1: // BYTE-OP
	  // Operate shifter. Result goes to O bus.
	  operate_shifter(I);
	  // Load MFO from O-bus
	  pS[I].MFObus = pS[I].Obus;

	  if(pS[I].Iregister.Byte.Misc > 0){ logmsgf(LT_LAMBDA,0,"Misc "); pS[I].cpu_die_rq = 1; }
	  if(pS[I].Iregister.Byte.Spare > 0){ logmsgf(LT_LAMBDA,0,"BYTE-SPARE "); pS[I].cpu_die_rq = 1; }
	  break;

	case 2: // JUMP-OP
	  if(pS[I].Iregister.Jump.LC_Increment != 0){ logmsgf(LT_LAMBDA,0," LCINC"); pS[I].cpu_die_rq = 1; }
	  if(pS[I].Iregister.Jump.Spare != 0){ logmsgf(LT_LAMBDA,0," JUMP-SPARE"); pS[I].cpu_die_rq = 1; }
	  if(pS[I].Iregister.Jump.Spare2 != 0){ logmsgf(LT_LAMBDA,0," JUMP-SPARE2"); pS[I].cpu_die_rq = 1; }

	  // Handle condition
	  pS[I].test_true = false;
	  if(pS[I].Iregister.Jump.Test != 0){
	    // Operate ALU
	    alu_sub_stub(I,0); // Do M-A
	    alu_cleanup_result(I);

	    // Perform test
	    switch(pS[I].Iregister.Jump.Cond){

	    case 01: // LAM-JUMP-COND-M<A
	      if((0x80000000^pS[I].Mbus) < (0x80000000^pS[I].Abus)){
		pS[I].test_true = true;
	      }
	      break;

	    case 02: // LAM-JUMP-COND-M<=A
	      if((pS[I].Abus == 0 && pS[I].Mbus == 0x80000000) || ((pS[I].ALU_Result&0x80000000) != 0)){
		pS[I].test_true = true;
	      }
	      break;

	    case 03: // M != A
	      if(pS[I].ALU_Result != 0xFFFFFFFF){
		pS[I].test_true = true;
	      }
	      break;

	    case 04: // LAM-JUMP-COND-PAGE-FAULT (INVERTED!)
	      if(pS[I].Page_Fault == 0){
		pS[I].test_true = 1;
	      }
	      break;

	    case 05: // LAM-JUMP-COND-PAGE-FAULT-OR-INTERRUPT
	      if(pS[I].Page_Fault != 0){
		pS[I].test_true = 1;
	      }
	      // DETECT INTERRUPT
	      {
		int x=0;
		if(pS[I].InterruptPending != 0){
		  while(x<0x100){
		    if(pS[I].InterruptStatus[x] != 0){
		      // We have an interrupt!
		      // Stuff vector in interrupt-pointer and return true
		      pS[I].InterruptVector = x;
		      pS[I].test_true = 1;
		      break;
		    }
		    x++;
		  }
		}
	      }
	      break;

	    case 06: // LAM-JUMP-COND-PAGE-FAULT-OR-INTERRUPT-OR-SEQUENCE-BREAK
	      // SEQUENCE BREAK BIT IS INVERTED
	      if(pS[I].Page_Fault != 0 || pS[I].RG_Mode.Sequence_Break == 0){
		pS[I].test_true = 1;
	      }
	      // DETECT INTERRUPT
	      {
		int x=0;
		if(pS[I].InterruptPending != 0){
		  while(x<0x100){
		    if(pS[I].InterruptStatus[x] != 0){
		      // We have an interrupt!
		      // Stuff vector in interrupt-pointer and return true
		      pS[I].InterruptVector = x;
		      pS[I].test_true = 1;
		      break;
		    }
		    x++;
		  }
		}
	      }
	      break;

	    case 07: // LAM-JUMP-COND-UNC
	      pS[I].test_true = true;
	      break;

	    case 011: // LAM-JUMP-COND-DATA-TYPE-NOT-EQUAL
	      if((pS[I].Mbus&0x3E000000) != (pS[I].Abus&0x3E000000)){
		pS[I].test_true = true;
	      }
	      break;

	    default:
	      logmsgf(LT_LAMBDA,0,"Unknown jump cond %o\n",pS[I].Iregister.Jump.Cond);
	      pS[I].cpu_die_rq = 1;
	    }
	  }else{
	    // BIT-SET
	    pS[I].test_true = left_rotate(pS[I].Mbus,pS[I].Iregister.Jump.Cond)&0x01;
	  }
	  // If the invert bit is set, reverse the condition
	  if(pS[I].Iregister.Jump.Invert){
	    pS[I].test_true ^= 1;
	  }
	  break;
	case 3: // DISP-OP
	  // logmsgf(LT_LAMBDA,10,"TREG: DISPATCH-OP\n");
	  {
	    uint32_t Mask=0;
	    uint32_t dispatch_source=0;
	    int gc_volatilty_flag;
	    int oldspace_flag;
	    int disp_address=0;
	    // DispatchWord disp_word;

	    // Stop for investigation if...
	    // if(pS[I].Iregister.Dispatch.Constant > 0){ logmsgf(LT_LAMBDA,10,"Constant "); pS[I].cpu_die_rq = 1; }
	    // if(pS[I].Iregister.Dispatch.LPC > 0){ logmsgf(LT_LAMBDA,10,"LPC "); pS[I].cpu_die_rq = 1; }
	    // if(pS[I].Iregister.Dispatch.Write_VMA > 0){ logmsgf(LT_LAMBDA,10,"WriteVMA "); pS[I].cpu_die_rq = 1; }
	    // if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta > 0){ logmsgf(LT_LAMBDA,10,"EnableGCVMeta "); pS[I].cpu_die_rq = 1; }
	    // if(pS[I].Iregister.Dispatch.Enable_Oldspace_Meta > 0){ logmsgf(LT_LAMBDA,10,"EnableOldspaceMeta "); pS[I].cpu_die_rq = 1; }
	    if(pS[I].Iregister.Dispatch.Spare > 0){ logmsgf(LT_LAMBDA,0,"DISP-SPARE "); pS[I].cpu_die_rq = 1; }
	    // if(pS[I].popj_after_nxt != -1){ logmsgf(LT_LAMBDA,10,"DISPATCH with POPJ-AFTER-NEXT armed?\n"); pS[I].cpu_die_rq = 1; }

	    // Load VMA from M source
	    if(pS[I].Iregister.Dispatch.Write_VMA != 0){ pS[I].VMAregister.raw = pS[I].Mbus; }

	    // Lambda doesn't have dispatch-source, so I assume it's always R-bus
	    Mask = (1 << pS[I].Iregister.Dispatch.Len) - 1;
	    // Meta handling
	    if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta || pS[I].Iregister.Dispatch.Enable_Oldspace_Meta){
	      Mask = Mask & 0xfffffffe;
	    }
	    // Investigation stop
	    if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta && pS[I].Iregister.Dispatch.Enable_Oldspace_Meta){
	      logmsgf(LT_LAMBDA,0,"DISPATCH: Enable GCV and Oldspace meta simultaneously?\n");
	      pS[I].cpu_die_rq=10;
	    }
	    if(pS[I].microtrace){
	      logmsgf(LT_LAMBDA,10,"DISPATCH: GENERATED MASK 0x%X\n",Mask);
	    }

	    // Lambda does not have a rotate direction flag.
	    dispatch_source = left_rotate(pS[I].Mbus, pS[I].Iregister.Dispatch.Pos) & Mask;

	    if(pS[I].microtrace){
	      logmsgf(LT_LAMBDA,10,"DISPATCH: dispatch_source = 0x%X\n",dispatch_source);
	    }

	    // More meta bits
	    gc_volatilty_flag = 0;
	    if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta != 0){
	      int present_gcv = 0;
	      // IDENTIFYING GC VOLATILITY BIT
	      // Bit 4?
	      // "MAP2C-4 IS REALLY FROM THE GC-WRITE-LOGIC, NOT DIRECTLY THE L2MAP, THESE DAYS."
	      // GCV happens when a NEWER object is written into an OLDER memory.

	      // Do a map resolve for what's in MD
	      pS[I].vm_lv2_index.raw = 0;
	      pS[I].vm_lv2_index.VPage_Offset = pS[I].MDregister.VM.VPage_Offset;
	      pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block;
	      // Extract the present LV1 GC volatility
	      // present_gcv = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB;
	      present_gcv = (~pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB) & 03;
	      // Raven's CACHED GCV is (lv2_control & 0x1800) >> 11;
	      // GCV FLAG is (cached_gcv + 4 > (map_1_volatility ^ 7)) ? 0 : 1;

	      // Our CACHED GCV is the lv2 meta bits of the last reference.

	      // LV1 Meta Bits:
	      // 2 bits!
	      // "For hardware convenience, all three L1 map meta bits are stored in COMPLEMENTED form."
	      // S  H    What
	      // 0 (3) = Static Region (OLDEST)
	      // 1 (2) = Dynamic Region
	      // 2 (1) = Active Consing Region
	      // 3 (0) = Extra PDL Region (NEWEST)

	      // LV2 Meta Bits:
	      // 6 bits! But the bottom 2 bits are the same as the LV1 bits.
	      // 040 = Oldspace
	      // 020 = GCV-Flag
	      // 003 = LV1 GC Volatility

	      // So, if CACHED GCV is less than PRESENT GCV we wrote a newer item into an older page.
	      // The trap is taken if the flag is 0, so we want the inverse.
	      // What do I do if the MB validity bit isn't set?

	      // Meta bits are un-inverted now.
	      // gc_volatilty_flag 1 == don't trap
	      // So we want to set it 0 if we should trap.
	      // if(pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid != 0){ logmsgf(LT_LAMBDA,10,"GCV: LV1 invalid?\n"); pS[I].cpu_die_rq=1; } // Investigate
	      // if((pS[I].cached_gcv&03) <= present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){ gc_volatilty_flag = 1; }else{ pS[I].cpu_die_rq=0; } // ILLOP at PHTDEL6+11

	      // This comparison is correct for non-inverted meta.
	      // For the moment, LV1 invalidity forces a trap. This isn't conclusively proven correct, and may change.
	      // if((pS[I].cached_gcv&03) > present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){ gc_volatilty_flag = 1; }
	      if(pS[I].cached_gcv <= present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){
		gc_volatilty_flag = 1;
	      }

	      if(pS[I].microtrace){
		logmsgf(LT_LAMBDA,10,"DISPATCH: GCV: CACHED (LV2) GCV 0x%X\n",pS[I].cached_gcv&03);
		logmsgf(LT_LAMBDA,10,"DISPATCH: GCV: PRESENT (LV1) GCV 0x%X\n",present_gcv);
		logmsgf(LT_LAMBDA,10,"DISPATCH: GCV: LV1 ENT 0x%X = 0x%X (Meta 0x%X Validity %o)\n",
		       pS[I].MDregister.VM.VPage_Block,pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].raw,
		       pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB,
		       pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid);
		logmsgf(LT_LAMBDA,10,"DISPATCH: GCV: LV2 ENT 0x%X = 0x%X (Meta 0x%X)\n",
		       pS[I].vm_lv2_index.raw,pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].raw,
		       pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta);
	      }
	    }
	    oldspace_flag = 0;
	    if(pS[I].Iregister.Dispatch.Enable_Oldspace_Meta != 0){
	      // Do a map resolve for what's in MD
	      pS[I].vm_lv2_index.raw = 0;
	      pS[I].vm_lv2_index.VPage_Offset = pS[I].MDregister.VM.VPage_Offset;
	      pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block;
	      // Extract the oldspace bit (5?)
	      // oldspace_flag 0 means trap (oldspace)
	      // oldspace_flag 1 means don't trap (newspace)
	      // Reversing this causes infinite loop (GCV never tested), so this has to be right.
	      if((pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta&0x20) == 0x20){ oldspace_flag = 1; } // Not oldspace, don't trap
	      if(pS[I].microtrace){
		logmsgf(LT_LAMBDA,10,"DISPATCH: META: LV2 ENT 0x%X = 0x%X (Meta 0x%X)\n",
		       pS[I].vm_lv2_index.raw,pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].raw,
		       pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta);
	      }
	    }
	    // Set dispatch address
	    // Lambda uses the A source as the dispatch address!
	    // disp_address = (mir_mask & MInst_Disp_Address) | dispatch_source | gc_volatilty_flag | oldspace_flag;
	    disp_address = pS[I].Iregister.ASource|dispatch_source;
	    // Handle oldspace/GCV meta
	    if(pS[I].Iregister.Dispatch.Enable_Oldspace_Meta != 0 || pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta){
	      // I think this is how this is supposed to work
	      disp_address |= (oldspace_flag|gc_volatilty_flag);
	    }
	    // Load dispatch constant register
	    pS[I].disp_constant_reg = pS[I].Iregister.Dispatch.Constant;
	    // Lambda has no dispatch opcode field, so I assume it is always DISPATCH
	    pS[I].disp_word.raw = pS[I].Amemory[(disp_address)]; // A-source is already offset // Dmemory[disp_address];
	    if(pS[I].microtrace){
	      logmsgf(LT_LAMBDA,10,"DISPATCH: GENERATED ADDRESS 0x%X AND FETCHED WORD 0x%X\n",disp_address,pS[I].disp_word.raw);
	    }
	  }
	  break;
	}
      }
      // If we aren't holding...
      if(pS[I].ConReg.t_hold_l == 1){
	// Update UI clock (again)
	// The same SM clock which causes the source codeword to appear in TREG also raises UI clock.
	// if(pS[I].TREG.new_uinst != 0 && pS[I].PMR.Allow_UInst_Clocks != 0){
	/*
	  if(pS[I].ConReg.uinst_clock_l == 0){
	  pS[I].ConReg.uinst_clock_l = 1;
	  logmsgf(LT_LAMBDA,10,"TREG %d: UI CLOCK FALLING EDGE\n",I);
	  pS[I].uI_Clock_Pulse = true;
	  // Single step!
	  // pS[I].cpu_die_rq = 0;
	  }
	*/
	// Fetch TREG
	pS[I].TREG = pS[I].TRAM[pS[I].TRAM_PC];
	// Advance TRAM_PC
	switch(pS[I].TRAM_PC){
	default:
	  logmsgf(LT_LAMBDA,10,"TREG %d: UNKNOWN TRAM_PC 0%o\n",I,pS[I].TRAM_PC);
	  // Fall into
	case 0:     // ALU
	case 040:   // NOP'd ALU
	case 0400:  // JUMP
	case 0600:  // DISP
	case 01000: // Used by lambda-diag
	case 01004: // Used by lambda-diag
	case 01005: // Used by lambda-diag for force-source codeword
	case 03000: // DISPATCH TO EXECUTE PHASE
	case 03002: // Used After Dispatch uInst, goes to 3000
	case 04000: // HALT SETZ
	case 04400: // HALT JUMP
	case 07000: // "HALTED" LOOP
	  switch(pS[I].TREG.next_select){
	  case 0: // DISPATCH
	    pS[I].TRAM_PC = (pS[I].TREG.state&07);
	    if(pS[I].Iregister.Slow_Dest != 0){ pS[I].TRAM_PC |= 010; }
	    if(pS[I].ConReg.nop != 0){ pS[I].TRAM_PC |= 040; }
	    if(pS[I].Iregister.ILong != 0){ pS[I].TRAM_PC |= 0100; }
	    if((pS[I].Iregister.Opcode&01) != 0){ pS[I].TRAM_PC |= 0200; }
	    if((pS[I].Iregister.Opcode&02) != 0){ pS[I].TRAM_PC |= 0400; }
	    if(pS[I].Iregister.Halt != 0){ pS[I].TRAM_PC |= 04000; }
	    break;
	  case 1: // JUMP
	    pS[I].TRAM_PC = 03000+pS[I].TREG.state;
	    break;
	  default:
	    logmsgf(LT_LAMBDA,10,"TREG %d: Unknown NEXT-SELECT %o\n",I,pS[I].TREG.next_select);
	    break;
	  }
	  logmsgf(LT_LAMBDA,10,"TREG %d: NEXT TREG %o\n",I,pS[I].TRAM_PC);
	  break;
	}
      }else{
	logmsgf(LT_LAMBDA,10,"TREG %d: T-HOLD\n",I);
      }
      // If we did not write TRAM_PC, write it back to ADR
      if(pS[I].PMR.Spy_Address_TRAM_L == 1){
	pS[I].TRAM_Adr = pS[I].TRAM_PC;
      }
    }else{
      logmsgf(LT_LAMBDA,10,"RG %d: PMR: SM CLOCK LOW: TRAM_PC 0%o\n",I,pS[I].TRAM_PC);
      // Do UI pulse stuff
      if(pS[I].ConReg.uinst_clock_l == 1 && pS[I].uI_Clock_Pulse == true){
	pS[I].uI_Clock_Pulse = false;
	logmsgf(LT_LAMBDA,10,"TREG %d: Advance HPTR\n",I);
	pS[I].History_Pointer = pS[I].Next_History_Pointer;
	if(pS[I].wrote_uPC == false){
	  logmsgf(LT_LAMBDA,10,"TREG %d: UI CLOCK: ADVANCE uPC\n",I);
	  // ADVANCE LOCATION COUNTER
	  if(pS[I].loc_ctr_nxt != -1){
	    pS[I].loc_ctr_reg.raw = pS[I].loc_ctr_nxt; // Load next instruction
	    pS[I].loc_ctr_nxt = -1;          // Disarm
	  }else{
	    pS[I].loc_ctr_reg.raw++; // Otherwise get next instruction
	  }
	  pS[I].last_loc_ctr = pS[I].loc_ctr_reg.raw;    // Save This
	}else{
	  pS[I].wrote_uPC = false;
	}
	// LOAD NOP IF NOT FORCED?
	if(pS[I].NOP_Next != 0 && pS[I].PMR.Clear_NOP == 0){
	  pS[I].ConReg.nop = 1;
	}
	logmsgf(LT_LAMBDA,10,"TREG %d: New uPC: CNT %.4o REG %.4o NXT %.o\n",I,pS[I].loc_ctr_cnt,pS[I].loc_ctr_reg.raw,pS[I].loc_ctr_nxt);
      }
      // Update UI clock.
      if(pS[I].ConReg.uinst_clock_l != 1){
	pS[I].ConReg.uinst_clock_l = 1;
	logmsgf(LT_LAMBDA,10,"TREG %d: UI CLOCK FALLING EDGE\n",I);
	pS[I].uI_Clock_Pulse = true;
	// FETCH IREG?
	/*
	  if(pS[I].spy_wrote_ireg == false){
	  int addr = pS[I].loc_ctr_cnt&0xFFFF;
	  int paddr = pS[I].CRAM_map[addr>>4] & 03777;
	  paddr <<= 4;
	  paddr |= (addr&0xF);
	  if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"UI: MAP 0x%X -> 0x%X\n",pS[I].loc_ctr_cnt&0xFFFF,paddr);
	  }
	  pS[I].Iregister.raw = pS[I].WCS[paddr].raw;
	  }else{
	  pS[I].spy_wrote_ireg = false;
	  }
	*/
	// Process HRAM
	logmsgf(LT_LAMBDA,10,"TREG %d: Load History RAM\n",I);
	pS[I].History_RAM[pS[I].History_Pointer&0xFFF] = pS[I].loc_ctr_cnt;
	pS[I].Next_History_Pointer = pS[I].History_Pointer+1;
	if(pS[I].Next_History_Pointer > 0xFFF){ pS[I].Next_History_Pointer = 0; }
      }
      // Ensure stopped
      pS[I].cpu_die_rq = 1;
    }
  }else{
    if(old_clock == 0){
      // This is a SM CLOCK.
      // A normal instruction happens in two of these, one of which must be 03000.
      // 03000 is a source phase
      logmsgf(LT_LAMBDA,10,"RG %d: PMR: SM CLOCK RISING EDGE: TRAM_PC 0%o\n",I,pS[I].TRAM_PC);
      // Update UI clock.
      // The same SM clock which causes the source codeword to appear in TREG also raises UI clock.
      if(pS[I].TREG.new_uinst != 0 && pS[I].PMR.Allow_UInst_Clocks != 0){
	pS[I].ConReg.uinst_clock_l = 0;
	logmsgf(LT_LAMBDA,10,"TREG %d: UI CLOCK RISING EDGE\n",I);
	// Light halt req bit?
	if((pS[I].Iregister.Halt)){
	  if(pS[I].ConReg.halt_request != 1){
	    logmsgf(LT_LAMBDA,10,"TREG %d: HALT REQ SET\n",I);
	    pS[I].ConReg.halt_request = 1;
	  }
	}else{
	  if(pS[I].ConReg.halt_request != 0){
	    logmsgf(LT_LAMBDA,10,"TREG %d: HALT REQ CLEARED\n",I);
	    pS[I].ConReg.halt_request = 0;
	  }
	}
	// Save present uPC
	pS[I].loc_ctr_cnt = pS[I].loc_ctr_reg.raw;
	// TRIGGER DEST WRITES / JUMPS
	if(pS[I].ConReg.nop == 0){
	  switch(pS[I].Iregister.Opcode){
	  case 0: // ALU-OP
	    // Handle destination selector
	    handle_destination(I);
	    // Process Q register
	    if(pS[I].Iregister.ALU.QControl > 0){ logmsgf(LT_LAMBDA,10,"TREG: Q-CONTROL %o ALU-RESULT %lo\n",pS[I].Iregister.ALU.QControl,pS[I].ALU_Result); }
	    handle_q_register(I);
	    // Load MFO from O-bus
	    pS[I].MFObus = pS[I].Obus;
	    break;
	  case 1: // BYTE-OP
	    // Store result
	    handle_destination(I);
	    break;
	  case 2: // JUMP-OP
	    /* Handle RPN.
	       CODES ARE:
	       R P N  (RETURN, PUSH, INHIBIT)

	       0 0 0 = Branch-Xct-Next
	       0 0 1 = Branch
	       0 1 0 = Call-Xct-Next
	       0 1 1 = Call
	       1 0 0 = Return-Xct-Next
	       1 0 1 = Return
	       1 1 0 = NOP (JUMP2-XCT-NEXT) (UNDEFINED ON LAMBDA)
	       1 1 1 = SKIP (JUMP2) (UNDEFINED ON LAMBDA)
	    */

	    if(pS[I].test_true){
	      pS[I].wrote_uPC = true; // We are writing the uPC, so don't advance it.
	      // If we have a pending POPJ-AFTER-NEXT, cancel it
	      if(pS[I].popj_after_nxt != -1 && (pS[I].Iregister.Jump.RPN != 2 && pS[I].Iregister.Jump.RPN != 3 && pS[I].Iregister.Jump.RPN != 5)){
		logmsgf(LT_LAMBDA,0,"JUMP: Pending PJAN investigation stop: Not CALL or RETURN (Op %o)\n",pS[I].Iregister.Jump.RPN);
		pS[I].cpu_die_rq = 1;
	      }
	      // Handle operation
	      switch(pS[I].Iregister.Jump.RPN){
	      case 0: // Jump-Branch-Xct-Next
		// Jump, but DO NOT inhibit the next instruction!
		if(pS[I].loc_ctr_nxt != -1){
		  logmsgf(LT_LAMBDA,0,"JUMP: Pending JUMP-After-Next collision investigation stop\n");
		  pS[I].cpu_die_rq = 1;
		}
		pS[I].loc_ctr_nxt = pS[I].Iregister.Jump.Address;
		break;

	      case 1: // Jump-Branch
		if(pS[I].microtrace && pS[I].loc_ctr_reg.raw != (pS[I].loc_ctr_cnt + 1)){
		  logmsgf(LT_LAMBDA,10,"JUMP: Pending JUMP collision investigation marker\n");
		  // pS[I].cpu_die_rq = 1;
		}
		pS[I].loc_ctr_reg.raw = pS[I].Iregister.Jump.Address;
		pS[I].NOP_Next = 1;
		break;

	      case 2: // Jump-Call-Xct-Next
		// Call, but DO NOT inhibit the next instruction!
		pS[I].uPCS_ptr_reg++; pS[I].uPCS_ptr_reg &= 0xFF;
		pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw+1; // Pushes the address of the next instruction
		if(pS[I].microtrace){
		  char *location;
		  char symloc[100];
		  int offset;

		  logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

		  location = "";
		  offset = 0;
		  location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
		  if(location != 0){
		    if(offset != 0){
		      sprintf(symloc, "%s+%o", location, offset);
		    }else{
		      sprintf(symloc, "%s", location);
		    }
		    logmsgf(LT_LAMBDA,10,"%s",symloc);
		  }
		  logmsgf(LT_LAMBDA,10," (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
		}
		pS[I].loc_ctr_nxt = pS[I].Iregister.Jump.Address;
		if(pS[I].popj_after_nxt == 0){
		  // PJAN is armed. We want this call to return to my caller instead of here.
		  // So we'll pop this return address now.
		  pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
		  pS[I].popj_after_nxt = -1;
		}
		break;

	      case 3: // Jump-Call
		// PUSH ADDRESS
		pS[I].uPCS_ptr_reg++;  pS[I].uPCS_ptr_reg &= 0xFF;
		pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw;
		if(pS[I].microtrace){
		  char *location;
		  char symloc[100];
		  int offset;

		  logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

		  location = "";
		  offset = 0;
		  location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
		  if(location != 0){
		    if(offset != 0){
		      sprintf(symloc, "%s+%o", location, offset);
		    }else{
		      sprintf(symloc, "%s", location);
		    }
		    logmsgf(LT_LAMBDA,10,"%s",symloc);
		  }
		  logmsgf(LT_LAMBDA,10," (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
		}
		// Jump
		pS[I].loc_ctr_reg.raw = pS[I].Iregister.Jump.Address;
		pS[I].NOP_Next = 1;
		if(pS[I].popj_after_nxt == 0){
		  // PJAN is armed. We want this call to return to my caller instead of here.
		  // So we'll pop this return address now.
		  pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
		  pS[I].popj_after_nxt = -1;
		}
		break;

	      case 4: // Jump-Return-XCT-Next
		if(pS[I].popj_after_nxt != -1){
		  // PJAN is armed. Do not double!
		  logmsgf(LT_LAMBDA,0,"RETURN-XCT-NEXT with PJAN armed!\n");
		  pS[I].cpu_die_rq=1;
		}
		// POP ADDRESS
		pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
		pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
		break;

	      case 5: // Jump-Return
		if(pS[I].popj_after_nxt != -1){
		  // PJAN is armed. Do not double!
		  // logmsgf(LT_LAMBDA,10,"RETURN with PJAN armed!\n");
		  // pS[I].cpu_die_rq=1;
		  // All we are doing is making the return immediate, so just disable PJAN.
		  pS[I].popj_after_nxt = -1;
		}
		// POP ADDRESS
		pS[I].loc_ctr_reg.raw = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
		pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
		pS[I].NOP_Next = 1;
		break;

	      default:
		logmsgf(LT_LAMBDA,0,"Unknown jump RPN %o\n",pS[I].Iregister.Jump.RPN);
		pS[I].cpu_die_rq=1;
	      }
	    }
	    break;
	  case 3: // DISP-OP
	    // logmsgf(LT_LAMBDA,10,"TREG: DISPATCH-OP\n");
	    {
	      // Handle dispatch word
	      if(pS[I].microtrace){
		char *location;
		char symloc[100];
		int offset;

		logmsgf(LT_LAMBDA,10,"DISPATCH: OP %s DEST ",jump_op_str[pS[I].disp_word.Operation]);
		location = "";
		offset = 0;
		location = sym_find_last(1, pS[I].disp_word.PC, &offset);
		if(location != 0){
		  if(offset != 0){
		    sprintf(symloc, "%s+%o", location, offset);
		  }else{
		    sprintf(symloc, "%s", location);
		  }
		  logmsgf(LT_LAMBDA,10,"%s",symloc);
		}
		logmsgf(LT_LAMBDA,10," (%o)\n",pS[I].disp_word.PC);
	      }
	      // Handle operation of Start-Memory-Read
	      if(pS[I].disp_word.StartRead){
		if(pS[I].microtrace != 0){
		  logmsgf(LT_LAMBDA,10," START-MEMORY-READ");
		}
		// Load VMA from pS[I].Obus and initiate a read.
		pS[I].VMAregister.raw = pS[I].Mbus; // Load VMA
		VM_resolve_address(I,VM_READ,0);
		if(pS[I].Page_Fault == 0 && pS[I].ConReg.Enable_NU_Master == 1){
		  // Do it
		  if(pS[I].RG_Mode.Aux_Stat_Count_Control == 01){
		    pS[I].stat_counter_aux++;
		  }
		  if(pS[I].RG_Mode.Main_Stat_Count_Control == 01){
		    pS[I].stat_counter_main++;
		  }
		  lcbus_io_request(VM_READ,I,pS[I].vm_phys_addr.raw,0);
		}
	      }
	      // Handle operation
	      switch(pS[I].disp_word.Operation){
	      case 0: // Jump-Branch-Xct-Next
		// Jump, but DO NOT inhibit the next instruction!
		if(pS[I].loc_ctr_nxt != -1){
		  logmsgf(LT_LAMBDA,0,"DISPATCH: Pending JUMP-After-Next collision investigation stop\n");
		  pS[I].cpu_die_rq = 1;
		}
		pS[I].loc_ctr_nxt = pS[I].disp_word.PC;
		break;

	      case 1: // Jump-Branch
		if(pS[I].loc_ctr_reg.raw != (pS[I].loc_ctr_cnt + 1)){
		  logmsgf(LT_LAMBDA,0,"DISPATCH: Pending JUMP collision investigation stop\n");
		  pS[I].cpu_die_rq = 1;
		}
		pS[I].loc_ctr_reg.raw = pS[I].disp_word.PC;
		pS[I].NOP_Next = 1;
		break;

	      case 2: // Jump-Call-Xct-Next
		// Call, but DO NOT inhibit the next instruction!
		if(pS[I].popj_after_nxt != -1){
		  // PJAN is armed. Do not double!
		  pS[I].popj_after_nxt = -1;
		}
		pS[I].uPCS_ptr_reg++; pS[I].uPCS_ptr_reg &= 0xFF;
		// Raven does not handle this, but should we? (Should Raven?)
		if(pS[I].Iregister.Dispatch.LPC){
		  logmsgf(LT_LAMBDA,0,"DISPATCH: LPC set w/ call-xct-next?\n");
		  pS[I].cpu_die_rq = 1;
		}
		pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw+1; // Pushes the address of the next instruction
		if(pS[I].microtrace){
		  char *location;
		  char symloc[100];
		  int offset;

		  logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

		  location = "";
		  offset = 0;
		  location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
		  if(location != 0){
		    if(offset != 0){
		      sprintf(symloc, "%s+%o", location, offset);
		    }else{
		      sprintf(symloc, "%s", location);
		    }
		    logmsgf(LT_LAMBDA,10,"%s",symloc);
		  }
		  logmsgf(LT_LAMBDA,10," (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
		}
		pS[I].loc_ctr_nxt = pS[I].disp_word.PC;
		if(pS[I].popj_after_nxt == 0){
		  // PJAN is armed. We want this call to return to my caller instead of here.
		  // So we'll pop this return address now.
		  pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
		  pS[I].popj_after_nxt = -1;
		}
		break;

	      case 3: // Jump-Call
		// PUSH ADDRESS
		if(pS[I].popj_after_nxt != -1){
		  // PJAN is armed. Do not double!
		  //logmsgf(LT_LAMBDA,10,"RETURN-XCT-NEXT with PJAN armed!\n");
		  pS[I].popj_after_nxt = -1;
		}
		pS[I].uPCS_ptr_reg++;  pS[I].uPCS_ptr_reg &= 0xFF;
		if(pS[I].Iregister.Dispatch.LPC){
		  pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_cnt; // Stack-Own-Address
		}else{
		  pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw;
		}
		if(pS[I].microtrace){
		  char *location;
		  char symloc[100];
		  int offset;

		  logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

		  location = "";
		  offset = 0;
		  location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
		  if(location != 0){
		    if(offset != 0){
		      sprintf(symloc, "%s+%o", location, offset);
		    }else{
		      sprintf(symloc, "%s", location);
		    }
		    logmsgf(LT_LAMBDA,10,"%s",symloc);
		  }
		  logmsgf(LT_LAMBDA,10," (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
		}
		// Jump
		pS[I].loc_ctr_reg.raw = pS[I].disp_word.PC;
		pS[I].NOP_Next = 1;
		if(pS[I].popj_after_nxt == 0){
		  // PJAN is armed. We want this call to return to my caller instead of here.
		  // So we'll pop this return address now.
		  pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
		  pS[I].popj_after_nxt = -1;
		}
		break;

	      case 4: // Jump-Return-XCT-Next
		// POP ADDRESS
		pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
		pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
		break;

		// Used in d-swap-quantum-map-dispatch, should return
	      case 5: // Jump-Return
		// POP ADDRESS
		pS[I].loc_ctr_reg.raw = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
		pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
		pS[I].NOP_Next = 1;
		break;

	      case 6: // Undefined-NOP
		/*
		// PUSH ADDRESS
		pS[I].uPCS_ptr_reg++;  pS[I].uPCS_ptr_reg &= 0xFF;
		pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw;
		// POP ADDRESS
		pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
		pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
		*/
		break;

	      case 7: // Undefined-NOP (Raven SKIP)
		pS[I].loc_ctr_reg.raw++;
		pS[I].NOP_Next = 1;
		break;


	      default:
		logmsgf(LT_LAMBDA,0,"Unknown dispatch RPN %o\n",pS[I].disp_word.Operation);
		pS[I].cpu_die_rq=1;
	      }
	    }
	    break;
	  }
	}
	// pS[I].loc_ctr_reg.raw = pS[I].loc_ctr_cnt + 1; // Otherwise get next instruction
	logmsgf(LT_LAMBDA,10,"TREG %d: uPC: CNT %.4o REG %.4o NXT %.o\n",I,pS[I].loc_ctr_cnt,pS[I].loc_ctr_reg.raw,pS[I].loc_ctr_nxt);
      }
      // Fool the CSMRAM data path test
      if(pS[I].CSM_Adr.Addr == 0xfff){
	logmsgf(LT_LAMBDA,10,"RG: CSM Output Reg Loaded from CSMRAM\n");
	pS[I].CSM_Output = pS[I].CSMRAM[pS[I].CSM_Adr.Addr].raw;
      }
      // Handle source read
      if(pS[I].TREG.A_clock_next == 1 && pS[I].TREG.M_clock_next == 1 &&
	 pS[I].TREG.A_WE_L == 1 && pS[I].TREG.M_WE_L == 1){
	// This is probably the source read
	logmsgf(LT_LAMBDA,10,"TREG %d: Triggering source reads\n",I);
	handle_source(I,0);
	logmsgf(LT_LAMBDA,10,"TREG %d: M = 0x%.8X A = 0x%.8X\n",I,pS[I].Mbus,pS[I].Abus);
      }
    }else{
      logmsgf(LT_LAMBDA,10,"RG %d: PMR: SM CLOCK HI: TRAM_PC 0%o\n",I,pS[I].TRAM_PC);
    }
  }
}

// Nubus Slave Operation
void lambda_nubus_pulse(int I){
  if(NUbus_Busy == 2 && NUbus_acknowledge == 0 && NUbus_Address.Card == pS[I].NUbus_ID){
    lambda_nubus_slave(I);
  }
}

// Normal Clock Pulse
void lambda_clockpulse(int I){
  // Run one clock
  // Keep time
  pS[I].cycle_count++;
  if(pS[I].cycle_count == 5){
    if(pS[I].RG_Mode.Aux_Stat_Count_Control == 6){
      pS[I].stat_counter_aux++;
    }
    pS[I].cycle_count = 0;
  }

  // Bus interface operation
  if(pS[I].LCbus_Busy > 0){
    if(pS[I].NUbus_Master == 1){
      // We are the master. Drive the bus.
      nubus_clock_pulse();
      // Pull the nubus clock signals back to the LC bus.
      pS[I].LCbus_error = NUbus_error;
      pS[I].LCbus_Busy = NUbus_Busy;
      pS[I].LCbus_acknowledge = NUbus_acknowledge;
      pS[I].LCbus_Address = NUbus_Address;
      pS[I].LCbus_Data = NUbus_Data;
      pS[I].LCbus_Request = NUbus_Request;
#ifdef CONFIG_CACHE
      if((pS[I].LCbus_Address.raw&0x03) == 0x2){
	pS[I].LCbus_Block[0] = NUbus_Block[0];
	pS[I].LCbus_Block[1] = NUbus_Block[1];
	pS[I].LCbus_Block[2] = NUbus_Block[2];
	pS[I].LCbus_Block[3] = NUbus_Block[3];
      }
#endif
    }else{
#ifdef CONFIG_CACHE
      // We are not the bus master. This must be a cache hit!
      // First, put data on the bus...
      uint8_t Sector_Offset = (pS[I].LCbus_Address.raw&0xC)>>2;
      switch(pS[I].LCbus_Request){
      case VM_READ:
	// WORD READ
	switch(pS[I].LCbus_Address.raw&0x3){
	case 0:
	  // FULL WORD
	  /*
	  logmsgf(LT_LAMBDA,2,"CACHE: READ-WORD-HIT: Returned 0x%.8X\n",
	  pS[I].Cache_Data[pS[I].Cache_Sector][Sector_Offset].word); */
	  pS[I].LCbus_Data.word = pS[I].Cache_Data[pS[I].Cache_Sector][Sector_Offset].word;
	  break;
	case 1: // LOW HALF
	case 3: // HI HALF
	case 2: // BLOCK (illegal)
	default:
	  logmsgf(LT_LAMBDA,0,"CACHE: WORD READ - UI MODE %o\n",pS[I].LCbus_Address.raw&0x3);
	  exit(-1);
	}
	break;
      case VM_BYTE_READ:
	// Byte read
      default:
	logmsgf(LT_LAMBDA,0,"CACHE: HIT: UI OP %o\n",pS[I].LCbus_Request);
	exit(-1);
      }
      // Drive ack
      pS[I].LCbus_acknowledge = 1;
      pS[I].LCbus_error = 0; // Just in case
      // Count down busy
      pS[I].LCbus_Busy--;
      // logmsgf(LT_LAMBDA,2,"LCBUS: BUSY %d\n",pS[I].LCbus_Busy);
#endif
    }
    // Is this a read?
    if((pS[I].LCbus_Request&0x01)==0){
      // Since this is our read request, reload MD here.
      // Since Lambda doesn't handle timeouts, I guess this happens regardless of acknowledge?
      pS[I].MDregister.raw = pS[I].LCbus_Data.word;
#ifdef CONFIG_CACHE
      // Also update the cache if this is a read through and the read ack'd
      if(pS[I].NUbus_Master == 1 && pS[I].Cache_Permit != 0 && pS[I].Cache_Sector_Hit == 1 && pS[I].LCbus_acknowledge == 1){
	uint8_t Sector_Offset = (pS[I].LCbus_Address.raw&0xF)>>2;
	switch(pS[I].LCbus_Request){
	case VM_READ:
	  // Word read
	  switch(pS[I].LCbus_Address.raw&0x03){
	  case 0:
	    // FULL WORD
	    pS[I].Cache_Data[pS[I].Cache_Sector][Sector_Offset].word = pS[I].LCbus_Data.word;
	    pS[I].Cache_Status[pS[I].Cache_Sector][Sector_Offset] = 0x0F;
	    // logmsgf(LT_LAMBDA,2,"CACHE: READ-THROUGH-FILL: WORD: Filled with 0x%.8X\n",pS[I].LCbus_Data.word);
	    break;
	  case 1:
	    // LOW HALF
	  case 3:
	    // HI HALF
	  case 2:
	    // BLOCK
	    pS[I].Cache_Data[pS[I].Cache_Sector][0].word = pS[I].LCbus_Block[0].word;
	    pS[I].Cache_Status[pS[I].Cache_Sector][0] = 0x0F;
	    pS[I].Cache_Data[pS[I].Cache_Sector][1].word = pS[I].LCbus_Block[1].word;
	    pS[I].Cache_Status[pS[I].Cache_Sector][1] = 0x0F;
	    pS[I].Cache_Data[pS[I].Cache_Sector][2].word = pS[I].LCbus_Block[2].word;
	    pS[I].Cache_Status[pS[I].Cache_Sector][2] = 0x0F;
	    pS[I].Cache_Data[pS[I].Cache_Sector][3].word = pS[I].LCbus_Block[3].word;
	    pS[I].Cache_Status[pS[I].Cache_Sector][3] = 0x0F;
	    /*
	    logmsgf(LT_LAMBDA,2,"CACHE: READ-THROUGH-FILL: BLOCK: Sector filled with 0x%.8X 0x%.8X 0x%.8X 0x%.8X\n",
		    pS[I].Cache_Data[pS[I].Cache_Sector][0].word,
		    pS[I].Cache_Data[pS[I].Cache_Sector][1].word,
		    pS[I].Cache_Data[pS[I].Cache_Sector][2].word,
		    pS[I].Cache_Data[pS[I].Cache_Sector][3].word);
	    */
	    break;
	  default:
	    logmsgf(LT_LAMBDA,0,"CACHE: READ-THROUGH-FILL: UI MODE %o\n",pS[I].LCbus_Address.raw&0x3);
	    exit(-1);
	  }
	  break;
	case VM_BYTE_READ:
	  // Byte read
	default:
	  logmsgf(LT_LAMBDA,0,"CACHE: READ-THROUGH-FILL: UI OP %o\n",pS[I].LCbus_Request);
	  exit(-1);
	}
      }
#endif
    }
    // Was it acknowledged?
    if(pS[I].LCbus_acknowledge == 1){
      /*
      if(NUbus_trace == 1){
	logmsgf(LT_NUBUS,10,"NUBUS: Cycle Complete: Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) Ack %o\n",
	       NUbus_Request,NUbus_Address.raw,NUbus_Address.raw,NUbus_Data.word,NUbus_Data.word,NUbus_acknowledge);
      }
      */
      // Release the bus
      pS[I].LCbus_Busy = 0;
      // Also release the NUbus if we have it.
      if(pS[I].NUbus_Master == 1){
	pS[I].NUbus_Master = 0;
	release_nubus_mastership();
      }
    }else{
      // If the request timed out and we have the nubus, release it.
      if(pS[I].LCbus_Busy == 0 && pS[I].NUbus_Master == 1){
	pS[I].NUbus_Master = 0;
	release_nubus_mastership();
      }
    }
  }

  // NUbus slave operation (RG board's interface)
  // Under some conditions the Lambda will talk to itself
  /*
  if(NUbus_Busy == 2 && NUbus_acknowledge == 0 && NUbus_Address.Card == pS[I].NUbus_ID){
    lambda_nubus_slave(I);
  }
  */

  // If we are halted, we are done here.
  if(pS[I].cpu_die_rq != 0){
    return;
  }else{
    /*
    if(pS[I].PMR.Allow_UInst_Clocks != 0 && pS[I].ConReg.uinst_clock_l != 0){
      logmsgf(LT_LAMBDA,10,"CONREG: UINST_CLOCK\n");
      pS[I].ConReg.uinst_clock_l = 0;
    }else{
      pS[I].ConReg.uinst_clock_l = 1;
    }
    */
  }

  // If we are not holding...
  if(pS[I].exec_hold == false && pS[I].PMR.Force_T_Hold == 0){
    if(pS[I].ConReg.t_hold_l != 1){
      pS[I].ConReg.t_hold_l = 1; // Not holding
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"CONREG: Hold Cleared\n");
      }
    }
    // Decrement POPJ-After-Next flag if set
    if(pS[I].popj_after_nxt > -1){
      if(pS[I].popj_after_nxt == 0){
	// Time for popj
	pS[I].loc_ctr_reg.raw = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
	pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"PJAN FIRED: GOING TO %o\n",pS[I].loc_ctr_reg.raw);
	}
	// pS[I].cpu_die_rq = 1;
      }
      pS[I].popj_after_nxt--;
    }

    if(pS[I].slow_dest == true){
      // Slow destination. Waste a cycle.
      // pS[I].stall_count++;
      pS[I].slow_dest = false;
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"SLOW-DEST cycle used\n");
      }
      return;
    }

    if(pS[I].long_inst == true){
      // Long instruction. Waste a cycle.
      // pS[I].stall_count++;
      pS[I].long_inst = false;
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"LONG-INST cycle used\n");
      }
      return;
    }

    if(pS[I].NOP_Next == true){
      // Inhibit bit set. Waste a cycle.
      pS[I].ConReg.nop = 0;
      // pS[I].stall_count++;
      pS[I].NOP_Next = false;
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"NOP-NEXT cycle used\n");
      }
      return;
    }else{
      pS[I].ConReg.nop = 1;
    }

    if(pS[I].cram_write_cyc == true){
      // CRAM or map write. Waste a cycle.
      // pS[I].stall_count++;
      pS[I].cram_write_cyc = false;
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"CRAM WRITE cycle used\n");
      }
      return;
    }

    // MACRO INSTRUCTION DISPATCH
    if(pS[I].macro_dispatch_inst > -1){
      pS[I].macro_dispatch_inst--;
      if(pS[I].macro_dispatch_inst == 0){
	DispatchWord disp_word;
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"MI: MACRO DISPATCH CYCLE\n");
	}
	// Stop conditions
	if(pS[I].RG_Mode.Enable_Misc_MID == 0){ logmsgf(LT_LAMBDA,0,"EMM-OFF\n"); pS[I].cpu_die_rq = 1; }
	// Generate address
	if(((pS[I].LCregister.raw>>1)&0x01) == 0x01){
	  // Left half
	  pS[I].MIDAddr.Opcode = pS[I].MIregister.mi[0].Misc.Opcode;
	}else{
	  // Right half
	  pS[I].MIDAddr.Opcode = pS[I].MIregister.mi[1].Misc.Opcode;
	}
	// If Enable_Misc_MID is set, we dispatch on the whole MISC field of the instruction, if it is a MISC instruction to destination ignore (0).
	// MID memory 6000 - 7777 is used to hold dispatch addresses for MISC (6000-6777) and MISC1 (7000 - 7777).
	// So what we are doing is conditionally setting the high bits if this is an appropriate MISC op.
	pS[I].MIDAddr.Hi = pS[I].RG_Mode.MID_Hi_Adr;
	// Obtain dispatch word
	disp_word.raw = pS[I].MIDmemory[pS[I].MIDAddr.raw];
	// Log it
        if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"MISC/MISC1 INSTRUCTION!\n");
	  logmsgf(LT_LAMBDA,10,"MI: MACRO DISPATCH\n");
          logmsgf(LT_LAMBDA,10,"MID: MIR = 0x%X LC = 0x%X RG.Hi = %o RG.Enable_Misc_MID = %o Opcode %o: GENERATED ADDR %o DATA = 0x%X\n",
		 pS[I].MIregister.raw,pS[I].LCregister.raw,pS[I].RG_Mode.MID_Hi_Adr,pS[I].RG_Mode.Enable_Misc_MID,
		 pS[I].MIDAddr.Opcode,pS[I].MIDAddr.raw,disp_word.raw);
	  logmsgf(LT_LAMBDA,10,"MID: OP %s DEST ",jump_op_str[disp_word.Operation]);
	  {
	    char *location;
	    char symloc[100];
	    int offset;
	    location = "";
	    offset = 0;
	    location = sym_find_last(1, disp_word.PC, &offset);
	    if(location != 0){
	      if(offset != 0){
		sprintf(symloc, "%s+%o", location, offset);
	      }else{
		sprintf(symloc, "%s", location);
	      }
	      logmsgf(LT_LAMBDA,10,"%s",symloc);
	    }
	    logmsgf(LT_LAMBDA,10," (%o)",disp_word.PC);
	  }
	  logmsgf(LT_LAMBDA,10,"\n");
	}
	if(disp_word.StartRead){ logmsgf(LT_LAMBDA,0," START-MEMORY-READ"); pS[I].cpu_die_rq = 1; } // Stop for investigation
	// Handle operation
	switch(disp_word.Operation){
	case 0: // Jump-Branch-Xct-Next
	  // Jump, but DO NOT inhibit the next instruction!
	  if(pS[I].loc_ctr_nxt != -1){
	    logmsgf(LT_LAMBDA,0,"MACRO DISPATCH: Pending JUMP-After-Next collision investigation stop\n");
	    pS[I].cpu_die_rq = 1;
	  }
	  pS[I].loc_ctr_nxt = disp_word.PC;
	  break;

	case 1: // Jump-Branch
	  if(pS[I].loc_ctr_reg.raw != (pS[I].loc_ctr_cnt + 1)){
	    logmsgf(LT_LAMBDA,0,"MACRO DISPATCH: Pending JUMP collision investigation stop\n");
	    pS[I].cpu_die_rq = 1;
	  }
	  pS[I].loc_ctr_reg.raw = disp_word.PC;
	  pS[I].NOP_Next = 1;
	  break;

	case 2: // Jump-Call-Xct-Next
	  // Call, but DO NOT inhibit the next instruction!
	  pS[I].uPCS_ptr_reg++; pS[I].uPCS_ptr_reg &= 0xFF;
	  // Raven does not handle this, but should we? (Should Raven?)
	  if(pS[I].Iregister.Dispatch.LPC){
	    logmsgf(LT_LAMBDA,0,"DISPATCH: LPC set w/ call-xct-next?\n");
	    pS[I].cpu_die_rq = 1;
	  }
	  pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw+1; // Pushes the address of the next instruction
	  if(pS[I].microtrace){
	    char *location;
	    char symloc[100];
	    int offset;

	    logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

	    location = "";
	    offset = 0;
	    location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    logmsgf(LT_LAMBDA,10,"%s (%o)\n",symloc,pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
	  }
	  pS[I].loc_ctr_nxt = disp_word.PC;
	  break;

	case 3: // Jump-Call
	  // PUSH ADDRESS
	  pS[I].uPCS_ptr_reg++;  pS[I].uPCS_ptr_reg &= 0xFF;
	  if(pS[I].Iregister.Dispatch.LPC){
	    pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_cnt; // Stack-Own-Address
	  }else{
	    pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw;
	  }
	  if(pS[I].microtrace){
	    char *location;
	    char symloc[100];
	    int offset;

	    logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

	    location = "";
	    offset = 0;
	    location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    logmsgf(LT_LAMBDA,10,"%s (%o)\n",symloc,pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
	  }
	  // Jump
	  pS[I].loc_ctr_reg.raw = disp_word.PC;
	  pS[I].NOP_Next = 1;
	  break;

	case 6: // Undefined-NOP
	  /*
	  // PUSH ADDRESS
	  pS[I].uPCS_ptr_reg++;  pS[I].uPCS_ptr_reg &= 0xFF;
	  pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw;
	  // POP ADDRESS
	  pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
	  pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
	  */
	  break;

	  /* Are these valid for dispatch? Stop for investigation
	     case 4: // Jump-Return-XCT-Next
	     // POP ADDRESS
	     pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
	     pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
	     break;

	     case 5: // Jump-Return
	     // POP ADDRESS
	     pS[I].loc_ctr_reg.raw = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
	     pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
	     pS[I].NOP_Next = 1;
	     break;
	  */

	default:
	  logmsgf(LT_LAMBDA,0,"Unknown macro-dispatch RPN %o\n",disp_word.Operation);
	  pS[I].cpu_die_rq=1;
	}
	// pS[I].cpu_die_rq = 1;
	// This uses up a cycle
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"MI: MACRO DISPATCH cycle completed\n");
	}
	return;
      }
    }

    // Update PC.
    pS[I].loc_ctr_cnt = pS[I].loc_ctr_reg.raw; // Prepare to fetch next.

    // If AfterNEXT
    if(pS[I].loc_ctr_nxt != -1){
      pS[I].loc_ctr_reg.raw = pS[I].loc_ctr_nxt; // Load next instruction
      pS[I].loc_ctr_nxt = -1;          // Disarm
    }else{
      pS[I].loc_ctr_reg.raw = pS[I].loc_ctr_cnt + 1; // Otherwise get next instruction
    }
    pS[I].last_loc_ctr = pS[I].loc_ctr_reg.raw;    // Save This

    // TRAP PC GENERATOR
    // Lambda does not have traps!

    // History maintenance
    pS[I].History_RAM[pS[I].History_Pointer&0xFFF] = pS[I].loc_ctr_cnt;
    pS[I].History_Pointer++;
    if(pS[I].History_Pointer > 0xFFF){ pS[I].History_Pointer = 0; }

    // Clobber conreg bits
    pS[I].ConReg.halt_request = 0;
    pS[I].ConReg.any_parity_error_synced_l = 1;

    // FETCH
    if(pS[I].spy_wrote_ireg == false){
      int addr = pS[I].loc_ctr_cnt&0xFFFF;
      int paddr = pS[I].CRAM_map[addr>>4] & 03777;
      paddr <<= 4;
      paddr |= (addr&0xF);
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"UI: MAP 0x%X -> 0x%X\n",pS[I].loc_ctr_cnt&0xFFFF,paddr);
      }
      pS[I].Iregister.raw = pS[I].WCS[paddr].raw;
    }else{
      pS[I].spy_wrote_ireg = false;
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"UI: Execute from modified IReg\n");
      }
    }

    // IMOD LOGIC > IR
    if(pS[I].imod_en != 0){
      pS[I].Iregister.raw |= ((uint64_t)pS[I].imod_hi << 32) | pS[I].imod_lo;
      pS[I].imod_en = 0;
      pS[I].imod_hi = 0;
      pS[I].imod_lo = 0;
    }

    // Handle STAT bit
    if(pS[I].RG_Mode.Aux_Stat_Count_Control == 0 && pS[I].Iregister.Stat_Bit != 0){
      pS[I].stat_counter_aux++;
    }
    if(pS[I].RG_Mode.Main_Stat_Count_Control == 0 && pS[I].Iregister.Stat_Bit != 0){
      pS[I].stat_counter_main++;
    }

    // Handle HALT bit
    if((pS[I].Iregister.Halt)){
      logmsgf(LT_LAMBDA,1,"**MInst-HALT** #%d\n",I);
      pS[I].cpu_die_rq = 1;
    }
  }else{
    // We are holding
    if(pS[I].microtrace != 0){
      logmsgf(LT_LAMBDA,10,"CONREG: ");
      if(pS[I].PMR.Force_T_Hold != 0){ logmsgf(LT_LAMBDA,10,"Forced "); }
      logmsgf(LT_LAMBDA,10,"Hold\n");
    }
    if(pS[I].RG_Mode.Aux_Stat_Count_Control == 04){
      pS[I].stat_counter_aux++;
    }
    if(pS[I].RG_Mode.Main_Stat_Count_Control == 04){
      pS[I].stat_counter_main++;
    }
    pS[I].ConReg.t_hold_l = 0;
  }

  // Handle global fields
  if(pS[I].Iregister.Clobbers_Mem_Subr_Bit != 0 && pS[I].ConReg.Enable_NU_Master == 1){
    // "Avoid clobbering mem-subr in progress"
    // Seems to be lit whenever Raven would do a stall for IO completion.
    // Maybe that's what we're supposed to do with it?
    // For timing purposes we do this by holding execution.
    // Is this ever lit when we aren't the bus master?
    // *** YES ***
    // THIS GETS LIT IF WE ARE WAITING FOR THE -RESULT- OF A BUS OPERATION, NOT INITIATING IT!
    // IN THE NORMAL PATH OF THINGS, IT LOOKS LIKE NUBUS MASTERSHIP IS NEVER REALLY OBTAINED!

#ifdef NEVER
    if(pS[I].LCbus_Busy > 0){
      if(pS[I].microtrace != 0){
	logmsgf(LT_LAMBDA,10,"LAMBDA: CMSB: Awaiting completion\n");
      }
      // If we aren't the master, consider taking it?
      /*
      if(NUbus_master != pS[I].NUbus_ID){
	// Do it
	logmsgf(LT_LAMBDA,10,"TAKING NUBUS MASTERSHIP...\n");
	take_nubus_mastership(); // THIS BREAKS THE SDU, BUT CAUSE A BUS CLASH TO SHOW ITSELF
	logmsgf(LT_LAMBDA,10,"HAVE NUBUS MASTERSHIP...\n");
      }
      */
      // pS[I].stall_count++; // Track ticks burned
      pS[I].exec_hold = true;
      return;
    }
#endif
  }
  // MD source stall handling.
  // If reading MD, we are the bus master, and the request is outstanding, stall.
  if(pS[I].Iregister.MSource == 0161 && pS[I].LCbus_Busy > 0 && // NUbus_master == pS[I].NUbus_ID &&
     !(pS[I].LCbus_acknowledge != 0 || pS[I].LCbus_error != 0)){
    if(pS[I].microtrace != 0){
      logmsgf(LT_LAMBDA,10,"LAMBDA: M-SRC-MD: Awaiting cycle completion...\n");
    }
    // pS[I].stall_count++; // Track ticks burned
    pS[I].exec_hold = true;
    return;
  }
  // Handle top-level flag (Raven's PJ14)
  if((pS[I].loc_ctr_nxt != -1 && (pS[I].loc_ctr_nxt&0x40000) != 0) ||
     (pS[I].loc_ctr_reg.raw&0x40000) != 0){
    if(pS[I].microtrace != 0){
      logmsgf(LT_LAMBDA,10,"TOPLEVEL FLAG SET! NOP-NEXT %o LOC-CTR-REG = 0x%X LOC-CTR-NXT = 0x%X\n",
	     pS[I].NOP_Next,pS[I].loc_ctr_reg.raw,pS[I].loc_ctr_nxt);
    }

    if(((pS[I].LCregister.raw>>1)&0x01) == 0x0){
      pS[I].RG_Mode.Need_Macro_Inst_Fetch = 1;
      if(pS[I].microtrace != 0){
	logmsgf(LT_LAMBDA,10,"ODD PHASE LIGHTS NEEDS FETCH\n");
      }
    }else{
      // Clobber pagefault anyway?
      if(pS[I].Page_Fault != 0){
	pS[I].Page_Fault = 0;
      }
    }

    // If needfetch is set, do it
    if(pS[I].RG_Mode.Need_Macro_Inst_Fetch){
      // Raven saves the VMA, but Lambda doesn't?
      if(pS[I].microtrace != 0){
	logmsgf(LT_LAMBDA,10,"NEED-FETCH IS LIT\n");
      }
      // We will need the memory bus. Take it.
      // maybe_take_nubus_mastership(I);
      /*
      if(NUbus_Busy > 0 && pS[I].ConReg.Enable_NU_Master == 1){
	if(pS[I].microtrace != 0){
	  logmsgf(LT_LAMBDA,10,"AWAITING MEMORY BUS...\n");
	}
	// No, burn a cycle and come back
	pS[I].stall_count++; // Track ticks burned
	pS[I].exec_hold = true;
	return;
      }
      */
      // We can has bus
      pS[I].VMAregister.raw = (pS[I].LCregister.raw >> 2) & 0x1ffffff;
      VM_resolve_address(I,VM_READ,0);
      if(pS[I].Page_Fault == 0 && pS[I].ConReg.Enable_NU_Master == 1){
	if(pS[I].RG_Mode.Aux_Stat_Count_Control == 01){
	  pS[I].stat_counter_aux++;
	}
	if(pS[I].RG_Mode.Main_Stat_Count_Control == 01){
	  pS[I].stat_counter_main++;
	}
	lcbus_io_request(VM_READ,I,pS[I].vm_phys_addr.raw,0);
      }
      // Clear needfetch
      pS[I].RG_Mode.Need_Macro_Inst_Fetch = 0;
      pS[I].mirInvalid = 1;
    }else{
      if(pS[I].microtrace != 0){
	logmsgf(LT_LAMBDA,10,"NO NEED FOR FETCHING\n");
      }
    }
    // Advance LC
    pS[I].LCregister.raw += 2;
    if(pS[I].RG_Mode.Aux_Stat_Count_Control == 03){
      pS[I].stat_counter_aux++;
    }
    if(pS[I].RG_Mode.Main_Stat_Count_Control == 03){
      pS[I].stat_counter_main++;
    }

    if(pS[I].microtrace != 0){
      logmsgf(LT_LAMBDA,10,"MI: LC ADVANCE: NEW LC = 0x%X (0%o)\n",pS[I].LCregister.raw,pS[I].LCregister.raw);
    }
    // All done, clear the flags
    if(pS[I].loc_ctr_nxt != -1 && (pS[I].loc_ctr_nxt&0x40000)){ pS[I].loc_ctr_nxt ^= 0x40000; }
    if((pS[I].loc_ctr_reg.raw&0x40000) != 0){ pS[I].loc_ctr_reg.raw ^= 0x40000; }
    // Stop for investigation
    // pS[I].cpu_die_rq = 1;
  }
  // Handle Macro-Stream-Advance
  if(pS[I].Iregister.Macro_Stream_Advance != 0){
    // Is this the expected scenario?
    if(!(pS[I].Iregister.Opcode == 3 && pS[I].Iregister.MSource == 0107 && pS[I].Iregister.Dispatch.Pos == 020)){ // && pS[I].RG_Mode.Need_Macro_Inst_Fetch != 1)){
      logmsgf(LT_LAMBDA,0,"USE OF MACRO-STREAM-ADVANCE DOES NOT FIT EXPECTED SCENARIO - INVESTIGATE!\n");
      pS[I].cpu_die_rq = 1;
    }
    // Will we need the bus?
    if(((pS[I].LCregister.raw>>1)&0x01) == 0x0){
      // We will need the memory bus. Take it.
      // maybe_take_nubus_mastership(I);
      /*
      if(NUbus_Busy > 0 && pS[I].ConReg.Enable_NU_Master == 1){
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"MACRO-STREAM-ADVANCE: AWAITING MEMORY BUS...\n");
	}
	// No, burn a cycle and come back
	pS[I].stall_count++; // Track ticks burned
	pS[I].exec_hold = true;
	return;
      }
      */
    }else{
      // This MSA will not cause a fetch. Is it acceptable to clobber VMA and read anyway?
      // We should check.
      // logmsgf(LT_LAMBDA,10,"MACRO-STREAM-ADVANCE: NO READ WILL BE NECESSARY - INVESTIGATE IMPLICATIONS OF DOING IT ANYWAY / NOT DOING IT\n");
      // pS[I].cpu_die_rq = 1;
    }

    if(((pS[I].LCregister.raw>>1)&0x01) == 0x0){
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"MACRO-STREAM-ADVANCE: INITIATING MEMORY READ...\n");
      }
      pS[I].VMAregister.raw = (pS[I].LCregister.raw >> 2) & 0x1ffffff;
      VM_resolve_address(I,VM_READ,0);
      if(pS[I].Page_Fault == 0 && pS[I].ConReg.Enable_NU_Master == 1){
	if(pS[I].RG_Mode.Aux_Stat_Count_Control == 01){
	  pS[I].stat_counter_aux++;
	}
	if(pS[I].RG_Mode.Main_Stat_Count_Control == 01){
	  pS[I].stat_counter_main++;
	}
	lcbus_io_request(pS[I].vm_byte_mode|VM_READ,I,pS[I].vm_phys_addr.raw,0);
      }
    }
    // Advance LC, then load VMA from LC and initiate a read.
    pS[I].LCregister.raw += 2; // Yes, this is right
    if(pS[I].RG_Mode.Aux_Stat_Count_Control == 03){
      pS[I].stat_counter_aux++;
    }
    if(pS[I].RG_Mode.Main_Stat_Count_Control == 03){
      pS[I].stat_counter_main++;
    }
    if(pS[I].microtrace){
      logmsgf(LT_LAMBDA,10,"MACRO-STREAM-ADVANCE: ");
      logmsgf(LT_LAMBDA,10,"NEW LC = 0x%X (0%o)\n",pS[I].LCregister.raw,pS[I].LCregister.raw);
    }
  }

  // If we are in forced hold, die here.
  if(pS[I].PMR.Force_T_Hold != 0){
    pS[I].exec_hold = true;
    return;
  }

  pS[I].exec_hold = false; // Release hold
  if(pS[I].Iregister.Stat_Bit != 0){ logmsgf(LT_LAMBDA,0,"\nSTAT\n"); pS[I].cpu_die_rq = 1; }
  // if(pS[I].Iregister.ILong != 0){ logmsgf(LT_LAMBDA,10,"\nILong\n"); pS[I].cpu_die_rq = 1; }
  // if(pS[I].Iregister.Macro_IR_Disp != 0){ logmsgf(LT_LAMBDA,10,"\nMIR-DISP\n"); pS[I].cpu_die_rq = 1; }
  if(pS[I].Iregister.PopJ_After_Next != 0){
    // If we are using SLOW-DEST we add another instruction
    if(pS[I].Iregister.Slow_Dest != 0){
      pS[I].popj_after_nxt = 2;
    }else{
      pS[I].popj_after_nxt = 1;
    }
  }

  // If SLOW-DEST is set, we burn an extra cycle after this instruction to allow
  // writes to complete.
  if(pS[I].Iregister.Slow_Dest != 0){ pS[I].slow_dest = true; } // Burn the next cycle
  // ILONG works the same way
  if(pS[I].Iregister.ILong != 0){ pS[I].long_inst = true; } // Burn the next cycle
  // Fetch sources
  handle_source(I,0);

  // Source-To-Macro-IR.
  // Load the Macro IR from the pS[I].Mbus
  if(pS[I].Iregister.Src_to_Macro_IR != 0){
    if(((pS[I].LCregister.raw>>1)&0x01) == 0x01){
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"S2MIR: Loaded MIR\n");
      }
      pS[I].MIregister.raw = pS[I].Mbus;
      pS[I].mirInvalid = 0;
    }else{
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"S2MIR: Suppressed Loading MIR\n");
      }
      if(pS[I].mirInvalid == 1 || pS[I].loc_ctr_reg.raw > 036000){
	pS[I].MIregister.raw = pS[I].Mbus;
	//pS[I].MIregister.raw &= 0xFFFF0000;
	//pS[I].MIregister.raw |= (pS[I].Mbus & 0xFFFF0000);
	pS[I].mirInvalid = 0;
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"S2MIR: mirInvalid override suppress of MIR\n");
	  logmsgf(LT_LAMBDA,10,"MID: MIR = %o %o LC = %o\n",
		 pS[I].MIregister.mi[0].raw,pS[I].MIregister.mi[1].raw,
		 pS[I].LCregister.raw);
	}
      }
    }
  }

  // MIR-DISP
  if(pS[I].Iregister.Macro_IR_Disp != 0){
    if(pS[I].microtrace){
      logmsgf(LT_LAMBDA,10,"MIR-DISP: Flag Set\n");
    }
    pS[I].macro_dispatch_inst = 1; // Next instruction will be a macro dispatch
  }

  // Switch opcode
  switch(pS[I].Iregister.Opcode){
  case 0: // ALU-OP
    // Perform ALU operation
    operate_alu(I);

    // Operate O bus
    handle_o_bus(I);

    if(pS[I].Iregister.ALU.Misc > 0){ logmsgf(LT_LAMBDA,0,"ALU-MISC "); pS[I].cpu_die_rq = 1; }

    // Handle destination selector
    handle_destination(I);

    // Halt if spare bit set
    if(pS[I].Iregister.ALU.Spare > 0){ logmsgf(LT_LAMBDA,0,"ALU-SPARE "); pS[I].cpu_die_rq = 1; }

    // Process Q register
    handle_q_register(I);

    // Load MFO from O-bus
    pS[I].MFObus = pS[I].Obus;
    break;

  case 1: // BYTE-OP
    // Operate shifter. Result goes to O bus.
    operate_shifter(I);
    // Load MFO from O-bus
    pS[I].MFObus = pS[I].Obus;
    // Store result
    handle_destination(I);

    if(pS[I].Iregister.Byte.Misc > 0){ logmsgf(LT_LAMBDA,0,"Misc "); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.Byte.Spare > 0){ logmsgf(LT_LAMBDA,0,"BYTE-SPARE "); pS[I].cpu_die_rq = 1; }
    break;

  case 2: // JUMP-OP
    if(pS[I].Iregister.Jump.LC_Increment != 0){ logmsgf(LT_LAMBDA,0," LCINC"); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.Jump.Spare != 0){ logmsgf(LT_LAMBDA,0," JUMP-SPARE"); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.Jump.Spare2 != 0){ logmsgf(LT_LAMBDA,0," JUMP-SPARE2"); pS[I].cpu_die_rq = 1; }

    // Handle condition
    pS[I].test_true = false;
    if(pS[I].Iregister.Jump.Test != 0){
      // Operate ALU
      alu_sub_stub(I,0); // Do M-A
      alu_cleanup_result(I);

      // Perform test
      switch(pS[I].Iregister.Jump.Cond){

      case 01: // LAM-JUMP-COND-M<A
	if((0x80000000^pS[I].Mbus) < (0x80000000^pS[I].Abus)){
	  pS[I].test_true = true;
	}
	break;

      case 02: // LAM-JUMP-COND-M<=A
	if((pS[I].Abus == 0 && pS[I].Mbus == 0x80000000) || ((pS[I].ALU_Result&0x80000000) != 0)){
	  pS[I].test_true = true;
	}
	break;

      case 03: // M != A
	if(pS[I].ALU_Result != 0xFFFFFFFF){
	  pS[I].test_true = true;
	}
	break;

      case 04: // LAM-JUMP-COND-PAGE-FAULT (INVERTED!)
	if(pS[I].Page_Fault == 0){
	  pS[I].test_true = 1;
	}
	break;

      case 05: // LAM-JUMP-COND-PAGE-FAULT-OR-INTERRUPT
	if(pS[I].Page_Fault != 0){
	  pS[I].test_true = 1;
	}
	// DETECT INTERRUPT
	{
	  int x=0;
	  if(pS[I].InterruptPending != 0){
	    while(x<0x100){
	      if(pS[I].InterruptStatus[x] != 0){
		// We have an interrupt!
		// Stuff vector in interrupt-pointer and return true
		pS[I].InterruptVector = x;
		pS[I].test_true = 1;
		break;
	      }
	      x++;
	    }
	  }
	}
	break;

      case 06: // LAM-JUMP-COND-PAGE-FAULT-OR-INTERRUPT-OR-SEQUENCE-BREAK
	// SEQUENCE BREAK BIT IS INVERTED
	if(pS[I].Page_Fault != 0 || pS[I].RG_Mode.Sequence_Break == 0){
	  pS[I].test_true = 1;
	}
	// DETECT INTERRUPT
	{
          int x=0;
	  if(pS[I].InterruptPending != 0){
	    while(x<0x100){
	      if(pS[I].InterruptStatus[x] != 0){
		// We have an interrupt!
		// Stuff vector in interrupt-pointer and return true
		pS[I].InterruptVector = x;
		pS[I].test_true = 1;
		break;
	      }
	      x++;
	    }
	  }
        }
	break;

      case 07: // LAM-JUMP-COND-UNC
	pS[I].test_true = true;
	break;

      case 011: // LAM-JUMP-COND-DATA-TYPE-NOT-EQUAL
	if((pS[I].Mbus&0x3E000000) != (pS[I].Abus&0x3E000000)){
	  pS[I].test_true = true;
	}
	break;

      default:
	logmsgf(LT_LAMBDA,0,"Unknown jump cond %o\n",pS[I].Iregister.Jump.Cond);
	pS[I].cpu_die_rq = 1;
      }
    }else{
      // BIT-SET
      pS[I].test_true = left_rotate(pS[I].Mbus,pS[I].Iregister.Jump.Cond)&0x01;
    }
    // If the invert bit is set, reverse the condition
    if(pS[I].Iregister.Jump.Invert){
      pS[I].test_true ^= 1;
    }

    /* Handle RPN.
       CODES ARE:
         R P N  (RETURN, PUSH, INHIBIT)

         0 0 0 = Branch-Xct-Next
         0 0 1 = Branch
         0 1 0 = Call-Xct-Next
         0 1 1 = Call
         1 0 0 = Return-Xct-Next
         1 0 1 = Return
         1 1 0 = NOP (JUMP2-XCT-NEXT) (UNDEFINED ON LAMBDA)
         1 1 1 = SKIP (JUMP2) (UNDEFINED ON LAMBDA)
    */

    if(pS[I].test_true){
      // If we have a pending POPJ-AFTER-NEXT, cancel it
      if(pS[I].popj_after_nxt != -1 && (pS[I].Iregister.Jump.RPN != 2 && pS[I].Iregister.Jump.RPN != 3 && pS[I].Iregister.Jump.RPN != 5)){
	logmsgf(LT_LAMBDA,0,"JUMP: Pending PJAN investigation stop: Not CALL or RETURN (Op %o)\n",pS[I].Iregister.Jump.RPN);
	pS[I].cpu_die_rq = 1;
      }
      // Handle operation
      switch(pS[I].Iregister.Jump.RPN){
      case 0: // Jump-Branch-Xct-Next
        // Jump, but DO NOT inhibit the next instruction!
	if(pS[I].loc_ctr_nxt != -1){
	  logmsgf(LT_LAMBDA,0,"JUMP: Pending JUMP-After-Next collision investigation stop\n");
	  pS[I].cpu_die_rq = 1;
	}
        pS[I].loc_ctr_nxt = pS[I].Iregister.Jump.Address;
        break;

      case 1: // Jump-Branch
	if(pS[I].microtrace && pS[I].loc_ctr_reg.raw != (pS[I].loc_ctr_cnt + 1)){
	  logmsgf(LT_LAMBDA,10,"JUMP: Pending JUMP collision investigation marker\n");
	  // pS[I].cpu_die_rq = 1;
	}
        pS[I].loc_ctr_reg.raw = pS[I].Iregister.Jump.Address;
        pS[I].NOP_Next = 1;
        break;

      case 2: // Jump-Call-Xct-Next
        // Call, but DO NOT inhibit the next instruction!
        pS[I].uPCS_ptr_reg++; pS[I].uPCS_ptr_reg &= 0xFF;
        pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw+1; // Pushes the address of the next instruction
        if(pS[I].microtrace){
          char *location;
          char symloc[100];
          int offset;

          logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

          location = "";
          offset = 0;
          location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	  if(location != 0){
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    logmsgf(LT_LAMBDA,10,"%s",symloc);
	  }
          logmsgf(LT_LAMBDA,10," (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
        }
        pS[I].loc_ctr_nxt = pS[I].Iregister.Jump.Address;
	if(pS[I].popj_after_nxt == 0){
	  // PJAN is armed. We want this call to return to my caller instead of here.
	  // So we'll pop this return address now.
	  pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
	  pS[I].popj_after_nxt = -1;
	}
        break;

      case 3: // Jump-Call
        // PUSH ADDRESS
        pS[I].uPCS_ptr_reg++;  pS[I].uPCS_ptr_reg &= 0xFF;
        pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw;
        if(pS[I].microtrace){
          char *location;
          char symloc[100];
          int offset;

          logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

          location = "";
          offset = 0;
          location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	  if(location != 0){
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    logmsgf(LT_LAMBDA,10,"%s",symloc);
	  }
          logmsgf(LT_LAMBDA,10," (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
        }
        // Jump
        pS[I].loc_ctr_reg.raw = pS[I].Iregister.Jump.Address;
        pS[I].NOP_Next = 1;
	if(pS[I].popj_after_nxt == 0){
	  // PJAN is armed. We want this call to return to my caller instead of here.
	  // So we'll pop this return address now.
	  pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
	  pS[I].popj_after_nxt = -1;
	}
        break;

      case 4: // Jump-Return-XCT-Next
	if(pS[I].popj_after_nxt != -1){
	  // PJAN is armed. Do not double!
	  logmsgf(LT_LAMBDA,0,"RETURN-XCT-NEXT with PJAN armed!\n");
	  pS[I].cpu_die_rq=1;
	}
        // POP ADDRESS
        pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
        pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
        break;

      case 5: // Jump-Return
	if(pS[I].popj_after_nxt != -1){
	  // PJAN is armed. Do not double!
	  // logmsgf(LT_LAMBDA,10,"RETURN with PJAN armed!\n");
	  // pS[I].cpu_die_rq=1;
	  // All we are doing is making the return immediate, so just disable PJAN.
	  pS[I].popj_after_nxt = -1;
	}
        // POP ADDRESS
        pS[I].loc_ctr_reg.raw = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
        pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
        pS[I].NOP_Next = 1;
        break;

      default:
	logmsgf(LT_LAMBDA,0,"Unknown jump RPN %o\n",pS[I].Iregister.Jump.RPN);
	pS[I].cpu_die_rq=1;
      }
    }

    break;
  case 3: // DISP-OP
    {
      // Dispatch items
      uint32_t Mask=0;
      uint32_t dispatch_source=0;
      int gc_volatilty_flag;
      int oldspace_flag;
      int disp_address=0;
      DispatchWord disp_word;

      // Stop for investigation if...
      // if(pS[I].Iregister.Dispatch.Constant > 0){ logmsgf(LT_LAMBDA,10,"Constant "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].Iregister.Dispatch.LPC > 0){ logmsgf(LT_LAMBDA,10,"LPC "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].Iregister.Dispatch.Write_VMA > 0){ logmsgf(LT_LAMBDA,10,"WriteVMA "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta > 0){ logmsgf(LT_LAMBDA,10,"EnableGCVMeta "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].Iregister.Dispatch.Enable_Oldspace_Meta > 0){ logmsgf(LT_LAMBDA,10,"EnableOldspaceMeta "); pS[I].cpu_die_rq = 1; }
      if(pS[I].Iregister.Dispatch.Spare > 0){ logmsgf(LT_LAMBDA,0,"DISP-SPARE "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].popj_after_nxt != -1){ logmsgf(LT_LAMBDA,10,"DISPATCH with POPJ-AFTER-NEXT armed?\n"); pS[I].cpu_die_rq = 1; }

      // Load VMA from M source
      if(pS[I].Iregister.Dispatch.Write_VMA != 0){ pS[I].VMAregister.raw = pS[I].Mbus; }

      // Lambda doesn't have dispatch-source, so I assume it's always R-bus
      Mask = (1 << pS[I].Iregister.Dispatch.Len) - 1;
      // Meta handling
      if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta || pS[I].Iregister.Dispatch.Enable_Oldspace_Meta){
	Mask = Mask & 0xfffffffe;
      }
      // Investigation stop
      if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta && pS[I].Iregister.Dispatch.Enable_Oldspace_Meta){
	logmsgf(LT_LAMBDA,0,"DISPATCH: Enable GCV and Oldspace meta simultaneously?\n");
	pS[I].cpu_die_rq=10;
      }
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"DISPATCH: GENERATED MASK 0x%X\n",Mask);
      }

      // Lambda does not have a rotate direction flag.
      dispatch_source = left_rotate(pS[I].Mbus, pS[I].Iregister.Dispatch.Pos) & Mask;

      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"DISPATCH: dispatch_source = 0x%X\n",dispatch_source);
      }

      // More meta bits
      gc_volatilty_flag = 0;
      if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta != 0){
	int present_gcv = 0;
	// IDENTIFYING GC VOLATILITY BIT
	// Bit 4?
	// "MAP2C-4 IS REALLY FROM THE GC-WRITE-LOGIC, NOT DIRECTLY THE L2MAP, THESE DAYS."
	// GCV happens when a NEWER object is written into an OLDER memory.

	// Do a map resolve for what's in MD
	pS[I].vm_lv2_index.raw = 0;
	pS[I].vm_lv2_index.VPage_Offset = pS[I].MDregister.VM.VPage_Offset;
	pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block;
	// Extract the present LV1 GC volatility
	// present_gcv = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB;
	present_gcv = (~pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB) & 03;
	// Raven's CACHED GCV is (lv2_control & 0x1800) >> 11;
	// GCV FLAG is (cached_gcv + 4 > (map_1_volatility ^ 7)) ? 0 : 1;

	// Our CACHED GCV is the lv2 meta bits of the last reference.

	// LV1 Meta Bits:
	// 2 bits!
	// "For hardware convenience, all three L1 map meta bits are stored in COMPLEMENTED form."
	// S  H    What
	// 0 (3) = Static Region (OLDEST)
	// 1 (2) = Dynamic Region
	// 2 (1) = Active Consing Region
	// 3 (0) = Extra PDL Region (NEWEST)

	// LV2 Meta Bits:
	// 6 bits! But the bottom 2 bits are the same as the LV1 bits.
	// 040 = Oldspace
	// 020 = GCV-Flag
	// 003 = LV1 GC Volatility

	// So, if CACHED GCV is less than PRESENT GCV we wrote a newer item into an older page.
	// The trap is taken if the flag is 0, so we want the inverse.
	// What do I do if the MB validity bit isn't set?

	// Meta bits are un-inverted now.
	// gc_volatilty_flag 1 == don't trap
	// So we want to set it 0 if we should trap.
	// if(pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid != 0){ logmsgf(LT_LAMBDA,10,"GCV: LV1 invalid?\n"); pS[I].cpu_die_rq=1; } // Investigate
	// if((pS[I].cached_gcv&03) <= present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){ gc_volatilty_flag = 1; }else{ pS[I].cpu_die_rq=0; } // ILLOP at PHTDEL6+11

	// This comparison is correct for non-inverted meta.
	// For the moment, LV1 invalidity forces a trap. This isn't conclusively proven correct, and may change.
	// if((pS[I].cached_gcv&03) > present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){ gc_volatilty_flag = 1; }
	if(pS[I].cached_gcv <= present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){
	  gc_volatilty_flag = 1;
	}

	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"DISPATCH: GCV: CACHED (LV2) GCV 0x%X\n",pS[I].cached_gcv&03);
	  logmsgf(LT_LAMBDA,10,"DISPATCH: GCV: PRESENT (LV1) GCV 0x%X\n",present_gcv);
	  logmsgf(LT_LAMBDA,10,"DISPATCH: GCV: LV1 ENT 0x%X = 0x%X (Meta 0x%X Validity %o)\n",
		 pS[I].MDregister.VM.VPage_Block,pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].raw,
		 pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB,
		 pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid);
	  logmsgf(LT_LAMBDA,10,"DISPATCH: GCV: LV2 ENT 0x%X = 0x%X (Meta 0x%X)\n",
		 pS[I].vm_lv2_index.raw,pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].raw,
		 pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta);
	}
      }
      oldspace_flag = 0;
      if(pS[I].Iregister.Dispatch.Enable_Oldspace_Meta != 0){
	// Do a map resolve for what's in MD
	pS[I].vm_lv2_index.raw = 0;
	pS[I].vm_lv2_index.VPage_Offset = pS[I].MDregister.VM.VPage_Offset;
	pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block;
	// Extract the oldspace bit (5?)
	// oldspace_flag 0 means trap (oldspace)
	// oldspace_flag 1 means don't trap (newspace)
	// Reversing this causes infinite loop (GCV never tested), so this has to be right.
	if((pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta&0x20) == 0x20){ oldspace_flag = 1; } // Not oldspace, don't trap
	if(pS[I].microtrace){
	  logmsgf(LT_LAMBDA,10,"DISPATCH: META: LV2 ENT 0x%X = 0x%X (Meta 0x%X)\n",
		 pS[I].vm_lv2_index.raw,pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].raw,
		 pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta);
	}
      }
      // Set dispatch address
      // Lambda uses the A source as the dispatch address!
      // disp_address = (mir_mask & MInst_Disp_Address) | dispatch_source | gc_volatilty_flag | oldspace_flag;
      disp_address = pS[I].Iregister.ASource|dispatch_source;
      // Handle oldspace/GCV meta
      if(pS[I].Iregister.Dispatch.Enable_Oldspace_Meta != 0 || pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta){
	// I think this is how this is supposed to work
	disp_address |= (oldspace_flag|gc_volatilty_flag);
      }
      // Load dispatch constant register
      pS[I].disp_constant_reg = pS[I].Iregister.Dispatch.Constant;
      // Lambda has no dispatch opcode field, so I assume it is always DISPATCH
      disp_word.raw = pS[I].Amemory[(disp_address)]; // A-source is already offset // Dmemory[disp_address];
      if(pS[I].microtrace){
	logmsgf(LT_LAMBDA,10,"DISPATCH: GENERATED ADDRESS 0x%X AND FETCHED WORD 0x%X\n",disp_address,disp_word.raw);
      }
      // Handle dispatch word
      if(pS[I].microtrace){
	char *location;
	char symloc[100];
	int offset;

	logmsgf(LT_LAMBDA,10,"DISPATCH: OP %s DEST ",jump_op_str[disp_word.Operation]);
	location = "";
	offset = 0;
	location = sym_find_last(1, disp_word.PC, &offset);
	if(location != 0){
	  if(offset != 0){
	    sprintf(symloc, "%s+%o", location, offset);
	  }else{
	    sprintf(symloc, "%s", location);
	  }
	  logmsgf(LT_LAMBDA,10,"%s",symloc);
	}
	logmsgf(LT_LAMBDA,10," (%o)\n",disp_word.PC);
      }
      // Handle operation of Start-Memory-Read
      if(disp_word.StartRead){
	if(pS[I].microtrace != 0){
	  logmsgf(LT_LAMBDA,10," START-MEMORY-READ");
	}
	// Load VMA from pS[I].Obus and initiate a read.
	pS[I].VMAregister.raw = pS[I].Mbus; // Load VMA
	VM_resolve_address(I,VM_READ,0);
	if(pS[I].Page_Fault == 0 && pS[I].ConReg.Enable_NU_Master == 1){
	  // Do it
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control == 01){
	    pS[I].stat_counter_aux++;
	  }
	  if(pS[I].RG_Mode.Main_Stat_Count_Control == 01){
	    pS[I].stat_counter_main++;
	  }
	  lcbus_io_request(VM_READ,I,pS[I].vm_phys_addr.raw,0);
	}
      }
      // Handle operation
      switch(disp_word.Operation){
      case 0: // Jump-Branch-Xct-Next
        // Jump, but DO NOT inhibit the next instruction!
	if(pS[I].loc_ctr_nxt != -1){
	  logmsgf(LT_LAMBDA,0,"DISPATCH: Pending JUMP-After-Next collision investigation stop\n");
	  pS[I].cpu_die_rq = 1;
	}
        pS[I].loc_ctr_nxt = disp_word.PC;
        break;

      case 1: // Jump-Branch
	if(pS[I].loc_ctr_reg.raw != (pS[I].loc_ctr_cnt + 1)){
	  logmsgf(LT_LAMBDA,0,"DISPATCH: Pending JUMP collision investigation stop\n");
	  pS[I].cpu_die_rq = 1;
	}
        pS[I].loc_ctr_reg.raw = disp_word.PC;
        pS[I].NOP_Next = 1;
        break;

      case 2: // Jump-Call-Xct-Next
        // Call, but DO NOT inhibit the next instruction!
        if(pS[I].popj_after_nxt != -1){
          // PJAN is armed. Do not double!
          pS[I].popj_after_nxt = -1;
        }
        pS[I].uPCS_ptr_reg++; pS[I].uPCS_ptr_reg &= 0xFF;
	// Raven does not handle this, but should we? (Should Raven?)
	if(pS[I].Iregister.Dispatch.LPC){
	  logmsgf(LT_LAMBDA,0,"DISPATCH: LPC set w/ call-xct-next?\n");
	  pS[I].cpu_die_rq = 1;
	}
	pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw+1; // Pushes the address of the next instruction
        if(pS[I].microtrace){
          char *location;
          char symloc[100];
          int offset;

          logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

          location = "";
          offset = 0;
          location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	  if(location != 0){
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    logmsgf(LT_LAMBDA,10,"%s",symloc);
	  }
          logmsgf(LT_LAMBDA,10," (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
        }
        pS[I].loc_ctr_nxt = disp_word.PC;
        if(pS[I].popj_after_nxt == 0){
          // PJAN is armed. We want this call to return to my caller instead of here.
          // So we'll pop this return address now.
          pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
          pS[I].popj_after_nxt = -1;
        }
        break;

      case 3: // Jump-Call
        // PUSH ADDRESS
        if(pS[I].popj_after_nxt != -1){
          // PJAN is armed. Do not double!
          //logmsgf(LT_LAMBDA,10,"RETURN-XCT-NEXT with PJAN armed!\n");
          pS[I].popj_after_nxt = -1;
        }
        pS[I].uPCS_ptr_reg++;  pS[I].uPCS_ptr_reg &= 0xFF;
	if(pS[I].Iregister.Dispatch.LPC){
	  pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_cnt; // Stack-Own-Address
	}else{
	  pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw;
	}
        if(pS[I].microtrace){
          char *location;
          char symloc[100];
          int offset;

          logmsgf(LT_LAMBDA,10,"uStack[%o] = ",pS[I].uPCS_ptr_reg);

          location = "";
          offset = 0;
          location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	  if(location != 0){
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    logmsgf(LT_LAMBDA,10,"%s",symloc);
	  }
          logmsgf(LT_LAMBDA,10," (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
        }
        // Jump
        pS[I].loc_ctr_reg.raw = disp_word.PC;
        pS[I].NOP_Next = 1;
        if(pS[I].popj_after_nxt == 0){
          // PJAN is armed. We want this call to return to my caller instead of here.
          // So we'll pop this return address now.
          pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
          pS[I].popj_after_nxt = -1;
        }
        break;

      case 4: // Jump-Return-XCT-Next
        // POP ADDRESS
        pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
        pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
        break;

	// Used in d-swap-quantum-map-dispatch, should return
      case 5: // Jump-Return
        // POP ADDRESS
        pS[I].loc_ctr_reg.raw = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
        pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
        pS[I].NOP_Next = 1;
        break;

      case 6: // Undefined-NOP
	/*
        // PUSH ADDRESS
        pS[I].uPCS_ptr_reg++;  pS[I].uPCS_ptr_reg &= 0xFF;
	pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw;
        // POP ADDRESS
        pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
        pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
	*/
	break;

      case 7: // Undefined-NOP (Raven SKIP)
	pS[I].loc_ctr_reg.raw++;
        pS[I].NOP_Next = 1;
        break;

      default:
	logmsgf(LT_LAMBDA,0,"Unknown dispatch RPN %o\n",disp_word.Operation);
	pS[I].cpu_die_rq=1;
      }
    }
    break;
  }

  // Done with instruction
  if(pS[I].NOP_Next != 0){
    pS[I].ConReg.nop_next = 0;
  }else{
    pS[I].ConReg.nop_next = 1;
  }

  // If we are doing a single step, stop
  if(pS[I].ConReg.Enable_SM_Clock != 1 || pS[I].PMR.Advance_UInst_Request == 1){
    if(pS[I].microtrace){
      logmsgf(LT_LAMBDA,10,"LAMBDA %d: Single Step completed\n",I);
    }
    pS[I].cpu_die_rq=1;
  }

  // Advancing T here breaks the detection of the uinst boundary.

  // Debug stuff
#ifdef LAMBDA_DEBUGTRACE
  if(!pS[I].microtrace){
    debugtrace_ir[I][debugtrace_ptr[I]] = pS[I].Iregister.raw;
    debugtrace_reg[I][debugtrace_ptr[I]][0] = pS[I].loc_ctr_cnt;
    debugtrace_reg[I][debugtrace_ptr[I]][1] = pS[I].loc_ctr_reg.raw;
    debugtrace_reg[I][debugtrace_ptr[I]][2] = pS[I].Abus;
    debugtrace_reg[I][debugtrace_ptr[I]][3] = pS[I].Mbus;
    debugtrace_reg[I][debugtrace_ptr[I]][4] = pS[I].Obus;
    debugtrace_reg[I][debugtrace_ptr[I]][5] = pS[I].uPCS_ptr_reg;
    debugtrace_reg[I][debugtrace_ptr[I]][6] = pS[I].VMAregister.raw;
    debugtrace_reg[I][debugtrace_ptr[I]][7] = pS[I].MDregister.raw;
    // debugtrace_reg[I][debugtrace_ptr[I]][8] = pS[I].LCregister;
    debugtrace_reg[I][debugtrace_ptr[I]][9] = 0;
    // Set flags
    // Next!
    debugtrace_ptr[I]++; if(debugtrace_ptr[I] == MAX_DEBUG_TRACE){ debugtrace_ptr[I] = 0; }
  }
#endif

  if(pS[I].microtrace || pS[I].Iregister.Halt || pS[I].cpu_die_rq){
    debug_disassemble_IR(I);
  }

  // Stat counter hi.c
  if(pS[I].RG_Mode.Aux_Stat_Count_Control == 07){
    pS[I].stat_counter_aux++;
  }
  if(pS[I].RG_Mode.Main_Stat_Count_Control == 07){
    pS[I].stat_counter_main++;
  }

  // Enable microtracing
  /*
  if(pS[I].loc_ctr_reg.raw == 036521){
    pS[I].microtrace = true;
    NUbus_trace = 1;
  }
  */

  // Die if requested
  if(pS[I].cpu_die_rq){
    logmsgf(LT_LAMBDA,1,"[CPU %d] MASTER CLOCK STOPPED\n",I);
    pS[I].ConReg.halt_request = 1;
    pS[I].ConReg.uinst_clock_l = 0;
    pS[I].uI_Clock_Pulse = false;
    pS[I].wrote_uPC = false;
    pS[I].SM_Clock = 1;
    pS[I].TRAM_PC = 07000; // TRAM will loop here on a "normal" halt. It's the "halt version" of 03000
    //disassemble_IR();
    //disassemble_MIR();
#ifdef LAMBDA_DEBUGTRACE
    if(!pS[I].microtrace && pS[I].loc_ctr_reg.raw != 036004){
      int x=debugtrace_ptr[I];
      logmsgf(LT_LAMBDA,1,"Writing debug log...\n");
      write_debugtrace_ent(I,x);
      x++;
      if(x == MAX_DEBUG_TRACE){ x=0; }
      while(x != debugtrace_ptr[I]){
	write_debugtrace_ent(I,x);
	x++;
	if(x == MAX_DEBUG_TRACE){ x=0; }
      }
    }
    logmsgf(LT_LAMBDA,1,"Completed!\n");
#endif
  }
}

// Uncomment this to have the Lambda print timing loop statistics (for Lambda 0)
// #define LAMBDA_SPEED_CHECK

// ** LAMBDA EXECUTION THREAD **
void *lam_thread(void *arg){
  int I = *(int *)arg;
  int y=0;
  int sleeps=0;
  uint64_t delays=0;
  struct timespec start_time,this_time;
  uint64_t reference_time = 0;
  uint64_t current_time = 0;
  uint64_t elapsed_wall_time = 0;
  uint64_t elapsed_run_time = 0;
  struct timespec sleep_time;
  sleep_time.tv_sec = 0;
  clock_gettime(CLOCK_MONOTONIC,&start_time); // Initialize
  reference_time = (((uint64_t)start_time.tv_sec*1000000000)+start_time.tv_nsec);
  // printf("%ld SECONDS, %ld NSEC\n",start_time.tv_sec,start_time.tv_nsec);
  // Lambda runs at 5MHz
  // NB: ITIMER LD RAN AT 1/10TH OF A SECOND PER ITERATION
  while(ld_die_rq == 0){
    int x = 0;
    while(ld_die_rq == 0 &&
	  (pS[I].cpu_die_rq == 0 && pS[I].ConReg.Enable_SM_Clock == 1 && pS[I].PMR.Allow_UInst_Clocks != 0) &&
	  x < 83334){
      int pulse = pS[I].SM_Clock_Pulse;
      // Check for SM clock pulses
      if(pulse > 0){
	printf("LAMBDA RUN THREAD %d NEEDS %d SM CLOCKS\n",I,pulse);
	sm_clock_pulse(I,pS[I].PMR.Debug_Clock,pS[I].SM_Old_PMR);
	pS[I].SM_Clock_Pulse--;
      }else{
	lambda_clockpulse(I);
      }
      x++;
    }
    if(!(pS[I].cpu_die_rq == 0 && pS[I].ConReg.Enable_SM_Clock == 1 && pS[I].PMR.Allow_UInst_Clocks != 0)){
      // We aren't enabled, so sleep here.
      printf("LAMBDA THREAD %d GOING TO SLEEP\n",I);
      // Clobber the delta timer
      pS[I].delta_time = 0;
      // Sleep.
      while(ld_die_rq == 0 &&
	    !(pS[I].cpu_die_rq == 0 && pS[I].ConReg.Enable_SM_Clock == 1 && pS[I].PMR.Allow_UInst_Clocks != 0)){
	// Check for SM clock pulses.
	int pulse = pS[I].SM_Clock_Pulse;
	// If we have one, do it
	while(pulse > 0){
	  // printf("LAMBDA THREAD %d GENERATING %d SM CLOCKS\n",I,pulse);
	  sm_clock_pulse(I,pS[I].PMR.Debug_Clock,pS[I].SM_Old_PMR);
	  pS[I].SM_Clock_Pulse -= pulse;
	  sleep_time.tv_nsec = 20000;
	  nanosleep(&sleep_time,NULL); // Will we get another fast?
	  pulse = pS[I].SM_Clock_Pulse;
	  // Loop if we did!
	}
	// Otherwise wait longer.
	sleep_time.tv_nsec = 200000;
	nanosleep(&sleep_time,NULL); // Will we get another fast?
      }
      if(ld_die_rq != 0){ break; } // Bail out if dying
      // Reset reference time and start over.
      printf("LAMBDA THREAD %d WAKING UP\n",I);
      // pS[I].microtrace = 1; // For funsies
      clock_gettime(CLOCK_MONOTONIC,&start_time);
      reference_time = (((uint64_t)start_time.tv_sec*1000000000)+start_time.tv_nsec);
      continue;
    }
    if(ld_die_rq != 0){ break; } // Bail out if dying
    // 1 MHz = 0.001 cycles per nanosecond.
    // 16666667
    // elapsed_run_time += 100000000;
    elapsed_run_time += 16666667;
    // How long?
    clock_gettime(CLOCK_MONOTONIC,&this_time);
    current_time = (((uint64_t)this_time.tv_sec*1000000000)+this_time.tv_nsec);
    while(current_time < (reference_time+16666667)){
      // int delay = (((reference_time+16666667)-current_time)/1500);
      int64_t delay = (((reference_time+16666667)-current_time)/2);
      // Don't try to sleep if we are too far behind
      if(pS[I].delta_time > 10){ break; }
      if(delay < 1){ break; }
      delays += delay;
      // usleep(delay);
      sleep_time.tv_nsec = delay;
      nanosleep(&sleep_time,NULL); // Will we get another fast?
      sleeps++;
      clock_gettime(CLOCK_MONOTONIC,&this_time);
      current_time = (((uint64_t)this_time.tv_sec*1000000000)+this_time.tv_nsec);
    }
    // printf("%llu -> %llu\n",reference_time,current_time);
    // printf("LOOP %d DONE IN %llu ns, %d sleeps\n",y,current_time-reference_time,sleeps);
    reference_time = current_time;
    y++;
    if(y == 10){
      // Compute total skew
      int64_t clock_skew;
      elapsed_wall_time = (((uint64_t)this_time.tv_sec*1000000000)+this_time.tv_nsec);
      elapsed_wall_time -= (((uint64_t)start_time.tv_sec*1000000000)+start_time.tv_nsec);
      // clock_skew = (elapsed_run_time-elapsed_wall_time);
      clock_skew = elapsed_wall_time-elapsed_run_time;
#ifdef LAMBDA_SPEED_CHECK
      if(I == 0){
	printf("ELAPSED RUN TIME:  %llu\nELAPSED WALL TIME: %llu\n",
	       elapsed_run_time,elapsed_wall_time);
	printf("CLOCK SKEW:        %lld\nSLEEPS:            %d\nDELAYS:            %lld\n",
	       clock_skew,sleeps,delays);
      }
#endif
      sleeps = 0;
      delays = 0;
      // Update delta timer (in deciseconds)
      pS[I].delta_time = clock_skew/100000000;
      // If clock skew is positive, REAL TIME is ahead of RUN TIME, so we want to allow more RUN TIME.
      if(clock_skew > 2000){
	uint64_t clocks = (clock_skew/200);
#ifdef LAMBDA_SPEED_CHECK
	if(I == 0){
	  printf("NEED TO MAKE UP %lld CLOCKS\n",clocks);
	}
#endif
	// Cap make-up clocks at one frame
	if(clocks > 83334){
	  clocks = 83334;
	}
	elapsed_run_time += (clocks*200);
	while(clocks > 0){
	  lambda_clockpulse(I);
	  clocks--;
	}
      }
      y = 0;
    }
  }
  // If we somehow still have the bus master mutex, release it.
  if(pS[I].NUbus_Master == 1){
    release_nubus_mastership();
  }
  // Done.
  logmsgf(LT_LAMBDA,0,"LAMBDA THREAD %d TERMINATING\n",I);
  pthread_exit(NULL);
}
