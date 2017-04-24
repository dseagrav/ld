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
#include "lambda_cpu.h"
#include "nubus.h"
#include "mem.h"
#include "sdu.h"
#include "syms.h"

// FIXME: Remove the #define and the conditionals later
#define ISTREAM

// Configuration PROM message
static uint8_t prom_string[0x12] = "LMI LAMBDA V5.0";
static uint8_t prom_modelno_string[0x12] = "LAM001 V5.0";

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

#ifdef SHADOW
// Shadow memory maintenance
void shadow_write(uint32_t addr,Q data){
  ShadowMemory[addr] = data;
}

Q shadow_read(uint32_t addr){
  return(ShadowMemory[addr]);
}
#endif

// Utility functions
void debug_disassemble_IR(int I);

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
  uint32_t x=0;
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
    printf("SHIFTER: LMI = 040, GENERATED LEFT MASK 0x");
    writeH32(shift_left_mask[left_mask_index]);
    printf("\n");
  }
  */

  // Final mask
  Mask = shift_left_mask[left_mask_index]&shift_right_mask[right_mask_index];

  if(pS[I].microtrace){
    printf("SHIFTER: LMI = %d RMI = %d SLM = 0x%X SRM = 0x%X MASK = 0x%X\n",
	   left_mask_index,right_mask_index,
	   shift_left_mask[left_mask_index],shift_right_mask[right_mask_index],
	   Mask);
  }

  // Merge A with R, using bits from R if the mask bit is 1
  pS[I].Obus = 0;
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
  // SAVE FOR LATER ABUSE
  pS[I].Rbus = R;
  if(pS[I].microtrace){
    printf("SHIFTER: COMPLETED! O = 0x%X\n",pS[I].Obus);
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

void disassemble(uint64_t inst){
  disassembleWork(stdout, inst, 0);
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
  printf("[%s (%o)]\n\t",symloc,pS[I].loc_ctr_cnt);
  disassemble(pS[I].Iregister.raw);
  printf("\n");
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
  printf("[%s (%o) A=%X M=%X O=%X]\n\t",symloc,
	 pS[I].loc_ctr_cnt,pS[I].Abus,pS[I].Mbus,pS[I].Obus);
  disassemble(pS[I].Iregister.raw);
  printf("\n");
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
  printf("[%s (%o) A=%X M=%X O=%X]\n\t",symloc,
	 debugtrace_reg[I][x][0],
	 debugtrace_reg[I][x][2],
	 debugtrace_reg[I][x][3],
	 debugtrace_reg[I][x][4]);
  disassemble(debugtrace_ir[I][x]);
  printf("\n");

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
  // Reset flags
  pS[I].popj_after_nxt = -1;
  pS[I].slow_dest = false;
  pS[I].long_inst = false;
  pS[I].macro_dispatch_inst = -1;
  pS[I].exec_hold = false;
  pS[I].NOP_Next = false;
  pS[I].mirInvalid = 0;
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
      // printf("DFSTEP: NEGATIVE DIVISOR\n");
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
      // printf("DSTEP: NEGATIVE DIVISOR\n");
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
      // printf("RSTEP: NEGATIVE DIVISOR\n");
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
    printf("Unknown ALU Operation %o\n",pS[I].Iregister.ALU.Operation);
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
    
    if(pS[I].Iregister.ALU.Mask){ printf("MASKER: BIT 8 SET\n"); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.ALU.Output&0x040){ printf("MASKER: BIT 11 SET\n"); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.ALU.Spare&0x040){ printf("MASKER: BIT 29 SET\n"); pS[I].cpu_die_rq = 1; }
    
    if(pS[I].Iregister.ALU.Operation != 0){	
      // printf("MASKER: Masker PROM unimplemented\n");
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
    printf("Unknown output select %o\n",pS[I].Iregister.ALU.Output);
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
      printf("Unknown Q control %o\n",pS[I].Iregister.ALU.QControl);
      pS[I].cpu_die_rq = 1;
    }
  }
}

// Virtual memory mapping process 
void VM_resolve_address(int I,int access,int force){
  if(pS[I].microtrace){
    printf("VM: Access %o Force %o: VMA = 0x%X\n",
	   access,force,pS[I].VMAregister.raw);
    printf("VM: LV1 ENT = 0x%X (LV2 Block %o MB %o MB-Validity %o)\n",
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

  // Print LV2 data
  if(pS[I].microtrace){
    printf("VM: LV2 CTL ENT = 0x%X (Meta %o Status %o Access %o Force-Allowed %o Packet_Code %o Packetize-Writes %o Enable-Cache %o Lock-Nubus %o)\n",
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].raw,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Meta,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Access,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Force_Allowed,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packet_Code,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packetize_Writes,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Cache_Permit,
	   pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Lock_NUbus);
    printf("VM: LV2 ADR ENT 0x%X = 0x%X (PPN %o Byte_Code %o)\n",
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
      printf("VM: No-access page fault\n");
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
	printf("VM: No-read-access page fault\n");
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
	printf("VM: No-write-access page fault\n");
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
     pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Status != 06){ printf(" (VM-STA-STOP) \n"); pS[I].cpu_die_rq = 1; }
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
  if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packet_Code != 0 && pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].Byte_Code != 0){
    if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packet_Code != 1){
      printf("UNEXPECTED PACKET CODE %o\n",pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packet_Code);
      pS[I].cpu_die_rq = 1;
    }
    pS[I].vm_byte_mode = 2; // Byte access
  }

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
    printf("VM: Resulting PA 0x%X\n",pS[I].vm_phys_addr.raw);
  }
}

// Source fetch handling
void handle_source(int I,int source_mode){
  // Handle A Bus Input
  pS[I].Abus = pS[I].Amemory[pS[I].Iregister.ASource];
  // Handle M Bus Input
  if(pS[I].Iregister.MSource > 077){
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
        printf("MACRO-IR-DISPLACEMENT: LC = 0x%X, MIR = 0x%X, Fetched 0x%X\n",
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
        printf("MACRO-IR: LC = 0x%X, MIR = 0x%X, Fetched 0x%X\n",
	       pS[I].LCregister.raw,pS[I].MIregister.raw,pS[I].Mbus);
      }
      break;
    case 0104: // LAM-M-SRC-MACRO.IR.DECODE.RAM
      {
        // If Enable_Misc_MID is set, we dispatch on the whole MISC field of the instruction, if it is a MISC instruction to destination ignore (0).
        // MID memory 6000 - 7777 is used to hold dispatch addresses for MISC (6000-6777) and MISC1 (7000 - 7777).
        // So what we are doing is conditionally setting the high bits if this is an appropriate MISC op.
        if(pS[I].RG_Mode.Enable_Misc_MID != 0){ printf("EMM\n"); pS[I].cpu_die_rq = 1; }
        if(pS[I].MIregister.mi[0].raw != pS[I].MIregister.mi[1].raw){ printf("MIR\n"); pS[I].cpu_die_rq = 1; }
        // Generate address
        pS[I].MIDAddr.Opcode = pS[I].MIregister.mi[0].Misc.Opcode;
        pS[I].MIDAddr.Hi = pS[I].RG_Mode.MID_Hi_Adr;
        // Perform read
        pS[I].Mbus = pS[I].MIDmemory[pS[I].MIDAddr.raw];
        if(pS[I].microtrace){
          printf("MI: LAM-M-SRC-MACRO.IR.DECODE.RAM\n");
          printf("MID READ: MIR = 0x%X LC = 0x%X RG.Hi = %o RG.Enable_Misc_MID = %o (OPCODE %o) GENERATED ADDR %o DATA = 0x%X\n",
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
          printf("PJAN armed with a MICRO-STACK-POP\n");
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
        printf("VM: READ LV2 CTL ENT 0x%X\n",pS[I].vm_lv2_index.raw);
      }
      break;
    case 0125: // LAM-M-SRC-L2-MAP-PHYSICAL-PAGE
      // Addressed by MD!
      pS[I].vm_lv2_index.raw = 0;
      pS[I].vm_lv2_index.VPage_Offset = pS[I].MDregister.VM.VPage_Offset;
      pS[I].vm_lv2_index.LV2_Block = pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].LV2_Block;
      pS[I].Mbus = pS[I].vm_lv2_adr[pS[I].vm_lv2_index.raw].raw;
      if(pS[I].microtrace){
        printf("VM: READ LV2 ADR ENT 0x%X\n",pS[I].vm_lv2_index.raw);
      }
      break;
    case 0126: // LAM-M-SRC-LC
      pS[I].Mbus = pS[I].LCregister.raw;
      if(pS[I].microtrace || pS[I].macrotrace){
        printf("MI: LC READ: LC = 0x%X (0%o)\n",pS[I].LCregister.raw,pS[I].LCregister.raw);
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
        printf("MI: C-PDL-BUFFER-POINTER-POP: Addr Hi = 0x%X, NEW PTR = %o, DATA = %o\n",
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
        printf("MI: C-PDL-BUFFER-INDEX: Addr Hi = %X, INDEX = %o, DATA = %o\n",
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

        if(pS[I].RG_Mode.Enable_Misc_MID == 0){ printf("EMM-OFF\n"); pS[I].cpu_die_rq = 1; }
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
          printf("MI: MACRO-IR-DISPATCH-MISC\n");
          printf("MID: MIR = 0x%X LC = 0x%X RG.Hi = %o RG.Enable_Misc_MID = %o: GENERATED ADDR %o DATA = 0x%X\n",
		 pS[I].MIregister.raw,pS[I].LCregister.raw,pS[I].RG_Mode.MID_Hi_Adr,
		 pS[I].RG_Mode.Enable_Misc_MID,addr,disp_word.raw);
          printf(" DEST ");
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
              printf("%s",symloc);
            }
            printf(" (%o)",disp_word.PC);
          }
          printf("\n");
        }
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
        printf("MI: C-PDL-BUFFER-POINTER: Addr Hi = 0x%X, INDEX = %o, DATA = %o\n",
	       pS[I].DP_Mode.PDL_Addr_Hi,pS[I].pdl_index_reg,pS[I].Mbus);
      }
      break;
    case 0177: // Unknown, used by lambda-diag
      pS[I].Mbus = 0;
      break;
    default:
      printf("Unknown MF-Source %o\n",pS[I].Iregister.MSource);
      pS[I].cpu_die_rq = 1;
    }
  }else{
    pS[I].Mbus = pS[I].Mmemory[pS[I].Iregister.MSource];
  }
  // Done! Load MFO bus from M bus (for source cycle)
  pS[I].MFObus = pS[I].Mbus;
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
	  printf("MI: LC SET: NEW LC = 0x%X (0%o), need-fetch set\n",
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
	// if(pS[I].RG_Mode.MID_Hi_Adr != 0 && pS[I].RG_Mode.MID_Hi_Adr != 1){ printf("MIHI\n"); pS[I].cpu_die_rq = 1; }
	if(pS[I].RG_Mode.Enable_Misc_MID != 0){ printf("EMM\n"); pS[I].cpu_die_rq = 1; }
	if(pS[I].MIregister.mi[0].raw != pS[I].MIregister.mi[1].raw){ printf("MIR\n"); pS[I].cpu_die_rq = 1; }
	// Generate address
	pS[I].MIDAddr.Opcode = pS[I].MIregister.mi[0].Misc.Opcode;
	pS[I].MIDAddr.Hi = pS[I].RG_Mode.MID_Hi_Adr;
	// Perform write
	pS[I].MIDmemory[pS[I].MIDAddr.raw] = pS[I].Obus;
	// Log it
	if(pS[I].microtrace){
	  printf("MID WRITE: MIR = 0x%X RG.Hi = %o RG.Enable_Misc_MID = %o (OPCODE %o) GENERATED ADDR %o DATA = 0x%X\n",
		 pS[I].MIregister.raw,pS[I].RG_Mode.MID_Hi_Adr,pS[I].RG_Mode.Enable_Misc_MID,
		 pS[I].MIregister.mi[0].Misc.Opcode,pS[I].MIDAddr.raw,pS[I].Obus);
	}
	break;
      case 006: // LAM-FUNC-DEST-CRAM-HIGH
	// Same trick as LAM-FUNC-DEST-CRAM-MAP?
	// Addressed by the previous instruction, so loc_ctr_reg
	if(pS[I].microtrace){
	  printf("CRAM WRITE HI: Addr %o w/ data %o\n",pS[I].loc_ctr_reg.raw,pS[I].Obus);
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
	  printf("CRAM WRITE LO: Addr %o w/ data %o\n",pS[I].loc_ctr_reg.raw,pS[I].Obus);
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
	  printf("MI: C-PDL-BUFFER-POINTER: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
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
	  printf("MI: C-PDL-BUFFER-POINTER-PUSH: Addr Hi = 0x%X, NEW PTR = %o, DATA = %o\n",
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
          printf("MI: C-PDL-BUFFER-INDEX: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
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
	  printf("MI: PDL-BUFFER-POINTER: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
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
	  printf("uStack[%o] = %s (%o)\n",pS[I].uPCS_ptr_reg,symloc,pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
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
	  nubus_io_request(pS[I].vm_byte_mode|VM_READ,pS[I].NUbus_ID,pS[I].vm_phys_addr.raw,0);
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
	  nubus_io_request(pS[I].vm_byte_mode|VM_WRITE,pS[I].NUbus_ID,pS[I].vm_phys_addr.raw,pS[I].MDregister.raw);
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
          printf("VM: WRITE LV1 ENT 0x%X DATA 0x%X RESULT 0x%X (Meta %o Validity %o LV2_Block %o)\n",
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
	  printf("LV2 CTL BYTE CODE WRITE\n"); 
	  pS[I].cpu_die_rq = 1;
	}
	*/
	if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Packetize_Writes){ printf("LV2 CTL PACKETIZED WRITE\n"); pS[I].cpu_die_rq = 1; }
	if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Lock_NUbus){ printf("LV2 CTL LOCK NUBUS\n"); pS[I].cpu_die_rq = 1; }
	if(pS[I].vm_lv2_ctl[pS[I].vm_lv2_index.raw].Unused != 0){
	  printf("LV2 CTL UNUSED WRITE\n"); 
	  pS[I].cpu_die_rq = 1;
	}
	// Invert the meta bits?
        if(pS[I].microtrace || pS[I].cpu_die_rq == 1){
	  printf("VM: WRITE LV2 CTL ENT 0x%X (Meta %o Status %o Access %o Force-Allowed %o Packet-Code %o Packetize-Writes %o Enable-Cache %o Lock-Nubus %o Unused %o)\n",
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
	  printf("CRAM MAP WRITE: Addr %o w/ data %o\n",
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
	  nubus_io_request(pS[I].vm_byte_mode|VM_WRITE,pS[I].NUbus_ID,pS[I].vm_phys_addr.raw,pS[I].MDregister.raw);
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
          printf("MI: C-PDL-INDEX-INC: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
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
          printf("MI: C-PDL-INDEX-DEC: Addr Hi = 0x%X, PTR = %o, DATA = %o\n",
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
	    printf("MULTIPLIER: INPUT = 0x%X, X = 0x%X, Y = 0x%X, FT = 0x%X, OUT = 0x%X\n",
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
	  printf("RG Mode = %o (",pS[I].RG_Mode.raw);
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control != 0){ printf("Aux_Stat_Count_Control:%o ",pS[I].RG_Mode.Aux_Stat_Count_Control); }
	  if(pS[I].RG_Mode.Aux_Stat_Count_Clock != 0){ printf("Aux_Stat_Count_Clock:%o ",pS[I].RG_Mode.Aux_Stat_Count_Clock); }
	  if(pS[I].RG_Mode.NUbus_ID != 0){ printf("NUbus_ID:%o ",pS[I].RG_Mode.NUbus_ID); }
	  if(pS[I].RG_Mode.Need_Macro_Inst_Fetch != 0){ printf("Need_Macro_Inst_Fetch:%o ",pS[I].RG_Mode.Need_Macro_Inst_Fetch); }
	  if(pS[I].RG_Mode.Main_Stat_Count_Control != 0){ printf("Main_Stat_Count_Control:%o ",pS[I].RG_Mode.Main_Stat_Count_Control); }
	  if(pS[I].RG_Mode.Aux_Stat_Count_Control_2 != 0){ printf("Aux_Stat_Count_Control_2:%o ",pS[I].RG_Mode.Aux_Stat_Count_Control_2); }
	  if(pS[I].RG_Mode.Main_Stat_Clock_Control != 0){ printf("Main_Stat_Clock_Control:%o ",pS[I].RG_Mode.Main_Stat_Clock_Control); }
	  if(pS[I].RG_Mode.Enable_Misc_MID != 0){ printf("Enable_Misc_MID:%o ",pS[I].RG_Mode.Enable_Misc_MID); }
	  if(pS[I].RG_Mode.Sequence_Break != 0){ printf("Sequence_Break:%o ",pS[I].RG_Mode.Sequence_Break); }
	  if(pS[I].RG_Mode.Interrupt_Enable != 0){ printf("Interrupt_Enable:%o ",pS[I].RG_Mode.Interrupt_Enable); }
	  if(pS[I].RG_Mode.MID_Hi_Adr != 0){ printf("MID_Hi_Adr:%o ",pS[I].RG_Mode.MID_Hi_Adr); }
	  if(pS[I].RG_Mode.VA_Mode != 0){ printf("VA_Mode:%o ",pS[I].RG_Mode.VA_Mode); }
	  if(pS[I].RG_Mode.Single_Step_Macro_Inst != 0){ printf("Single_Step_Macro_Inst:%o ",pS[I].RG_Mode.Single_Step_Macro_Inst); }
	  printf(")\n");
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
	  printf("MI: PDL-BUFFER-INDEX: Addr Hi = 0x%X, INDEX = %o, DATA = %o\n",
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
	  nubus_io_request(pS[I].vm_byte_mode|VM_READ,pS[I].NUbus_ID,pS[I].vm_phys_addr.raw,0);
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
	  printf("VM: LV2 ADR ENT 0x%X = 0x%X\n",pS[I].vm_lv2_index.raw,pS[I].Obus);
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
	  nubus_io_request(pS[I].vm_byte_mode|VM_WRITE,pS[I].NUbus_ID,pS[I].vm_phys_addr.raw,pS[I].MDregister.raw);
	}
	break;
      default:
	printf("Unknown F-Dest %o\n",pS[I].Iregister.Destination.F.Dest);
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
          printf("RG: SPY: CSM Write, addr 0x%X, data 0x%X\n",pS[I].CSM_Adr.Addr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].CSMRAM[pS[I].CSM_Adr.Addr].raw;
          printf("RG: SPY: CSM Read, addr 0x%X, data 0x%X\n",pS[I].CSM_Adr.Addr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
	if(NUbus_Request == VM_BYTE_WRITE){
          pS[I].CSMRAM[pS[I].CSM_Adr.Addr].byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          printf("RG: SPY: CSM Byte Write, addr 0x%X, data 0x%X\n",pS[I].CSM_Adr.Addr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].CSMRAM[pS[I].CSM_Adr.Addr].byte[NUbus_Address.Byte];
          printf("RG: SPY: CSM Byte Read, addr 0x%X, data 0x%X\n",pS[I].CSM_Adr.Addr,NUbus_Data.byte[NUbus_Address.Byte]);
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
          printf("RG: SPY: CSM.ADR Write, data 0x%X\n",pS[I].CSM_Adr.Addr);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].CSM_Adr.raw;
          printf("RG: SPY: CSM.ADR Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          uint32_t Word = pS[I].CSM_Adr.raw;
          Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          printf("RG: SPY: CSM.ADR Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;	

      case 002: // CSM.REG
	// Output register of CSM. Read-only.
	if(NUbus_Request == VM_WRITE){
          printf("RG: SPY: CSM.REG Write, data discarded\n");
          NUbus_acknowledge = 1;
          return;
        }
	if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].CSM_Output;
          printf("RG: SPY: CSM.REG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        break;

      case 004: // HIGH IREG
	if(NUbus_Request == VM_WRITE){
          pS[I].Iregister.word[1] = NUbus_Data.word;
          printf("RG: SPY: HI IREG Write, data 0x%X\n",NUbus_Data.word);
	  printf("DISASSEMBLY OF WRITTEN UI:\n");
	  disassemble_IR(I);
          NUbus_acknowledge = 1;
	  pS[I].spy_wrote_ireg = true;
	  // SOURCE CYCLE FETCH HAPPENS HERE!
	  handle_source(I,1);
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          pS[I].Iregister.byte[4+NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          printf("RG: SPY: HI IREG Byte Write, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
	  pS[I].spy_wrote_ireg = true;
	  if(NUbus_Address.Byte == 3){
	    printf("DISASSEMBLY OF WRITTEN UI:\n");
	    disassemble_IR(I);
	    // SOURCE CYCLE FETCH HAPPENS HERE!
	    handle_source(I,1);
	  }
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].Iregister.word[1];
          printf("RG: SPY: HI IREG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].Iregister.byte[4+NUbus_Address.Byte];
          printf("RG: SPY: HI IREG Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 005: // LO IREG
	if(NUbus_Request == VM_WRITE){
          pS[I].Iregister.word[0] = NUbus_Data.word;
          printf("RG: SPY: LO IREG Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
	  pS[I].spy_wrote_ireg = true;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          pS[I].Iregister.byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          printf("RG: SPY: LO IREG Byte Write, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
	  pS[I].spy_wrote_ireg = true;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].Iregister.word[0];
          printf("RG: SPY: LO IREG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].Iregister.byte[NUbus_Address.Byte];
          printf("RG: SPY: HI IREG Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
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
          printf("RG: SPY: HI CRAM Write, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          NUbus_Data.word = pS[I].WCS[paddr].word[1];
          printf("RG: SPY: HI CRAM Read, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          pS[I].WCS[paddr].byte[4+NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          printf("RG: SPY: HI CRAM Byte Write, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].WCS[paddr].byte[4+NUbus_Address.Byte];
          printf("RG: SPY: HI CRAM Byte Read, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.byte[NUbus_Address.Byte]);
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
          printf("RG: SPY: LO CRAM Write, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          NUbus_Data.word = pS[I].WCS[paddr].word[0];
          printf("RG: SPY: LO CRAM Read, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          pS[I].WCS[paddr].byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
          printf("RG: SPY: LO CRAM Byte Write, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          int addr = pS[I].loc_ctr_reg.raw&0xFFFF;
          int paddr = pS[I].CRAM_map[addr>>4] & 03777;
          paddr <<= 4;
          paddr |= (addr&0xF);
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].WCS[paddr].byte[NUbus_Address.Byte];
          printf("RG: SPY: LO CRAM Byte Read, addr 0x%X, paddr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,paddr,NUbus_Data.byte[NUbus_Address.Byte]);
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
	  printf("RG: SPY: TRAM.ADR Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = (pS[I].TRAM_Adr&0x0FFF);
          printf("RG: SPY: TRAM.ADR Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
	  uint32_t Word = (pS[I].TRAM_Adr&0x0FFF);
          Word >>= (8*NUbus_Address.Byte);
	  NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          printf("RG: SPY: TRAM.ADR Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 011: // TRAM
        if(NUbus_Request == VM_WRITE){
          pS[I].TRAM[pS[I].TRAM_Adr].word = NUbus_Data.word;
          printf("RG: SPY: TRAM Write, addr 0x%X, data 0x%X\n",pS[I].TRAM_Adr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_WRITE){
          pS[I].TRAM[pS[I].TRAM_Adr].byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
	  printf("RG: SPY: TRAM Byte Write, addr 0x%X, data 0x%X\n",pS[I].TRAM_Adr,NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].TRAM[pS[I].TRAM_Adr].word;
	  printf("RG: SPY: TRAM Read, addr 0x%X, data 0x%X\n",pS[I].TRAM_Adr,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].TRAM[pS[I].TRAM_Adr].byte[NUbus_Address.Byte];
	  printf("RG: SPY: TRAM Byte Read, addr 0x%X, data 0x%X\n",pS[I].TRAM_Adr,NUbus_Data.byte[NUbus_Address.Byte]);
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
          printf("RG: SPY: HRAM Write, addr 0x%X, data 0x%X\n",pS[I].History_Pointer&0xFFF,NUbus_Data.word);
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
	  printf("RG: SPY: HRAM Read, addr 0x%X, data 0x%X\n",pS[I].History_Pointer&0xFFF,NUbus_Data.word);
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
	  printf("RG: SPY: HPTR Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = (pS[I].History_Pointer&0x0FFF);
          printf("RG: SPY: HPTR Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
	if(NUbus_Request == VM_BYTE_READ){
	  uint32_t Word = (pS[I].History_Pointer&0x0FFF);
          Word >>= (8*NUbus_Address.Byte);
	  NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
	  printf("RG: SPY: HPTR Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
	  NUbus_acknowledge=1;
	  return;
	}
	break;

      case 014: // PC (read-only)
	if(NUbus_Request == VM_WRITE){
          printf("RG: SPY: PC Write, data discarded");
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
	  // If we have a pending PC, take that instead
          printf("RG: SPY: PC Read, ");
	  if(pS[I].loc_ctr_nxt != -1){
	    printf("Nxt");
	    NUbus_Data.word = pS[I].loc_ctr_nxt;
	  }else{
	    if(pS[I].TRAM_PC == 01005){
	      printf("Reg+1");
	      NUbus_Data.word = (pS[I].loc_ctr_reg.raw+1);	      
	    }else{
	      printf("Reg");
	      NUbus_Data.word = pS[I].loc_ctr_reg.raw;
	    }
	  }
	  printf(" = data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
	  uint32_t Word = 0;
          printf("RG: SPY: PC Byte Read, ");
          if(pS[I].loc_ctr_nxt != -1){
            printf("Nxt");
            Word = pS[I].loc_ctr_nxt;
          }else{
            if(pS[I].TRAM_PC == 01005){
              printf("Reg+1");
              Word = (pS[I].loc_ctr_reg.raw+1);
            }else{
              printf("Reg");
              Word = pS[I].loc_ctr_reg.raw;
            }
          }
	  printf(" = 0%o",Word);
	  Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          printf(", data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        break;

      case 015: // TREG (Output register of TRAM, read-only)
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].TREG.word;
          printf("RG: SPY: TREG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          NUbus_Data.byte[NUbus_Address.Byte] = pS[I].TREG.byte[NUbus_Address.Byte];
          printf("RG: SPY: TREG Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
	break;

      case 016: // MFO ENABLE (reading MFO bus)
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].MFObus;
          printf("RG: SPY: MFO BUS Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          uint32_t Word = pS[I].MFObus;
          Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          printf("RG: SPY: MFO BUS Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
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
	  printf("RG: SPY: SPY REG Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].SPY_Register;
          printf("RG: SPY: SPY REG Read, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          uint32_t Word = pS[I].SPY_Register;
          Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          printf("RG: SPY: SPY REG Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
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
	  printf("RG: SPY: PMR Write, data 0x%X\n",Word);
	  NUbus_acknowledge = 1;
	  // HANDLE BITS
	  if(pS[I].PMR.Clear_NOP != 0){
	    printf("RG: PMR: Clear NOP\n");
	    pS[I].ConReg.nop = 0;
	  }
	  if(pS[I].PMR.Reset_Interrupt_Counter != 0){
	    int x=0;
	    printf("RG: PMR: Reset Interrupt Counter\n");
	    pS[I].InterruptPending = 0;
	    while(x<0x100){
	      pS[I].InterruptStatus[x] = 0;
	      x++;
	    }
	    pS[I].InterruptVector = 0;	    
	  }
	  if(pS[I].PMR.Force_T_Hold != 0){
	    if(oldPMR.Force_T_Hold == 0){
	      printf("RG: PMR: FORCE-T-HOLD SET\n");
	      pS[I].exec_hold = true;
	      pS[I].ConReg.t_hold_l = 0; // Holding
	    }
	  }else{
	    if(oldPMR.Force_T_Hold != 0){
	      printf("RG: PMR: FORCE-T-HOLD CLEARED\n");
	      pS[I].exec_hold = false;
	      pS[I].ConReg.t_hold_l = 1; // Not Holding	      
	    }
	  }
	  // If CONREG Enable_SM_Clock is clear, the SM clock is controlled by PMR Debug_Clock
	  if(pS[I].ConReg.Enable_SM_Clock == 0){	    
	    if(pS[I].PMR.Debug_Clock == 0){
	      if(oldPMR.Debug_Clock != 0){
		printf("RG %d: PMR: SM CLOCK FALLING EDGE: TRAM_PC 0%o\n",I,pS[I].TRAM_PC);
		// Ensure stopped
		// pS[I].cpu_die_rq = 1;
		// If we are writing TRAM_PC, do so
		if(pS[I].PMR.Spy_Address_TRAM_L == 0 && oldPMR.Spy_Address_TRAM_L != 0){
		  pS[I].TRAM_PC = (pS[I].TRAM_Adr&0xFFF);
		  printf("TREG %d: PMR.Spy_Address_TRAM_L: %o\n",I,pS[I].TRAM_PC);
		  // pS[I].PMR.Spy_Address_TRAM_L = 1; // Do this only once?
		}
		// Update UI clock.
		if(pS[I].ConReg.uinst_clock_l != 1){
		  pS[I].ConReg.uinst_clock_l = 1;
		  printf("TREG %d: UI CLOCK FALLING EDGE\n",I);
		}
		// If Force-Hold is on, make sure hold stays lit
		if(pS[I].PMR.Force_T_Hold != 0 && pS[I].ConReg.t_hold_l != 0){ pS[I].ConReg.t_hold_l = 0; }
		// Operate ALU and shifter, load O bus
                if(pS[I].TREG.A_clock_next == 1 && pS[I].TREG.M_clock_next == 1 &&
                   pS[I].TREG.A_WE_L == 1 && pS[I].TREG.M_WE_L == 1){
                  // This is probably after the source read
                  printf("TREG %d: Operating ALU/Shifter\n",I);
		  // Switch opcode
		  switch(pS[I].Iregister.Opcode){
		  case 0: // ALU-OP
		    // Perform ALU operation
		    operate_alu(I);

		    // Operate O bus
		    handle_o_bus(I);

		    if(pS[I].Iregister.ALU.Misc > 0){ printf("ALU-MISC "); pS[I].cpu_die_rq = 1; }
		    if(pS[I].Iregister.ALU.Spare > 0){ printf("ALU-SPARE "); pS[I].cpu_die_rq = 1; }

		    // Process Q register
		    // This will get doubled! Do we need to do it?
		    // handle_q_register(I);

		    // Load MFO from O-bus
		    pS[I].MFObus = pS[I].Obus;
		    break;

		  case 1: // BYTE-OP
		    // Operate shifter. Result goes to O bus.
		    operate_shifter(I);
		    // Load MFO from O-bus
		    pS[I].MFObus = pS[I].Obus;

		    if(pS[I].Iregister.Byte.Misc > 0){ printf("Misc "); pS[I].cpu_die_rq = 1; }
		    if(pS[I].Iregister.Byte.Spare > 0){ printf("BYTE-SPARE "); pS[I].cpu_die_rq = 1; }
		    break;

		  case 2: // JUMP-OP
		    printf("TREG: JUMP-OP\n");
		    break;
		  case 3: // DISPATCH-OP
		    printf("TREG: DISPATCH-OP\n");
		    break;
		  }
		}
		// If we aren't holding...
		if(pS[I].ConReg.t_hold_l == 1){
		  // Update UI clock (again)
		  // The same SM clock which causes the source codeword to appear in TREG also raises UI clock.
		  if(pS[I].TREG.new_uinst != 0 && pS[I].PMR.Allow_UInst_Clocks != 0){
		    pS[I].ConReg.uinst_clock_l = 0;
		    printf("TREG %d: UI CLOCK RISING EDGE\n",I);
		    // Single step!
		    pS[I].cpu_die_rq = 0;
		  }
		  // Fetch TREG
		  pS[I].TREG = pS[I].TRAM[pS[I].TRAM_PC];		  
		  // Advance TRAM_PC
		  switch(pS[I].TRAM_PC){
		  default:
		    printf("TREG %d: UNKNOWN TRAM_PC 0%o\n",I,pS[I].TRAM_PC);
		    // Fall into
		  case 0: // SETZ
		  case 01000: // Used by lambda-diag
		  case 01004: // Used by lambda-diag
		  case 01005: // Used by lambda-diag for force-source codeword
		  case 03000: // DISPATCH TO EXECUTE PHASE
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
		      printf("TREG %d: Unknown NEXT-SELECT %o\n",I,pS[I].TREG.next_select);
		      break;
		    }
		    printf("TREG %d: NEXT TREG %o\n",I,pS[I].TRAM_PC);
		    break;
		  }
		}else{
		  printf("TREG %d: T-HOLD\n",I);
		}
		// If we did not write TRAM_PC, write it back to ADR
		if(pS[I].PMR.Spy_Address_TRAM_L == 1){
		  pS[I].TRAM_Adr = pS[I].TRAM_PC;
		}
	      }else{
		printf("RG %d: PMR: SM CLOCK LOW: TRAM_PC 0%o\n",I,pS[I].TRAM_PC);
		// Ensure stopped
		pS[I].cpu_die_rq = 1;
	      }
	    }else{
	      if(oldPMR.Debug_Clock == 0){
		// This is a SM CLOCK.
		// A normal instruction happens in two of these, one of which must be 03000.
		// 03000 is a source phase
		printf("RG %d: PMR: SM CLOCK RISING EDGE: TRAM_PC 0%o\n",I,pS[I].TRAM_PC);
		// Fool the CSMRAM data path test
		if(pS[I].CSM_Adr.Addr == 0xfff){
		  printf("RG: CSM Output Reg Loaded from CSMRAM\n");
		  pS[I].CSM_Output = pS[I].CSMRAM[pS[I].CSM_Adr.Addr].raw;
		}
		// Handle source read
		if(pS[I].TREG.A_clock_next == 1 && pS[I].TREG.M_clock_next == 1 &&
		   pS[I].TREG.A_WE_L == 1 && pS[I].TREG.M_WE_L == 1){
		  // This is probably the source read
		  printf("TREG %d: Triggering source reads\n",I);
		  handle_source(I,1);
		}
	      }else{
		printf("RG %d: PMR: SM CLOCK HI: TRAM_PC 0%o\n",I,pS[I].TRAM_PC);		
	      }
	    }
	  }
	  // What about the uinst clock?
	  // If the SM clock is running or being stepped, we can run it.	  
	  /*
	  if((pS[I].PMR.Allow_UInst_Clocks != 0) &&
	     (pS[I].ConReg.Enable_SM_Clock == 1 || (pS[I].ConReg.Enable_SM_Clock == 0 && (oldPMR.Debug_Clock == 0 && pS[I].PMR.Debug_Clock == 1)))){
	    if(pS[I].PMR.Allow_UInst_Clocks != 0){
	      if(pS[I].TRAM_PC == 03001){
		// This really happens at 03000, but since we just changed it if we stepped...
		printf("RG %d: PMR: UINST CLOCK ENABLED\n",I);
		// Dummy out the "just clearing the pipeline" SETZs here.
		if(pS[I].Iregister.word[0] == 0x00000000 && pS[I].Iregister.word[1] == 0xF0000000){
		  printf("RG: PMR: Pipeline clearing detected, not stepping\n");
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
		  // Process HPTR
		  pS[I].History_RAM[pS[I].History_Pointer&0xFFF] = pS[I].loc_ctr_cnt;
		  pS[I].History_Pointer++; 
		  if(pS[I].History_Pointer > 0xFFF){ pS[I].History_Pointer = 0; }
		}else{
		  pS[I].cpu_die_rq = 0;
		  // pS[I].microtrace = 1;
		}
	      }else{
		printf("RG: PMR: UINST CLOCK ENABLED @ TRAM_ADR 0%o, WAITING FOR 3001\n",pS[I].TRAM_PC);
		// printf(", UFETCH ARMED\n");
	      }
	    }else{
	      pS[I].ConReg.uinst_clock_l = 1; // Not Uinst
	      printf("RG %d: PMR: UINST CLOCK DISABLED\n",I);
	      pS[I].cpu_die_rq = 1;
	    }
	  }
	  */
	  // UInst Advance
	  if(pS[I].PMR.Advance_UInst_Request != 0 && oldPMR.Advance_UInst_Request == 0){
	    // a transition from 0 to 1 causes a single uinst step, or tries to restart the lambda if it is in a halted state
	    printf("RG %d: PMR: ADVANCE UINST REQUEST\n",I);
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
	  printf("RG: SPY: PMR Read, data 0x%X\n",NUbus_Data.word);
	  NUbus_acknowledge = 1;
	  return;
	}		
	if(NUbus_Request == VM_BYTE_READ){
	  uint32_t Word = (((pS[I].PMR.raw&0x00FFFFFF)<<8)|0xFF);
	  Word >>= (8*NUbus_Address.Byte);
	  NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
	  printf("RG: SPY: PMR Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
	  NUbus_acknowledge=1;
	  return;
	}
	break;	

      case 021: // PARITY VECTOR
        if(NUbus_Request == VM_WRITE){
          pS[I].Parity_Vector = NUbus_Data.word;
          printf("RG: SPY: Parity Vector Write, data 0x%X\n",NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          NUbus_Data.word = pS[I].Parity_Vector;
          printf("RG: SPY: Parity Vector Read, data 0x%X\n",NUbus_Data.word);
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
          printf("RG: SPY: CRAM ADR MAP Write, addr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_READ){
          int map_addr = ((pS[I].loc_ctr_reg.raw&0xFFFF)>>4);
          NUbus_Data.word = pS[I].CRAM_map[map_addr];
          printf("RG: SPY: CRAM ADR MAP Read, addr 0x%X, data 0x%X\n",pS[I].loc_ctr_reg.raw,NUbus_Data.word);
          NUbus_acknowledge = 1;
          return;
        }
        if(NUbus_Request == VM_BYTE_READ){
          int map_addr = ((pS[I].loc_ctr_reg.raw&0xFFFF)>>4);
          uint32_t Word = pS[I].CRAM_map[map_addr];
          Word >>= (8*NUbus_Address.Byte);
          NUbus_Data.byte[NUbus_Address.Byte] = Word&0xFF;
          printf("RG: SPY: CRAM ADR MAP Byte Read, data 0x%X\n",NUbus_Data.byte[NUbus_Address.Byte]);
          NUbus_acknowledge=1;
          return;
        }
        break;	

      }
      printf("RG: Unimplemented SPY Request %o Addr 0x%X (Reg 0%o) w/ data 0x%X (0%o)\n",
	     NUbus_Request,NUbus_Address.raw,spy_reg,NUbus_Data.word,NUbus_Data.word);
      pS[I].cpu_die_rq = 1;
    }
    break;

  case 0x400 ... 0x7FF:
    if(NUbus_Request == VM_WRITE){
      // Interrupt!
      uint8_t Vector = ((NUbus_Address.Addr>>2)&0xFF);
      if(NUbus_trace == 1){
	printf("RG: INTERRUPT RQ: Vector %o",Vector);
      }
      // Light vector bit if enabled
      if(pS[I].RG_Mode.Interrupt_Enable != 0){
	if(pS[I].InterruptStatus[Vector] != 1){
	  pS[I].InterruptStatus[Vector] = 1;
	  pS[I].InterruptPending++;
	}
	if(NUbus_trace == 1){
	  printf(" ACCEPTED");
	}
      }
      if(NUbus_trace == 1){
	printf("\n");
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
      printf("RG %d: CON REG write, data 0x%X, result 0x%X\n",
	     I,NUbus_Data.byte[0],pS[I].ConReg.word);
      // HANDLE THOSE BITS
      if(pS[I].ConReg.Init == 1){
	printf("CONREG: INIT\n");
	lambda_initialize(I,0);	
	pS[I].ConReg.Init = 0; // This makes newboot happy
      }
      if(pS[I].ConReg.Enable_SM_Clock == 1){
	printf("CONREG %d: ENABLE SM CLOCK\n",I);	
	if(pS[I].cpu_die_rq != 0){
	  if(pS[I].PMR.Allow_UInst_Clocks != 0){
	    extern int cp_state[2];
	    printf("CONREG %d: LAMBDA ENABLED\n",I);
	    pS[I].cpu_die_rq = 0;
	    // Update state
	    if(pS[I].loc_ctr_reg.raw == 036000){
	      // PROM START
	      cp_state[I] = 1;
	      printf("LAMBDA %d COLD BOOT START\n",I);
	    }else{
	      if(pS[I].loc_ctr_reg.raw == 000002){
		// LISP START
		cp_state[I] = 2;
		printf("LAMBDA %d LISP START\n",I);
		// Patch microcode bug
		printf("Patching loaded microcode...\r\n");
		pS[I].WCS[016047].ASource = 01114;    // Fix to XGCD1+2 - Add1 to Rotate Field Sub1 from Length
		// At this point, we should capture Lisp's starting state
		dump_lisp_start_state(I);
	      }
	    }	    
	  }else{
	    printf("CONREG: Lambda not enabled, PMR.Allow_UInst_Clocks is off\n");	    
	  }
	}
      }else{
	printf("CONREG %d: DISABLE SM CLOCK\n",I);	
	pS[I].cpu_die_rq = 1; // Ensure stopped
	pS[I].ConReg.LED = 1; // Not inverted - Does this force LED?
	pS[I].ConReg.uinst_clock_l = 1; // HACK HACK
      }
      NUbus_acknowledge=1;
      return;
    }
    if(NUbus_Request == VM_READ){
      NUbus_Data.word = pS[I].ConReg.word; // 0x80; // UINST-CLOCK-L-BIT
      printf("RG %d: CON REG read, data 0x%X\n",I,NUbus_Data.word);
      NUbus_acknowledge=1;
      return;
    }
  case 0xFFF7FD:
  case 0xFFF7FE:
  case 0xFFF7FF:
    if(NUbus_Request == VM_BYTE_READ){
      NUbus_Data.byte[NUbus_Address.Byte] = pS[I].ConReg.byte[NUbus_Address.Byte]; 
      printf("RG %d: CON REG byte read, data 0x%X\n",I,NUbus_Data.byte[NUbus_Address.Byte]);
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
  printf("RG: Unimplemented address: 0x%X\n",NUbus_Address.Addr);
  pS[I].cpu_die_rq = 1;  
}

// Clock Pulse
void lambda_clockpulse(int I){
  pS[I].cycle_count++;
  // Run one clock

  // NUbus interface operation
  if(NUbus_master == pS[I].NUbus_ID && NUbus_Busy > 0){
    if((NUbus_Request&0x01)==0){
      // Since this is our read request, reload MD here.
      // Since Lambda doesn't handle timeouts, I guess this happens regardless of acknowledge?
      pS[I].MDregister.raw = NUbus_Data.word;
    }
    // Was it acknowledged?
    if(NUbus_acknowledge == 1){
      if(NUbus_trace == 1){
	printf("NUBUS: Cycle Complete: Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) Ack %o\n",
	       NUbus_Request,NUbus_Address.raw,NUbus_Address.raw,NUbus_Data.word,NUbus_Data.word,NUbus_acknowledge);
      }
      // We can release the bus
      NUbus_Busy = 0;
    }
  }
  // NUbus slave operation (RG board's interface)
  // Under some conditions the Lambda will talk to itself
  if(NUbus_Busy == 2 && NUbus_acknowledge == 0 && NUbus_Address.Card == pS[I].NUbus_ID){
    lambda_nubus_slave(I);
  }

  // If we are halted, we are done here.
  if(pS[I].cpu_die_rq != 0){
    return; 
  }else{
    /*
    if(pS[I].PMR.Allow_UInst_Clocks != 0 && pS[I].ConReg.uinst_clock_l != 0){
      printf("CONREG: UINST_CLOCK\n");
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
	printf("CONREG: Hold Cleared\n");
      }
    }
    // Decrement POPJ-After-Next flag if set
    if(pS[I].popj_after_nxt > -1){
      if(pS[I].popj_after_nxt == 0){
	// Time for popj
	pS[I].loc_ctr_reg.raw = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
	pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
	if(pS[I].microtrace){
	  printf("PJAN FIRED: GOING TO %o\n",pS[I].loc_ctr_reg.raw);
	}
	// pS[I].cpu_die_rq = 1; 
      }
      pS[I].popj_after_nxt--;
    }
    
    if(pS[I].slow_dest == true){
      // Slow destination. Waste a cycle.
      pS[I].stall_count++;
      pS[I].slow_dest = false;
      if(pS[I].microtrace){
	printf("SLOW-DEST cycle used\n");
      }
      return;
    } 

    if(pS[I].long_inst == true){
      // Long instruction. Waste a cycle.
      pS[I].stall_count++;
      pS[I].long_inst = false;
      if(pS[I].microtrace){
	printf("LONG-INST cycle used\n");
      }
      return;
    } 

    if(pS[I].NOP_Next == true){
      // Inhibit bit set. Waste a cycle.
      pS[I].ConReg.nop = 0;
      pS[I].stall_count++;
      pS[I].NOP_Next = false;
      if(pS[I].microtrace){
	printf("NOP-NEXT cycle used\n");
      }
      return;
    }else{
      pS[I].ConReg.nop = 1;
    }

    if(pS[I].cram_write_cyc == true){
      // CRAM or map write. Waste a cycle.
      pS[I].stall_count++;
      pS[I].cram_write_cyc = false;
      if(pS[I].microtrace){
	printf("CRAM WRITE cycle used\n");
      }
      return;
    }

    // MACRO INSTRUCTION DISPATCH
    if(pS[I].macro_dispatch_inst > -1){
      pS[I].macro_dispatch_inst--;
      if(pS[I].macro_dispatch_inst == 0){
	DispatchWord disp_word;
	if(pS[I].microtrace){
	  printf("MI: MACRO DISPATCH CYCLE\n");
	}
	// Stop conditions
	if(pS[I].RG_Mode.Enable_Misc_MID == 0){ printf("EMM-OFF\n"); pS[I].cpu_die_rq = 1; }
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
	  printf("MISC/MISC1 INSTRUCTION!\n");
	  printf("MI: MACRO DISPATCH\n");
          printf("MID: MIR = 0x%X LC = 0x%X RG.Hi = %o RG.Enable_Misc_MID = %o Opcode %o: GENERATED ADDR %o DATA = 0x%X\n",
		 pS[I].MIregister.raw,pS[I].LCregister.raw,pS[I].RG_Mode.MID_Hi_Adr,pS[I].RG_Mode.Enable_Misc_MID,
		 pS[I].MIDAddr.Opcode,pS[I].MIDAddr.raw,disp_word.raw);
	  printf("MID: OP %s DEST ",jump_op_str[disp_word.Operation]);
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
	      printf("%s",symloc);
	    }
	    printf(" (%o)",disp_word.PC);
	  }
	  printf("\n");
	}
	if(disp_word.StartRead){ printf(" START-MEMORY-READ"); pS[I].cpu_die_rq = 1; } // Stop for investigation
	// Handle operation
	switch(disp_word.Operation){
	case 0: // Jump-Branch-Xct-Next
	  // Jump, but DO NOT inhibit the next instruction!
	  if(pS[I].loc_ctr_nxt != -1){
	    printf("MACRO DISPATCH: Pending JUMP-After-Next collision investigation stop\n");
	    pS[I].cpu_die_rq = 1;
	  }
	  pS[I].loc_ctr_nxt = disp_word.PC;
	  break;
	  
	case 1: // Jump-Branch
	  if(pS[I].loc_ctr_reg.raw != (pS[I].loc_ctr_cnt + 1)){
	    printf("MACRO DISPATCH: Pending JUMP collision investigation stop\n");
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
	    printf("DISPATCH: LPC set w/ call-xct-next?\n");
	    pS[I].cpu_die_rq = 1;
	  }
	  pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw+1; // Pushes the address of the next instruction
	  if(pS[I].microtrace){
	    char *location;
	    char symloc[100];
	    int offset;

	    printf("uStack[%o] = ",pS[I].uPCS_ptr_reg);

	    location = "";
	    offset = 0;
	    location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    printf("%s (%o)\n",symloc,pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
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

	    printf("uStack[%o] = ",pS[I].uPCS_ptr_reg);

	    location = "";
	    offset = 0;
	    location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    printf("%s (%o)\n",symloc,pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
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
	  printf("Unknown macro-dispatch RPN %o\n",disp_word.Operation);
	  pS[I].cpu_die_rq=1;
	}
	// pS[I].cpu_die_rq = 1;
	// This uses up a cycle
	if(pS[I].microtrace){
	  printf("MI: MACRO DISPATCH cycle completed\n");
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
	printf("UI: MAP 0x%X -> 0x%X\n",pS[I].loc_ctr_cnt&0xFFFF,paddr);
      }
      pS[I].Iregister.raw = pS[I].WCS[paddr].raw;
    }else{
      pS[I].spy_wrote_ireg = false;
      if(pS[I].microtrace){
	printf("UI: Execute from modified IReg\n");
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
      printf("**MInst-HALT** #%d\n",I);
      pS[I].cpu_die_rq = 1;
    }
  }else{
    // We are holding
    if(pS[I].microtrace != 0){
      printf("CONREG: ");
      if(pS[I].PMR.Force_T_Hold != 0){ printf("Forced "); }
      printf("Hold\n");
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
    if(NUbus_Busy > 0){
      if(pS[I].microtrace != 0){
	printf("LAMBDA: CMSB: Awaiting bus...\n");
      }
      pS[I].stall_count++; // Track ticks burned
      pS[I].exec_hold = true;
      return;
    }
  }
  // MD source stall handling
  if(pS[I].Iregister.MSource == 0161 && NUbus_Busy > 0 && NUbus_master == pS[I].NUbus_ID &&
     !(NUbus_acknowledge != 0 || NUbus_error != 0)){
    if(pS[I].microtrace != 0){
      printf("LAMBDA: M-SRC-MD: Awaiting cycle completion...\n");
    }
    pS[I].stall_count++; // Track ticks burned
    pS[I].exec_hold = true;
    return;
  }
  // Handle top-level flag (Raven's PJ14)
  if((pS[I].loc_ctr_nxt != -1 && (pS[I].loc_ctr_nxt&0x40000) != 0) ||
     (pS[I].loc_ctr_reg.raw&0x40000) != 0){
    if(pS[I].microtrace != 0){
      printf("TOPLEVEL FLAG SET! NOP-NEXT %o LOC-CTR-REG = 0x%X LOC-CTR-NXT = 0x%X\n",
	     pS[I].NOP_Next,pS[I].loc_ctr_reg.raw,pS[I].loc_ctr_nxt);
    }

#ifdef ISTREAM
    if(((pS[I].LCregister.raw>>1)&0x01) == 0x0){
      pS[I].RG_Mode.Need_Macro_Inst_Fetch = 1;
      if(pS[I].microtrace != 0){
	printf("ODD PHASE LIGHTS NEEDS FETCH\n");
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
	printf("NEED-FETCH IS LIT\n");
      }
#endif
      // We will need the memory bus. Can we have it?
      if(NUbus_Busy > 0 && pS[I].ConReg.Enable_NU_Master == 1){
	if(pS[I].microtrace != 0){
	  printf("AWAITING MEMORY BUS...\n");
	}
	// No, burn a cycle and come back
	pS[I].stall_count++; // Track ticks burned
	pS[I].exec_hold = true;
	return;
      }
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
	nubus_io_request(VM_READ,pS[I].NUbus_ID,pS[I].vm_phys_addr.raw,0);
      }
#ifdef ISTREAM
      // Clear needfetch
      pS[I].RG_Mode.Need_Macro_Inst_Fetch = 0;
      pS[I].mirInvalid = 1;
    }else{
      if(pS[I].microtrace != 0){
	printf("NO NEED FOR FETCHING\n");
      }
    }
#else
    // Clear needfetch
    pS[I].RG_Mode.Need_Macro_Inst_Fetch = 0;
#endif
    // Advance LC
    pS[I].LCregister.raw += 2;
    if(pS[I].RG_Mode.Aux_Stat_Count_Control == 03){
      pS[I].stat_counter_aux++;
    }
    if(pS[I].RG_Mode.Main_Stat_Count_Control == 03){
      pS[I].stat_counter_main++;
    }

    if(pS[I].microtrace != 0){
      printf("MI: LC ADVANCE: NEW LC = 0x%X (0%o)\n",pS[I].LCregister.raw,pS[I].LCregister.raw);
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
      printf("USE OF MACRO-STREAM-ADVANCE DOES NOT FIT EXPECTED SCENARIO - INVESTIGATE!\n");
      pS[I].cpu_die_rq = 1;
    }
#ifdef ISTREAM
    // Will we need the bus?
    if(((pS[I].LCregister.raw>>1)&0x01) == 0x0){
      // We will need the memory bus. Can we have it?
#endif
      if(NUbus_Busy > 0 && pS[I].ConReg.Enable_NU_Master == 1){
	if(pS[I].microtrace){
	  printf("MACRO-STREAM-ADVANCE: AWAITING MEMORY BUS...\n");
	}
	// No, burn a cycle and come back
	pS[I].stall_count++; // Track ticks burned
	pS[I].exec_hold = true;
	return;
      }
#ifdef ISTREAM
    }else{
      // This MSA will not cause a fetch. Is it acceptable to clobber VMA and read anyway?
      // We should check.
      printf("MACRO-STREAM-ADVANCE: NO READ WILL BE NECESSARY - INVESTIGATE IMPLICATIONS OF DOING IT ANYWAY / NOT DOING IT\n");
      // pS[I].cpu_die_rq = 1;
    }

    if(((pS[I].LCregister.raw>>1)&0x01) == 0x0){
#endif
      if(pS[I].microtrace){
	printf("MACRO-STREAM-ADVANCE: INITIATING MEMORY READ...\n");
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
	nubus_io_request(pS[I].vm_byte_mode|VM_READ,pS[I].NUbus_ID,pS[I].vm_phys_addr.raw,0);
      }   
#ifdef ISTREAM
    }
#endif
    // Advance LC, then load VMA from LC and initiate a read.
    pS[I].LCregister.raw += 2; // Yes, this is right
    if(pS[I].RG_Mode.Aux_Stat_Count_Control == 03){
      pS[I].stat_counter_aux++;
    }
    if(pS[I].RG_Mode.Main_Stat_Count_Control == 03){
      pS[I].stat_counter_main++;
    }
    if(pS[I].microtrace){
      printf("MACRO-STREAM-ADVANCE: ");
      printf("NEW LC = 0x%X (0%o)\n",pS[I].LCregister.raw,pS[I].LCregister.raw);
    }
  }

  // If we are in forced hold, die here.
  if(pS[I].PMR.Force_T_Hold != 0){
    pS[I].exec_hold = true;
    return;
  }

  pS[I].exec_hold = false; // Release hold
  if(pS[I].Iregister.Stat_Bit != 0){ printf("\nSTAT\n"); pS[I].cpu_die_rq = 1; }
  // if(pS[I].Iregister.ILong != 0){ printf("\nILong\n"); pS[I].cpu_die_rq = 1; }
  // if(pS[I].Iregister.Macro_IR_Disp != 0){ printf("\nMIR-DISP\n"); pS[I].cpu_die_rq = 1; }
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
#ifdef ISTREAM
    if(((pS[I].LCregister.raw>>1)&0x01) == 0x01){
#endif
      if(pS[I].microtrace){
	printf("S2MIR: Loaded MIR\n");
      }
      pS[I].MIregister.raw = pS[I].Mbus;
#ifdef ISTREAM
      pS[I].mirInvalid = 0;
    }else{
      if(pS[I].microtrace){
	printf("S2MIR: Suppressed Loading MIR\n");
      }
      if(pS[I].mirInvalid == 1 || pS[I].loc_ctr_reg.raw > 036000){
	pS[I].MIregister.raw = pS[I].Mbus;
	//pS[I].MIregister.raw &= 0xFFFF0000;
	//pS[I].MIregister.raw |= (pS[I].Mbus & 0xFFFF0000);
	pS[I].mirInvalid = 0;
	if(pS[I].microtrace){
	  printf("S2MIR: mirInvalid override suppress of MIR\n");
	  printf("MID: MIR = %o %o LC = %o\n",
		 pS[I].MIregister.mi[0].raw,pS[I].MIregister.mi[1].raw,
		 pS[I].LCregister.raw);
	}
      }
    }
#endif
  }

  // MIR-DISP
  if(pS[I].Iregister.Macro_IR_Disp != 0){
    if(pS[I].microtrace){
      printf("MIR-DISP: Flag Set\n"); 
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

    if(pS[I].Iregister.ALU.Misc > 0){ printf("ALU-MISC "); pS[I].cpu_die_rq = 1; }

    // Handle destination selector
    handle_destination(I);

    // Halt if spare bit set
    if(pS[I].Iregister.ALU.Spare > 0){ printf("ALU-SPARE "); pS[I].cpu_die_rq = 1; }
    
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

    if(pS[I].Iregister.Byte.Misc > 0){ printf("Misc "); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.Byte.Spare > 0){ printf("BYTE-SPARE "); pS[I].cpu_die_rq = 1; }
    break;

  case 2: // JUMP-OP    
    if(pS[I].Iregister.Jump.LC_Increment != 0){ printf(" LCINC"); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.Jump.Spare != 0){ printf(" JUMP-SPARE"); pS[I].cpu_die_rq = 1; }
    if(pS[I].Iregister.Jump.Spare2 != 0){ printf(" JUMP-SPARE2"); pS[I].cpu_die_rq = 1; }

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
	printf("Unknown jump cond %o\n",pS[I].Iregister.Jump.Cond);
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
	printf("JUMP: Pending PJAN investigation stop: Not CALL or RETURN (Op %o)\n",pS[I].Iregister.Jump.RPN);
	pS[I].cpu_die_rq = 1;
      }
      // Handle operation
      switch(pS[I].Iregister.Jump.RPN){
      case 0: // Jump-Branch-Xct-Next
        // Jump, but DO NOT inhibit the next instruction!
	if(pS[I].loc_ctr_nxt != -1){
	  printf("JUMP: Pending JUMP-After-Next collision investigation stop\n");
	  pS[I].cpu_die_rq = 1;
	}
        pS[I].loc_ctr_nxt = pS[I].Iregister.Jump.Address;
        break;
	
      case 1: // Jump-Branch
	if(pS[I].microtrace && pS[I].loc_ctr_reg.raw != (pS[I].loc_ctr_cnt + 1)){
	  printf("JUMP: Pending JUMP collision investigation marker\n");
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

          printf("uStack[%o] = ",pS[I].uPCS_ptr_reg);

          location = "";
          offset = 0;
          location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	  if(location != 0){
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    printf("%s",symloc);
	  }
          printf(" (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
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

          printf("uStack[%o] = ",pS[I].uPCS_ptr_reg);

          location = "";
          offset = 0;
          location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	  if(location != 0){
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    printf("%s",symloc);
	  }
          printf(" (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
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
	  printf("RETURN-XCT-NEXT with PJAN armed!\n");
	  pS[I].cpu_die_rq=1;
	}
        // POP ADDRESS
        pS[I].loc_ctr_nxt = pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]&0xFFFFF;
        pS[I].uPCS_ptr_reg--;  pS[I].uPCS_ptr_reg &= 0xFF;
        break;

      case 5: // Jump-Return
	if(pS[I].popj_after_nxt != -1){
	  // PJAN is armed. Do not double!
	  // printf("RETURN with PJAN armed!\n");
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
	printf("Unknown jump RPN %o\n",pS[I].Iregister.Jump.RPN);
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
      // if(pS[I].Iregister.Dispatch.Constant > 0){ printf("Constant "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].Iregister.Dispatch.LPC > 0){ printf("LPC "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].Iregister.Dispatch.Write_VMA > 0){ printf("WriteVMA "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta > 0){ printf("EnableGCVMeta "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].Iregister.Dispatch.Enable_Oldspace_Meta > 0){ printf("EnableOldspaceMeta "); pS[I].cpu_die_rq = 1; }
      if(pS[I].Iregister.Dispatch.Spare > 0){ printf("DISP-SPARE "); pS[I].cpu_die_rq = 1; }
      // if(pS[I].popj_after_nxt != -1){ printf("DISPATCH with POPJ-AFTER-NEXT armed?\n"); pS[I].cpu_die_rq = 1; }

      // Load VMA from MD     
      // if(pS[I].Iregister.Dispatch.Write_VMA != 0){ pS[I].VMAregister = pS[I].MDregister; }
      // Load VMA from M source instead!
      if(pS[I].Iregister.Dispatch.Write_VMA != 0){ pS[I].VMAregister.raw = pS[I].Mbus; }
      
      // Lambda doesn't have dispatch-source, so I assume it's always R-bus
      Mask = (1 << pS[I].Iregister.Dispatch.Len) - 1;      
      // Meta handling
      if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta || pS[I].Iregister.Dispatch.Enable_Oldspace_Meta){
	Mask = Mask & 0xfffffffe;	
      }
      // Investigation stop
      if(pS[I].Iregister.Dispatch.Enable_GC_Volatility_Meta && pS[I].Iregister.Dispatch.Enable_Oldspace_Meta){
	printf("DISPATCH: Enable GCV and Oldspace meta simultaneously?\n");
	pS[I].cpu_die_rq=10;
      }
      if(pS[I].microtrace){
	printf("DISPATCH: GENERATED MASK 0x%X\n",Mask);
      }

      // Lambda does not have a rotate direction flag.
      dispatch_source = left_rotate(pS[I].Mbus, pS[I].Iregister.Dispatch.Pos) & Mask;

      if(pS[I].microtrace){
	printf("DISPATCH: dispatch_source = 0x%X\n",dispatch_source);
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

	// I think this is how this is supposed to go
	// if((pS[I].cached_gcv&03) >= present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){ gc_volatilty_flag = 1; }else{ pS[I].cpu_die_rq=1; } 

	// Meta bits are un-inverted now.
	// gc_volatilty_flag 1 == don't trap
	// So we want to set it 0 if we should trap.
	// if(pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid != 0){ printf("GCV: LV1 invalid?\n"); pS[I].cpu_die_rq=1; } // Investigate
	// if((pS[I].cached_gcv&03) <= present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){ gc_volatilty_flag = 1; }else{ pS[I].cpu_die_rq=0; } // ILLOP at PHTDEL6+11

	// This comparison is correct for non-inverted meta.
	// For the moment, LV1 invalidity forces a trap. This isn't conclusively proven correct, and may change.
	// if((pS[I].cached_gcv&03) > present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){ gc_volatilty_flag = 1; }
	if(pS[I].cached_gcv <= present_gcv && pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid == 0){
	  gc_volatilty_flag = 1;
	}

	if(pS[I].microtrace){
	  printf("DISPATCH: GCV: CACHED (LV2) GCV 0x%X\n",pS[I].cached_gcv&03);
	  printf("DISPATCH: GCV: PRESENT (LV1) GCV 0x%X\n",present_gcv);
	  printf("DISPATCH: GCV: LV1 ENT 0x%X = 0x%X (Meta 0x%X Validity %o)\n",
		 pS[I].MDregister.VM.VPage_Block,pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].raw,
		 pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB,
		 pS[I].vm_lv1_map[pS[I].MDregister.VM.VPage_Block].MB_Valid);
	  printf("DISPATCH: GCV: LV2 ENT 0x%X = 0x%X (Meta 0x%X)\n",
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
	  printf("DISPATCH: META: LV2 ENT 0x%X = 0x%X (Meta 0x%X)\n",
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
	printf("DISPATCH: GENERATED ADDRESS 0x%X AND FETCHED WORD 0x%X\n",disp_address,disp_word.raw);
      }
      // Handle dispatch word
      if(pS[I].microtrace){
	char *location;
	char symloc[100];
	int offset;

	printf("DISPATCH: OP %s DEST ",jump_op_str[disp_word.Operation]);
	location = "";
	offset = 0;
	location = sym_find_last(1, disp_word.PC, &offset);
	if(location != 0){
	  if(offset != 0){
	    sprintf(symloc, "%s+%o", location, offset);
	  }else{
	    sprintf(symloc, "%s", location);
	  }
	  printf("%s",symloc);
	}
	printf(" (%o)\n",disp_word.PC);
      }
      // Handle operation of Start-Memory-Read
      if(disp_word.StartRead){
	if(pS[I].microtrace != 0){
	  printf(" START-MEMORY-READ");
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
	  nubus_io_request(VM_READ,pS[I].NUbus_ID,pS[I].vm_phys_addr.raw,0);
	}
      }
      // Handle operation
      switch(disp_word.Operation){
      case 0: // Jump-Branch-Xct-Next
        // Jump, but DO NOT inhibit the next instruction!
	if(pS[I].loc_ctr_nxt != -1){
	  printf("DISPATCH: Pending JUMP-After-Next collision investigation stop\n");
	  pS[I].cpu_die_rq = 1;
	}
        pS[I].loc_ctr_nxt = disp_word.PC;
        break;
	
      case 1: // Jump-Branch
	if(pS[I].loc_ctr_reg.raw != (pS[I].loc_ctr_cnt + 1)){
	  printf("DISPATCH: Pending JUMP collision investigation stop\n");
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
	  printf("DISPATCH: LPC set w/ call-xct-next?\n");
	  pS[I].cpu_die_rq = 1;
	}
	pS[I].uPCS_stack[pS[I].uPCS_ptr_reg] = pS[I].loc_ctr_reg.raw+1; // Pushes the address of the next instruction	
        if(pS[I].microtrace){
          char *location;
          char symloc[100];
          int offset;

          printf("uStack[%o] = ",pS[I].uPCS_ptr_reg);

          location = "";
          offset = 0;
          location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	  if(location != 0){
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }
	    printf("%s",symloc);
	  }
          printf(" (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
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
          //printf("RETURN-XCT-NEXT with PJAN armed!\n");
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

          printf("uStack[%o] = ",pS[I].uPCS_ptr_reg);

          location = "";
          offset = 0;
          location = sym_find_last(1, pS[I].uPCS_stack[pS[I].uPCS_ptr_reg], &offset);
	  if(location != 0){
	    if(offset != 0){
	      sprintf(symloc, "%s+%o", location, offset);
	    }else{
	      sprintf(symloc, "%s", location);
	    }	  
	    printf("%s",symloc);
	  }
          printf(" (%o)\n",pS[I].uPCS_stack[pS[I].uPCS_ptr_reg]);
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
	printf("Unknown dispatch RPN %o\n",disp_word.Operation);
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
      printf("LAMBDA %d: Single Step completed\n",I);
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
    printf("[CPU %d] MASTER CLOCK STOPPED\n",I);
    pS[I].ConReg.halt_request = 1;
    pS[I].ConReg.uinst_clock_l = 1;
    //disassemble_IR();
    //disassemble_MIR();
#ifdef LAMBDA_DEBUGTRACE
    if(!pS[I].microtrace && pS[I].loc_ctr_reg.raw != 036004){
      int x=debugtrace_ptr[I];
      printf("Writing debug log...\n");
      write_debugtrace_ent(I,x);
      x++;
      if(x == MAX_DEBUG_TRACE){ x=0; }
      while(x != debugtrace_ptr[I]){
	write_debugtrace_ent(I,x);
	x++;
	if(x == MAX_DEBUG_TRACE){ x=0; }
      }
    }
    printf("Completed!\n");
#endif
  }

  // Handle HALT bit
  if((pS[I].Iregister.Halt)){
    printf("HALTed PC points to next inst\n");
    pS[I].loc_ctr_reg.raw++;
  }
  
}
