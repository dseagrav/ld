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

/* LambdaDelta Global Header */

// Kernel items
void lambda_dump(int opts);
#ifdef BURR_BROWN
void debug_tx_rq(uint8_t rq,uint32_t addr,uint32_t data);
void debug_connect();
void debug_clockpulse();
#endif
void nubus_cycle(int sdu);
void sducons_write(char data);

// Host framebuffer interface
void framebuffer_update_word(int vn,uint32_t addr,uint32_t data);
void framebuffer_update_hword(int vn,uint32_t addr,uint16_t data);
void framebuffer_update_byte(int vn,uint32_t addr,uint8_t data);
void set_bow_mode(int vn,int mode);

#if defined(XBEEP) || defined(CONFIG_PHYSKBD)
// xbeep and/or physkbd beep
void xbeep(int halfwavelength, int duration);
#endif

#if defined(CONFIG_PHYSKBD) || !defined(XBEEP)
// Old hack
// beep on/off (called when vcmem keyboard control register 5 toggles "send break" bit.
void audio_control(int onoff,int console);
#endif

// Host ethernet interface
void ether_tx_pkt(uint8_t *data,uint32_t len);
uint32_t ether_rx_pkt();

// Mouse interface callback
void warp_mouse_callback(int cp);

// Logging stuff
int logmsgf(int type, int level, const char *format, ...);

// Type numbers
// Make sure these stay in sync with the array logtype_name in kernel.c
// I can't initialize that here because gcc whines about it (sigh)
#define LT_SYSTEM 0
#define LT_SDU 1
#define LT_LAMBDA 2
#define LT_NUBUS 3
#define LT_MULTIBUS 4
#define LT_RTC 5
#define LT_VCMEM 6
#define LT_3COM 7
#define LT_SMD 8
#define LT_TAPEMASTER 9
#define LT_MEM 10
#define LT_LISP 11
// If you let MAX_LOGTYPE get above the actual number of initialized types, you're gonna have a bad time
#define MAX_LOGTYPE 12

// Dump options bits
#define DUMP_A_MEM 0x00000001
#define DUMP_M_MEM 0x00000002
#define DUMP_U_STACK 0x00000004
#define DUMP_PDL 0x00000008
#define DUMP_P_MEM 0x00000010
#define DUMP_MID_MEM 0x00000020
#define DUMP_SHADOW_MEM 0x00000040
#define DUMP_T_MEM 0x00000080
#define DUMP_NO_INC_SEQ 0x40000000
#define DUMP_ALL 0x3FFFFFFF

/* Macroinstruction */
typedef union rMI {
  uint16_t raw;
  // MISC/MISC1-OP
  // This was mine
  struct {
    uint8_t Bits:6;     // ?
    uint16_t Opcode:10; // Opcode
  } __attribute__((packed)) Misc;
  // Fields from ulambda
  struct {
    uint16_t Adr:9;
    uint8_t Opcode:5;
    uint8_t Dest:2;
  } __attribute__((packed));
  struct {
    uint8_t Displacement:6;
    uint8_t Register:3;
    uint8_t Padding:4;
    uint8_t Sub_Opcode:3;
  } __attribute__((packed));
} MI;

/* Here is a Q */
typedef union rQ {
  uint32_t raw;
  // Q
  struct {
    uint32_t ptr:25;
    uint8_t dtp:5;
    uint8_t cdr:2;
  } __attribute__((packed));
  // (25-bit) virtual memory address
  struct {
    uint8_t  Offset:8;
    uint8_t  VPage_Offset:5;
    uint16_t VPage_Block:12;
  } __attribute__((packed)) VM;
  // Shadow memory indexing
  struct {
    uint8_t Offset:8;
    uint32_t VPage:17;
  } __attribute__((packed)) SM;
  // For easier parsing of macroinstructions
  struct {
    MI mi[2];
  } __attribute__((packed));
} Q;

/* macroinstruction dispatch memory addressing */
typedef union rMIDAddress {
  uint16_t raw;
  struct {
    uint16_t Opcode:10;
    uint8_t  Hi:2;
  } __attribute__((packed));
} MIDAddress;

/* Dispatch word */
typedef union rDispatchWord {
  uint32_t raw;
  struct {
    uint32_t PC:16;
    uint8_t Operation:3; // R-P-N
    uint8_t StartRead:1;
  } __attribute__((packed));
} DispatchWord;

/* Memory map entries */
/* Level 1 */
typedef union rlv1_ent {
  uint32_t raw;
  struct {
    uint8_t LV2_Block:7;
    uint8_t MB:2; // MB0, MB1
    uint8_t MB_Valid:1;
  } __attribute__((packed));
} lv1_ent;

/* Level 2 Index */
typedef union rlv2_idx {
  uint16_t raw;
  struct {
    uint8_t VPage_Offset:5;
    uint8_t LV2_Block:7;
  } __attribute__((packed));
} lv2_idx;

/* Level 2 Control */
typedef union rlv2_ctl_ent {
  uint32_t raw;
  struct {
    uint8_t Meta:6;
    uint8_t Status:3;
    uint8_t AccHi:1; // Hi bit of access (it overlaps, see next struct)
    uint8_t Force_Allowed:1;
    uint8_t Packet_Code:2; // Byte code? ("packet size")
    uint8_t Packetize_Writes:1; // ???
    uint8_t Cache_Permit:1;
    uint8_t Lock_NUbus:1;
    uint32_t Unused:16; // Stop if written
  } __attribute__((packed));
  struct {
    uint8_t Padding:8;
    uint8_t Access:2;
  } __attribute__((packed));
} lv2_ctl_ent;

/* Level 2 Address */
typedef union rlv2_adr_ent {
  uint32_t raw;
  struct {
    uint32_t PPN:22;
    uint8_t Byte_Code:2;
  } __attribute__((packed));
} lv2_adr_ent;

/* Physical Address (the result of the above mapping) */
typedef union rPhysAddr {
  uint32_t raw;
  struct {
    uint8_t Byte:2;
    uint8_t Offset:8;
    uint32_t PPN:22;
  } __attribute__((packed));
} PhysAddr;

/* Destination selector fields (ALU/BYTE instructions) */
/* If A.Flag is set, this is an A-Memory address.
   If A.Flag is clear and F.Dest is set, this is a functional destination.
   if A.Flag is clear and F.Dest is clear, this is an A+M-Memory address.
   What happens if A.Flag is clear, F.Dest is set, and M.Addr is also set? Write both?
 */
typedef union rDestSelect {
  uint32_t raw;
  struct A {
    uint16_t Padding:14;
    uint16_t Addr:12;
    uint8_t  Flag:1;
  } __attribute__((packed)) A;
  struct M {
    uint16_t Padding:14;
    uint8_t  Addr:6;
  } __attribute__((packed)) M;
  struct F {
    uint16_t Padding:14;
    uint8_t  Spare:6;
    uint8_t  Dest:6;
  } __attribute__((packed)) F;
} DestSelect;

/* Microaddress */
typedef union rMicroAddr {
  uint32_t raw;
  struct {
    uint8_t  Offset:4;
    uint16_t Page:12;
  } __attribute__((packed));
} MicroAddr;

/* Left half of jump microinstruction */
typedef union rJumpInst {
  uint64_t raw;
  struct {
    uint8_t  Cond:5;
    uint8_t  Spare2:1;
    uint8_t  Test:1;
    uint8_t  Invert:1;
    uint8_t  RPN:3;
    uint8_t  LC_Increment:1;
    uint8_t  Spare:2;
    uint32_t Address:17;
  } __attribute__((packed));
} JumpUInst;

/* Left half of ALU microinstruction */
typedef union rALUInst {
  uint64_t raw;
  struct {
    uint8_t  QControl:2;
    uint8_t  Carry:1;
    uint8_t  Operation:5;
    uint8_t  Mask:1;
    uint8_t  Output:3;
    uint8_t  Misc:2;
    uint16_t Dest:13;
    uint8_t  Spare:3; // Spare can be overwritten by Dest when Dest contains a CRAM address.
  } __attribute__((packed));
} ALUInst;

/* Left half of Byte microinstruction */
typedef union rByteUInst {
  uint64_t raw;
  struct {
    uint8_t  Pos:6;
    uint8_t  Len:6; // Actual length is this + 1
    uint8_t  Misc:2;
    uint16_t Dest:13;
    uint8_t  Spare:1;
    uint8_t  Rotate_Source:1;
    uint8_t  Rotate_Mask:1;
  } __attribute__((packed));
} ByteUInst;

typedef union rDispatchUInst {
  uint64_t raw;
  struct {
    uint8_t  Pos:6;
    uint8_t  Len:5;
    uint8_t  Spare:3;
    uint16_t Constant:12;
    uint8_t  LPC:1;
    uint8_t  Write_VMA:1;
    uint8_t  Enable_GC_Volatility_Meta:1;
    uint8_t  Enable_Oldspace_Meta:1;
  } __attribute__((packed));
} DispatchUInst;

/* Here is a Lambda microinstruction */
typedef union rUInst {
  uint64_t raw;
  uint32_t word[2];
  uint8_t byte[8];
  // Left Halves
  JumpUInst     Jump;
  ALUInst       ALU;
  ByteUInst     Byte;
  DispatchUInst Dispatch;
  // ALU/Byte destination select
  DestSelect    Destination;
  // Global (right half)
  struct {
    uint32_t rtHalf:30;
    uint8_t  Opcode:2;
    uint8_t  MSource:7;
    uint16_t ASource:12;
    uint8_t  PopJ_After_Next:1;
    uint8_t  Macro_IR_Disp:1;
    uint8_t  Src_to_Macro_IR:1;
    uint8_t  Macro_Stream_Advance:1;
    uint8_t  Slow_Dest:1;
    uint8_t  ILong:1;
    uint8_t  Stat_Bit:1;
    uint8_t  Clobbers_Mem_Subr_Bit:1;
    uint8_t  Halt:1;
    uint8_t  Parity:4;
  } __attribute__((packed));
} UInst;
