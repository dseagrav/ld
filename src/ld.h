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

/* Here is a macroinstruction, for parsing of Qs */
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
