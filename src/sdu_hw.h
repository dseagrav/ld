/* Copyright 2016-2017
   Daniel Seagraves <dseagrav@lunar-tokyo.net>
   Barry Silverman <barry@disus.com>
   Mike Chambers

   This file is part of LambdaDelta and includes code from Fake86.

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

void reset86();
void i8086_clockpulse();

#define regax 0
#define regcx 1
#define regdx 2
#define regbx 3
#define regsp 4
#define regbp 5
#define regsi 6
#define regdi 7
#define reges 0
#define regcs 1
#define regss 2
#define regds 3

#ifdef __BIG_ENDIAN__
#define regal 1
#define regah 0
#define regcl 3
#define regch 2
#define regdl 5
#define regdh 4
#define regbl 7
#define regbh 6
#else
#define regal 0
#define regah 1
#define regcl 2
#define regch 3
#define regdl 4
#define regdh 5
#define regbl 6
#define regbh 7
#endif

union _bytewordregs_ {
  uint16_t wordregs[8];
  uint8_t byteregs[8];
};

#ifdef CPU_ADDR_MODE_CACHE
struct addrmodecache_s {
  uint16_t exitcs;
  uint16_t exitip;
  uint16_t disp16;
  uint32_t len;
  uint8_t mode;
  uint8_t reg;
  uint8_t rm;
  uint8_t forcess;
  uint8_t valid;
};
#endif

#define StepIP(x)       ip += x
#define getmem8(x, y)   read86(segbase(x) + y)
#define getmem16(x, y)  readw86(segbase(x) + y)
#define putmem8(x, y, z)        write86(segbase(x) + y, z)
#define putmem16(x, y, z)       writew86(segbase(x) + y, z)
#define signext(value)  (int16_t)(int8_t)(value)
#define signext32(value)        (int32_t)(int16_t)(value)
#define getreg16(regid) regs.wordregs[regid]
#define getreg8(regid)  regs.byteregs[byteregtable[regid]]
#define putreg16(regid, writeval)       regs.wordregs[regid] = writeval
#define putreg8(regid, writeval)        regs.byteregs[byteregtable[regid]] = writeval
#define getsegreg(regid)        segregs[regid]
#define putsegreg(regid, writeval)      segregs[regid] = writeval
#define segbase(x)      ((uint32_t) x << 4)

#define makeflagsword() \
  ( \
  2 | (uint16_t) cf | ((uint16_t) pf << 2) | ((uint16_t) af << 4) | ((uint16_t) zf << 6) | ((uint16_t) sf << 7) | \
  ((uint16_t) tf << 8) | ((uint16_t) ifl << 9) | ((uint16_t) df << 10) | ((uint16_t) of << 11) \
    )

#define decodeflagsword(x) { \
  temp16 = x; \
  cf = temp16 & 1; \
  pf = (temp16 >> 2) & 1; \
  af = (temp16 >> 4) & 1; \
  zf = (temp16 >> 6) & 1; \
  sf = (temp16 >> 7) & 1; \
  tf = (temp16 >> 8) & 1; \
  ifl = (temp16 >> 9) & 1; \
  df = (temp16 >> 10) & 1; \
  of = (temp16 >> 11) & 1; \
  }

#define modregrm() {			  \
    addrbyte = getmem8(segregs[regcs], ip);	\
    StepIP(1);					\
    mode = addrbyte >> 6;			\
    reg = (addrbyte >> 3) & 7;			\
    rm = addrbyte & 7;				\
    switch(mode)				\
      {						\
      case 0:					\
	if(rm == 6) {				\
	  disp16 = getmem16(segregs[regcs], ip);	\
	  StepIP(2);					\
	}						\
	if(((rm == 2) || (rm == 3)) && !segoverride) {	\
	  useseg = segregs[regss];			\
	}						\
	break;						\
							\
      case 1:						\
	disp16 = signext(getmem8(segregs[regcs], ip));	\
	StepIP(1);					     \
	if(((rm == 2) || (rm == 3) || (rm == 6)) && !segoverride) {	\
	  useseg = segregs[regss];					\
	}								\
	break;								\
									\
      case 2:								\
	disp16 = getmem16(segregs[regcs], ip);				\
	StepIP(2);							\
	if(((rm == 2) || (rm == 3) || (rm == 6)) && !segoverride) {	\
	  useseg = segregs[regss];					\
	}								\
	break;								\
									\
      default:								\
	disp8 = 0;							\
	disp16 = 0;							\
      }									\
  }

// 8259 PIC
typedef struct tag_i8259 {
  uint8_t ISR; // In-Service Register
  uint8_t IRR; // Int Request Register
  uint8_t IMR; // Int Mask Register
  int State;   // 0 = init, 1-3 = ICWx, 4 = Operating
  int ICW4;    // Do or don't use ICW4
  int ReadReg; // Which reg to read out
  uint8_t Base; // Set in ICW2
  uint8_t IRQ; // Interrupt Request Lines
  uint8_t Last_IRQ; // Last IRQ status
  uint8_t NSMR; // Not-Slave Mask Register
  uint8_t SFNM; // Special Fully Nested Mode (used with NSMR)
  uint8_t Pending_ISR; // ISR was refreshed after EOI, reservice
} i8259;

// 8253 PIT
typedef struct tag_i8253 {
  uint16_t Counter[3];     // Binary counter
  double RCounter[3];      // Real Counter
  uint8_t Output[3];       // Output line
  uint8_t Output_Ticks[3]; // Count since last change
  uint16_t IV[3];          // Initial value
  uint8_t LoadHW[3];       // Load Halfword
  uint8_t Format[3];       // Format of count
  uint8_t Mode[3];         // Mode byte
  uint8_t Control[3];      // Control byte
  uint8_t State[3];        // State
} i8253;
