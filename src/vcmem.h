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

void vcmem_init(int vn,int slot);
void vcmem_clock_pulse(int vn);
void vcmem_kb_int(int vn);

// Register definitions

// Function Register
typedef union rFunctionReg {
  uint16_t raw;
  uint8_t byte[2];
  struct {
    uint8_t Reset:1;
    uint8_t BoardEnable:1;
    uint8_t LED:1;
    uint8_t Function:2; // 0 = XOR, 1 = OR, 2 = AND, 3 = STORE
    // Rest not documented/accessed?
    uint16_t Spare:11;
  } __attribute__((packed));
} FunctionReg;

// Memory Control Register
typedef union rMemoryControlReg {
  uint16_t raw;
  uint8_t byte[2];
  struct {
    uint8_t RefreshCycles:2; // Refresh cycles per horizontal line
    uint8_t MemBank:1;
    uint8_t MemCopy:1;       // Enable Bank A -> Bank B copying
    uint8_t ReverseVideo:1;
    uint8_t InterruptEnabled:1;
    uint8_t NUBusSelect:1;
  } __attribute__((packed));
} MemoryControlReg;

// Interrupt Status Register
typedef union rInterruptStatusReg {
  uint32_t raw;
  uint8_t byte[4];
  struct {
    // Status Register 0
    uint8_t VerticalBlank:1;
    uint8_t FieldSelect:1;
    uint8_t HorizontalBlank:1;
    uint8_t HorizontalSelect:1;
    uint8_t VerticalSync:1;
    uint8_t CompositeSync:1;
    uint8_t CompositeBlank:1;
    uint8_t TTLVideoOutput:1;
    // Status Register 1
    uint8_t SerialParityError:1;
    uint8_t SerialFrameError:1;
    uint8_t SerialOverrunError:1;
    uint8_t SerialXmitDataEmpty:1;
    uint8_t SerialXmitEmpty:1;
    uint8_t SerialFIFOEmpty:1;
    uint8_t SerialFIFOFull:1;
    // Spare
    uint32_t Spare:17;
  } __attribute__((packed));
} InterruptStatusReg;

// Serial port control register
typedef union rSerialControlReg {
  uint16_t raw;
  uint8_t byte[2];
  struct {
    uint8_t ParityInhibit:1;
    uint8_t Spare:7;
    uint8_t BaudRate:4;
    uint8_t StopBit:1;
    uint8_t EvenParity:1;
    uint8_t WordLength:2;
  } __attribute__((packed));
} SerialControlReg;

// Framebuffer and scanline table sizes
#define FB_SIZE (1024*128)
#define SLT_SIZE (0x800)

// Card state
struct vcmemState {
  // Memories
  uint8_t AMemory[FB_SIZE];
  uint8_t BMemory[FB_SIZE];
  uint32_t SLT[SLT_SIZE]; // Scanline Table
  // Register storage
  FunctionReg Function;
  MemoryControlReg MemoryControl;
  uint32_t InterruptAddr;
  InterruptStatusReg InterruptStatus;
  SerialControlReg SerialControl;
  // Timing
  uint32_t cycle_count;
  // Software state
  uint8_t Card;
};
