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

/* Bus Master Mutexes */
extern pthread_mutex_t nubus_master_mutex;

/* Bus structures */
typedef union rnuAddr {
  uint32_t raw;
  uint16_t hword[2];
  struct {
    uint8_t Byte:2;
    uint32_t word:22;
    uint8_t Card:8;
  } __attribute__((packed));
  struct {
    uint32_t Addr:24;
  } __attribute__((packed));
  // For matching SDU mapping registers
  struct {
    uint32_t Offset:10;
    uint32_t Page:22;
  } __attribute__((packed));
} nuAddr;

typedef union rnuData {
  uint32_t word;
  uint8_t byte[4];
  uint16_t hword[2];
} nuData;

/* Operations */
// CPU MASTER WORD IO
#define VM_READ  0x00
#define VM_WRITE 0x01
// CPU MASTER BYTE IO
#define VM_BYTE_READ  0x02
#define VM_BYTE_WRITE 0x03

/* NUbus Interface */
extern volatile int NUbus_error;
extern volatile int NUbus_Busy;
extern volatile int NUbus_acknowledge;
extern volatile int NUbus_master;
extern volatile nuAddr NUbus_Address;
extern volatile nuData NUbus_Data;
extern volatile nuData NUbus_Block[4]; // HACK FOR BLOCK TRANSFERS
extern volatile int NUbus_Request;
extern volatile int NUbus_trace;

/* Functions */
void take_nubus_mastership();
void release_nubus_mastership();
void nubus_clock_pulse();
void nubus_io_request(int access, int master, uint32_t address, uint32_t data);
void nubus_xfer(int access, int master, uint32_t address, uint32_t data);
