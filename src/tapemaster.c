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

/* Tapemaster controller */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <dirent.h>

#include "ld.h"
#include "nubus.h"
#include "sdu.h"

int tape_fd = -1;            // FD for tape
int tape_bot = 1;            // At bottom of tape
int tape_eot = 1;            // At end of tape
int tape_fm = 0;             // At filemark
int tape_reclen = 0;         // Tape record length
int tape_error = 0;          // Tapemaster error code
uint8_t tape_block[64*1024]; // Tape block buffer
char tape_fn[32] = "None";   // Current tape filename
int tape_file_sel = -1;      // Tape file selector

// Structures and such
typedef union rTape_PB_Control_HWord {
  uint16_t raw;
  struct {
    uint8_t Spare0:2;
    uint8_t Tape_Select:2;
    uint8_t Mailbox:1;
    uint8_t Interrupt:1;
    uint8_t Link:1;
    uint8_t Bus_Lock:1;
    uint8_t Bank_Select:1; // Really FORMATTER ADDRESS
    uint8_t Skip_EOT:1;
    uint8_t Reverse:1;
    uint8_t Speed_or_Density:1; // Some drives interpret this as SPEED,
                                // others DENSITY.
                                // Zero = low speed / high density
    uint8_t Continuous:1;
    uint8_t Buffered_Read_Enable:1; // Allows data to be xfer'd from TM buffer
                                    // to system memory when a tape time-out
                                    // happens during a Buffered Read.
    uint8_t Spare2:1;
    uint8_t Width:1; // 1 = 16-bit transfers, 0 = 8-bit transfers
  } __attribute__((packed));
} Tape_PB_Control_HWord;

typedef union rTM_DR_Status {
  uint8_t raw;
  struct {
    uint8_t Spare:1;
    uint8_t Write_Protect:1;
    uint8_t Formatter_Busy:1;
    uint8_t Ready:1;
    uint8_t End_Of_Tape:1;
    uint8_t Load_Point:1;
    uint8_t Online:1;
    uint8_t File_Mark:1;
  } __attribute__((packed));
} TM_DR_Status;

typedef union rTM_CD_Status {
  uint8_t raw;
  struct {
    uint8_t Error:5;
    uint8_t Retry:1;
    uint8_t Complete:1;
    uint8_t Entered:1;
  } __attribute__((packed));
} TM_CD_Status;

typedef union rTM_Pointer {
  uint16_t hword[2];
  struct {
    uint16_t Offset;
    uint16_t Base;
  } __attribute__((packed));
} TM_Pointer;

typedef union rTM_Tape_Parameter_Block {
  uint8_t byte[22];
  struct {
    uint8_t Command;
    uint8_t Command2;
    uint8_t MBZ[2];
    Tape_PB_Control_HWord Control;
    uint16_t Return_Count;
    uint16_t Buffer_Size;
    uint16_t Records;
    TM_Pointer Addr_Pointer;
    TM_DR_Status DR_Status;
    TM_CD_Status CD_Status;
    TM_Pointer Completion_Action_Pointer;
  } __attribute__((packed));
} TM_Tape_Parameter_Block;

typedef union rTM_Parameter_Block {
  uint8_t byte[28];
  uint8_t Command;
  TM_Tape_Parameter_Block Tape;
} TM_Parameter_Block;

typedef union rTM_Streaming_Block_Header {
  uint8_t byte[8];
  uint16_t word[4];
  struct {
    uint8_t Ready:1;
    uint8_t Busy:1;
    uint8_t Complete:1;
    uint8_t Fault:1;
    uint8_t Last_Block:1;
    uint8_t Disk_Terminate:1;
    uint8_t Spare:2;
    uint8_t MBZ;
    uint16_t Byte_Count;
    TM_Pointer Pointer;
  } __attribute__((packed));
} TM_Streaming_Block_Header;

volatile int TM_Controller_State;
int TM_Initialized = 0;
uint8_t TM_Controller_Gate;
uint8_t TM_Controller_CCW;
mbAddr TM_Xfer_Addr;
mbAddr SB_Header_Addr;
nuAddr TM_NB_Addr;
uint32_t TM_SCB_Addr;
uint32_t TM_CCB_Addr;
uint32_t TM_PB_Addr;
int TM_Xfer_Count;
int TM_Xfer_Index;
TM_Streaming_Block_Header TM_SB_Header;
uint8_t TM_Xfer_Buffer[65536];
TM_Parameter_Block TM_PB;
mbAddr TM_MB_Addr;
int SB_Gate_Wait_Time;

// TEMPORARY EXTERNALS
extern int ld_die_rq;

// Check type of file
int is_file(const struct dirent *ent){
  struct stat st;
  char fn[256] = "./tapes/";
  strncat(fn,ent->d_name,255);
  int x = stat(fn,&st);
  if(x < 0){
    char emsg[256] = "stat(): ";
    strncat(emsg,fn,255);
    perror(emsg);
    return(0);
  }
  if(S_ISREG(st.st_mode)){
    return(1);
  }
  return(0);
}

// Open next file
int tapemaster_open_next(){
  struct dirent **namelist = NULL;
  int n = 0;

  // If file is open, close it
  if(tape_fd != -1){
    logmsgf(LT_TAPEMASTER,1,"TM: Closing file %s\n",tape_fn);
    close(tape_fd);
    strncpy(tape_fn,"None",32);
    tape_fd = -1;
    tape_bot = 0;
    tape_eot = 0;
    tape_fm = 0;
    tape_error = 0;
    tape_reclen = 0;
    TM_PB.Tape.DR_Status.raw = 0;
  }
  tape_file_sel++;

  n = scandir("./tapes/",&namelist,is_file,alphasort);
  if(n < 0){
    perror("scandir(): ./tapes/");
    tape_file_sel = -1;
    return(-1);
  }else{
    char fn[256] = "./tapes/";
    if(n == 0){ tape_file_sel = -1; return(-1); } // No files
    if(tape_file_sel >= n){ tape_file_sel = 0; }
    // Attempt open
    strncat(fn,namelist[tape_file_sel]->d_name,255);
    tape_fd = open(fn,O_RDWR);
    if(tape_fd < 0){
      char emsg[256] = "open(): ";
      strncat(emsg,fn,255);
      perror(emsg);
      tape_fd = -1;
    }else{
      strncpy(tape_fn,namelist[tape_file_sel]->d_name,32);
      logmsgf(LT_TAPEMASTER,1,"TM: Opened file %s\n",tape_fn);
      tape_bot = 1;
      tape_eot = 0;
      tape_fm = 0;
      tape_error = 0;
      tape_reclen = 0;
    }
    // Done!
  }
  return(tape_file_sel);
}

// Utility functions
void tapemaster_init(){
  tape_fd = -1;
  tape_bot = 0;
  tape_eot = 0;
  tape_fm = 0;
  tape_error = 0;
  tape_reclen = 0;
  tapemaster_open_next();
}

int tape_space_block(){
  int reclen;
  ssize_t rv=0;
  tape_bot = 0; // No longer at bot
  tape_eot = 0;
  tape_fm = 0;
  tape_error = 0;
  tape_reclen = 0;
  // Obtain record length
  rv = read(tape_fd,(unsigned char *)&reclen,4);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    return(0);
  }
  if(rv < 4){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    return(0);
  }
  logmsgf(LT_TAPEMASTER,10,"TAPE: SPACE-BLOCK: Reclen %d\n",reclen);
  tape_reclen = reclen;
  if(reclen == 0){
    // Found file mark
    tape_fm = 1;
    return(0);
  }
  // Skip data
  rv = lseek(tape_fd,reclen,SEEK_CUR);
  if(rv == -1){
    perror("tape:lseek");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  // Obtain trailing record length
  rv = read(tape_fd,(unsigned char *)&reclen,4);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  if(rv < 4){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    tape_reclen = 0;
    return(0);
  }
  if(reclen != tape_reclen){
    logmsgf(LT_TAPEMASTER,0,"TAPE:reclen mismatch: %d vs %d\n",tape_reclen,reclen);
    ld_die_rq = 1;
    return(tape_reclen);
  }
  return(reclen);
}

int tape_backspace_block(){
  int reclen;
  ssize_t rv=0;
  if(tape_bot != 0){ return(0); } // Nothing to do
  tape_eot = 0;
  tape_fm = 0;
  tape_error = 0;
  tape_reclen = 0;
  // Get trailing record length of previous block
  rv = lseek(tape_fd,-4,SEEK_CUR);
  if(rv == -1){
    perror("tape:lseek");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  // Obtain record length
  rv = read(tape_fd,(unsigned char *)&reclen,4);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    return(0);
  }
  if(rv < 4){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    return(0);
  }
  logmsgf(LT_TAPEMASTER,10,"TAPE: BACKSPACE-BLOCK: Reclen %d\n",reclen);
  tape_reclen = reclen;
  if(reclen == 0){
    // Found file mark
    tape_fm = 1;
    return(0);
  }
  // Skip data and leading record length
  rv = lseek(tape_fd,-(reclen+8),SEEK_CUR);
  if(rv == -1){
    perror("tape:lseek");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  // Obtain leading record length
  rv = read(tape_fd,(unsigned char *)&reclen,4);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  if(rv < 4){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    tape_reclen = 0;
    return(0);
  }
  if(reclen != tape_reclen){
    logmsgf(LT_TAPEMASTER,0,"TAPE:reclen mismatch: %d vs %d\n",tape_reclen,reclen);
    ld_die_rq = 1;
    return(tape_reclen);
  }
  // Done with record length, reposition to re-read it on next op
  rv = lseek(tape_fd,-4,SEEK_CUR);
  if(rv == -1){
    perror("tape:lseek");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  // Mark BOT if we ended there
  if(rv == 0){
    tape_bot = 1;
  }
  return(reclen);
}

int tape_write(){
  ssize_t rv=0;
  tape_bot = 0; // No longer at bot
  tape_fm = 0;
  tape_error = 0;
  // If we were at EOT before, we will still be at EOT.
  // If we were not, we will not.
  // Write record length
  rv = write(tape_fd,(unsigned char *)&tape_reclen,4);
  if(rv == -1){
    perror("tape:write");
    tape_error = 0x0A; // IO error
    return(0);
  }
  if(rv < 4){
    // Unable to write, fake EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    return(0);
  }
  logmsgf(LT_TAPEMASTER,10,"TAPE: WRITE: Reclen %d\n",tape_reclen);
  // Write record
  rv = write(tape_fd,tape_block,tape_reclen);
  if(rv == -1){
    perror("tape:write");
    tape_error = 0x0A; // IO error
    return(0);
  }
  if(rv < tape_reclen){
    // Fake EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    return(0);
  }
  // Write length again
  rv = write(tape_fd,(unsigned char *)&tape_reclen,4);
  if(rv == -1){
    perror("tape:write");
    tape_error = 0x0A; // IO error
    return(0);
  }
  if(rv < 4){
    // Fake EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    return(0);
  }
  // All done!
  return(tape_reclen);
}

int tape_read(){
  int reclen;
  // int next_reclen;
  ssize_t rv=0;
  tape_bot = 0; // No longer at bot
  tape_eot = 0;
  tape_fm = 0;
  tape_error = 0;
  tape_reclen = 0;
  // Obtain record length
  rv = read(tape_fd,(unsigned char *)&reclen,4);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    return(0);
  }
  if(rv < 4){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    return(0);
  }
  logmsgf(LT_TAPEMASTER,10,"TAPE: READ: Reclen %d\n",reclen);
  tape_reclen = reclen;
  if(reclen == 0){
    // Found file mark
    tape_fm = 1;
    tape_error = 0x15; // Unexpected file mark
    return(0);
  }
  rv = read(tape_fd,tape_block,reclen);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  if(rv < reclen){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    tape_reclen = 0;
    return(0);
  }
  // Obtain trailing record length
  rv = read(tape_fd,(unsigned char *)&reclen,4);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  if(rv < 4){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    tape_reclen = 0;
    return(0);
  }
  if(reclen != tape_reclen){
    logmsgf(LT_TAPEMASTER,0,"TAPE:reclen mismatch: %d vs %d\n",tape_reclen,reclen);
    ld_die_rq = 1;
    return(tape_reclen);
  }
  /*
  // Check for EOF/EOT?
  rv = read(tape_fd,(unsigned char *)&next_reclen,4);
  if(rv < 4){
    // Found EOT
    tape_eot = 1;
    tape_fm = 1;
    logmsgf(LT_TAPEMASTER,10,"TAPE:Read found EOT\n");
  }else{
    if(rv == 4){
      if(next_reclen == 0){
        // Found EOF
        tape_fm = 1;
        logmsgf(LT_TAPEMASTER,10,"TAPE:Read found EOF\n");
      }else{
        logmsgf(LT_TAPEMASTER,10,"TAPE:next reclen %d, repositioning\n",next_reclen);
        rv = lseek(tape_fd,-4,SEEK_CUR); // Go back
      }
    }
  }
  */
  return(reclen);
}

int tape_reverse_read(){
  int reclen;
  // int next_reclen;
  ssize_t rv=0;
  if(tape_bot == 0){ return(0); } // Nothing to do
  tape_eot = 0;
  tape_fm = 0;
  tape_error = 0;
  tape_reclen = 0;
  // Obtain length of previous block
  rv = lseek(tape_fd,-4,SEEK_CUR);
  if(rv == -1){
    perror("tape:lseek");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  // Obtain record length
  rv = read(tape_fd,(unsigned char *)&reclen,4);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    return(0);
  }
  if(rv < 4){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    return(0);
  }
  logmsgf(LT_TAPEMASTER,10,"TAPE: READ: Reclen %d\n",reclen);
  tape_reclen = reclen;
  if(reclen == 0){
    // Found file mark
    tape_fm = 1;
    tape_error = 0x15; // Unexpected file mark
    return(0);
  }
  // Go to head of data
  rv = lseek(tape_fd,-(reclen+4),SEEK_CUR);
  if(rv == -1){
    perror("tape:lseek");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  // Obtain data
  rv = read(tape_fd,tape_block,reclen);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  if(rv < reclen){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    tape_reclen = 0;
    return(0);
  }
  // Go to head of record
  rv = lseek(tape_fd,-(reclen+4),SEEK_CUR);
  if(rv == -1){
    perror("tape:lseek");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  // Obtain leading record length
  rv = read(tape_fd,(unsigned char *)&reclen,4);
  if(rv == -1){
    perror("tape:read");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  if(rv < 4){
    // Found EOT
    tape_eot = 1;
    tape_error = 0x09; // Unexpected EOT
    tape_reclen = 0;
    return(0);
  }
  if(reclen != tape_reclen){
    logmsgf(LT_TAPEMASTER,0,"TAPE:reclen mismatch: %d vs %d\n",tape_reclen,reclen);
    ld_die_rq = 1;
    return(tape_reclen);
  }
  // Park at start of block
  rv = lseek(tape_fd,-4,SEEK_CUR);
  if(rv == -1){
    perror("tape:lseek");
    tape_error = 0x0A; // IO error
    tape_reclen = 0;
    return(0);
  }
  // Mark BOT if we ended there
  if(rv == 0){
    tape_bot = 1;
  }
  return(reclen);
}

// Interface functions

void tapemaster_attn(){
  logmsgf(LT_TAPEMASTER,10,"TM: CHANNEL ATTENTION\n");
  TM_Controller_State = 1;
}

void tapemaster_reset(){
  logmsgf(LT_TAPEMASTER,10,"TM: CPU RESET\n");
  TM_Initialized = 0;
}

void tapemaster_clock_pulse(){
  // Tapemaster execution
  if(TM_Controller_State > 0){
    switch(TM_Controller_State){
    case 1: // GO!
      if(TM_Initialized == 0){
        uint16_t offset=0;
        // Perform initialization
        logmsgf(LT_TAPEMASTER,10,"TM: Initialization\n");
        // Load pointer into xfer addr
	TM_MB_Addr.raw = 0x8A;
        TM_Xfer_Addr.byte[0] = multibus_read(TM_MB_Addr);
	TM_MB_Addr.raw = 0x8B;
        TM_Xfer_Addr.byte[1] = multibus_read(TM_MB_Addr);
        TM_Xfer_Addr.raw <<= 4;
	TM_MB_Addr.raw = 0x89;
        offset = multibus_read(TM_MB_Addr);;
        offset <<= 8;
	TM_MB_Addr.raw = 0x88;
        offset |= multibus_read(TM_MB_Addr);
        TM_Xfer_Addr.raw += offset;
        TM_SCB_Addr = TM_Xfer_Addr.raw; // Save for later
	logmsgf(LT_TAPEMASTER,10,"TM: SCB Addr = 0x%X\n",TM_SCB_Addr);
	// Obtain SCB
        TM_Controller_State = 2;
      }else{
        // Already initialized
        TM_Xfer_Addr.raw = TM_CCB_Addr;
        TM_Controller_State = 8; // Fetch CCB
      }
      break;
    case 2: // Initialization: Obtain SCB
      // Verify check byte
      TM_Xfer_Buffer[0] = multibus_read(TM_Xfer_Addr);
      if(TM_Xfer_Buffer[0] != 0x03){
	logmsgf(LT_TAPEMASTER,0,"TM: SCB Check Byte was 0x%X instead of 0x03\n",TM_Xfer_Buffer[0]);
	TM_Controller_State = 0; // Die
	ld_die_rq = 1;
	break;
      }
      // Otherwise...
      TM_Controller_State++;
    case 3: // Initialization: SCB obtained
      {
	uint8_t ptr[4];
	uint16_t seg,off;
	// Obtain pointer and move into TM_CCB_Addr
	TM_Xfer_Addr.raw += 2;
	ptr[0] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
	ptr[1] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
	ptr[2] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
	ptr[3] = multibus_read(TM_Xfer_Addr);
	seg = ptr[3]; seg <<= 8; seg |= ptr[2];
	off = ptr[1]; off <<= 8; off |= ptr[0];
	TM_CCB_Addr = seg;
	TM_CCB_Addr <<= 4;
	TM_CCB_Addr += off;
        logmsgf(LT_TAPEMASTER,10,"TM: CCB Addr = 0x%X\n",TM_CCB_Addr);
        TM_Xfer_Addr.raw = TM_CCB_Addr; // Fetch CCB
        TM_Controller_State = 8; // Go on
      }
      // Fall into
    case 8: // Obtain CCB
      {
	uint8_t ptr[4];
        uint16_t seg,off;
	TM_Controller_CCW = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
	TM_Controller_Gate = multibus_read(TM_Xfer_Addr);
	if(TM_Controller_Gate != 0xFF){
	  TM_Xfer_Addr.raw--;
	  break; // Retry
	}
	logmsgf(LT_TAPEMASTER,10,"TM: Gate Addr = 0x%X\n",TM_Xfer_Addr.raw);
	TM_Xfer_Addr.raw++;
	ptr[0] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
        ptr[1] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
        ptr[2] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
        ptr[3] = multibus_read(TM_Xfer_Addr);
        seg = ptr[3]; seg <<= 8; seg |= ptr[2];
        off = ptr[1]; off <<= 8; off |= ptr[0];
	TM_PB_Addr = seg;
	TM_PB_Addr <<= 4;
	TM_PB_Addr += off;
        logmsgf(LT_TAPEMASTER,10,"TM: PB Addr 0x%X\n",TM_PB_Addr);
	TM_Controller_State = 14;
      }
      // Fall into...
    case 14: // Initialization Complete / Command Execution
      // If this is the end of initialization, we just open the gate.
      if(TM_Initialized == 0){
        TM_Initialized = 1;
        TM_Controller_State = 95;
        logmsgf(LT_TAPEMASTER,10,"TM: Initialization completed! Opening gate\n");
        break;
      }
      // Read the Parameter Block
      TM_Xfer_Count = 0;
      TM_Xfer_Index = 0;
      TM_Xfer_Addr.raw = TM_PB_Addr;
      TM_Controller_State++;
      // Fall into
    case 15: // Read PB
      TM_PB.byte[TM_Xfer_Index+0] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
      TM_PB.byte[TM_Xfer_Index+1] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
      TM_PB.byte[TM_Xfer_Index+2] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
      TM_PB.byte[TM_Xfer_Index+3] = multibus_read(TM_Xfer_Addr); TM_Xfer_Addr.raw++;
      TM_Xfer_Index += 4; // TM_Xfer_Addr.raw += 4;
      TM_Controller_State = 18;
      // Fall into...
    case 18: // Are we Done?
      if(TM_Xfer_Count == 0){
        // We don't know the transfer length, determine it.
        if(TM_PB.Command == 0x80){
          // BLOCK MOVE PB
          TM_Xfer_Count = 28;
        }else{
          if(TM_PB.Command == 0x0C){
            // EXCHANGE PB
            TM_Xfer_Count = 22;
          }else{
            // TAPE PB
            TM_Xfer_Count = 22;
          }
        }
      }
      if(TM_Xfer_Index < TM_Xfer_Count){
        // Need more words
        // TM_Xfer_Addr.raw += 4;
        TM_Controller_State = 15;
        break;
      }
      // Read completed!
      logmsgf(LT_TAPEMASTER,10,"TM: PB OBTAINED!\n");
      // Set up to write COMMAND STATUS byte
      // If we are a BLOCK MOVE, we have a different PB and should skip this.
      if(TM_PB.Command == 0x80){
        TM_Controller_State = 22; // Go directly to dispatch
        break;
      }
      TM_PB.Tape.CD_Status.Entered = 1;
      TM_Xfer_Addr.raw = TM_PB_Addr+17; // CD STATUS
      TM_Controller_State++;
      // Fall into
    case 19: // Write status
      multibus_write(TM_Xfer_Addr,TM_PB.Tape.CD_Status.raw);
      TM_Controller_State = 22; // Go to dispatch
      // Fall into
    case 22: // Command Dispatch
      TM_PB.Tape.CD_Status.Error = 0;
      switch(TM_PB.Command){
      case 0x00: // Configure
        logmsgf(LT_TAPEMASTER,10,"TM: CONFIGURE command\n");
        // Perform housecleaning
        TM_PB.Tape.Return_Count = 0xFFFF; // Amount of buffer available
        TM_Controller_State = 85;
        break;

      case 0x28: // Drive Status
        logmsgf(LT_TAPEMASTER,10,"TM: DRIVE STATUS command\n");
        TM_PB.Tape.DR_Status.raw = 0;
	if(tape_fd > -1){
	  TM_PB.Tape.DR_Status.Ready = 1;
	  TM_PB.Tape.DR_Status.Load_Point = tape_bot;
	  TM_PB.Tape.DR_Status.Online = 1;
	}
        TM_PB.Tape.Return_Count = 0;
        TM_Controller_State = 85;
        break;

      case 0x2C: // Direct Read
	{
	  uint16_t seg,off;
	  // Read a block and transfer to system
	  logmsgf(LT_TAPEMASTER,10,"TM: DIRECT READ command\n");
	  seg = TM_PB.Tape.Addr_Pointer.Base;
	  off = TM_PB.Tape.Addr_Pointer.Offset;
	  TM_Xfer_Addr.raw = seg;
	  TM_Xfer_Addr.raw <<= 4;
	  TM_Xfer_Addr.raw += off;
	  logmsgf(LT_TAPEMASTER,10,"TM: Xfer Addr = 0x%X\n",TM_Xfer_Addr.raw);
	  logmsgf(LT_TAPEMASTER,10,"TM: Buffer Size = %d\n",TM_PB.Tape.Buffer_Size);
	  TM_Xfer_Count = 0;
	  TM_Xfer_Index = 0;
	  TM_Controller_State = 30;
	}
	break;

      case 0x30: // Direct Write
	{
	  uint16_t seg,off;
	  // Write block to tape
	  logmsgf(LT_TAPEMASTER,10,"TM: DIRECT WRITE command\n");
	  seg = TM_PB.Tape.Addr_Pointer.Base;
	  off = TM_PB.Tape.Addr_Pointer.Offset;
	  TM_Xfer_Addr.raw = seg;
	  TM_Xfer_Addr.raw <<= 4;
	  TM_Xfer_Addr.raw += off;
	  logmsgf(LT_TAPEMASTER,10,"TM: Xfer Addr = 0x%X\n",TM_Xfer_Addr.raw);
	  logmsgf(LT_TAPEMASTER,10,"TM: Buffer Size = %d\n",TM_PB.Tape.Buffer_Size);
	  TM_Xfer_Count = 0;
	  TM_Xfer_Index = 0;
	  tape_reclen = TM_PB.Tape.Buffer_Size;
	  TM_Controller_State = 40;
	}
	break;

      case 0x34: // Rewind
        logmsgf(LT_TAPEMASTER,10,"TM: REWIND command\n");
	if(tape_fd > -1){
	  size_t rv;
	  rv = lseek(tape_fd,0,SEEK_SET);
	  TM_PB.Tape.DR_Status.raw = 0;
	  if(rv == 0){
	    tape_bot = 1;
	    tape_eot = 0;
	    tape_fm = 0;
	    tape_error = 0;
	    tape_reclen = 0;
	    TM_PB.Tape.DR_Status.Ready = 1;
	    TM_PB.Tape.DR_Status.Load_Point = 1;
	    TM_PB.Tape.DR_Status.Online = 1;
	  }else{
	    perror("tape:lseek");
	    TM_PB.Tape.CD_Status.Error = 0x10; // Tape Not Ready
	  }
	}else{
	  TM_PB.Tape.CD_Status.Error = 0x10; // Tape Not Ready
	}
	TM_PB.Tape.Return_Count = 0;
        TM_Controller_State = 85;
        break;

      case 0x40: // Write File Mark
        logmsgf(LT_TAPEMASTER,10,"TM: WRITE FILE MARK command\n");
        if(tape_fd > -1){
          ssize_t rv;
	  tape_reclen = 0;
	  rv = write(tape_fd,(unsigned char *)&tape_reclen,4);
	  if(rv == -1){
	    perror("tape:write");
	    TM_PB.Tape.CD_Status.Error = 0x0A; // IO error
	  }else{
	    if(rv < 4){
	      tape_eot = 1;
	      TM_PB.Tape.CD_Status.Error = 0x09; // Unexpected EOT
	    }
	  }
        }else{
          TM_PB.Tape.CD_Status.Error = 0x10; // Tape Not Ready
        }
        TM_PB.Tape.Return_Count = 0;
        TM_Controller_State = 85;
        break;

      case 0x44: // Search File Mark
	logmsgf(LT_TAPEMASTER,10,"TM: SEARCH FILE MARK command\n");
	TM_Controller_State = 35;
	break;

      case 0x48: // Space
	// Skip forward or backward X number of blocks
	// File marks count as a block
	{
	  uint16_t Actual_Records = 0;
	  logmsgf(LT_TAPEMASTER,10,"TM: SPACE command, %d records\n",TM_PB.Tape.Records);
	  if(TM_PB.Tape.Control.Reverse){
	    // Read backward
	    while(Actual_Records < TM_PB.Tape.Records){
	      tape_backspace_block();
	      Actual_Records++;
	    }
	  }else{
	    // Read forward
	    while(Actual_Records < TM_PB.Tape.Records){
	      tape_space_block();
	      Actual_Records++;
	    }
	  }
	  TM_PB.Tape.Return_Count = 0;
	  TM_Controller_State = 85;
	}
	break;

      case 0x60: // Streaming Read
	{
	  uint16_t seg,off;
	  logmsgf(LT_TAPEMASTER,10,"TM: STREAMING READ command\n");
	  // Each block has a header
	  seg = TM_PB.Tape.Addr_Pointer.Base;
	  off = TM_PB.Tape.Addr_Pointer.Offset;
	  TM_Xfer_Addr.raw = seg;
	  TM_Xfer_Addr.raw <<= 4;
	  TM_Xfer_Addr.raw += off;
	  logmsgf(LT_TAPEMASTER,10,"TM: Xfer Addr = 0x%X\n",TM_Xfer_Addr.raw);
	  logmsgf(LT_TAPEMASTER,10,"TM: Buffer Size = %d\n",TM_PB.Tape.Buffer_Size);
	  SB_Header_Addr.raw = TM_Xfer_Addr.raw;
	  TM_Xfer_Count = 0;
	  TM_Xfer_Index = 0;
	  tape_reclen = TM_PB.Tape.Buffer_Size;
	  TM_Controller_State = 29;
	}
	break;

      case 0x64: // Streaming Write
	{
	  uint16_t seg,off;
	  logmsgf(LT_TAPEMASTER,10,"TM: STREAMING WRITE command\n");
	  seg = TM_PB.Tape.Addr_Pointer.Base;
	  off = TM_PB.Tape.Addr_Pointer.Offset;
	  TM_Xfer_Addr.raw = seg;
	  TM_Xfer_Addr.raw <<= 4;
	  TM_Xfer_Addr.raw += off;
	  logmsgf(LT_TAPEMASTER,10,"TM: Xfer Addr = 0x%X\n",TM_Xfer_Addr.raw);
	  logmsgf(LT_TAPEMASTER,10,"TM: Buffer Size = %d\n",TM_PB.Tape.Buffer_Size);
          SB_Header_Addr.raw = TM_Xfer_Addr.raw;
	  TM_Xfer_Count = 0;
	  TM_Xfer_Index = 0;
	  SB_Gate_Wait_Time = 0;
	  TM_Controller_State = 39;
	}
	break;

      case 0x70: // Space File Mark
	// Skip forward or backward X number of blocks
	// File marks cause early termination
	{
	  uint16_t Actual_Records = 0;
	  int reclen = 0;
	  logmsgf(LT_TAPEMASTER,10,"TM: SPACE FILE MARK command, %d blocks\n",TM_PB.Tape.Records);
	  if(TM_PB.Tape.Control.Reverse){
	    // Read backward
	    while(Actual_Records < TM_PB.Tape.Records){
	      reclen = tape_backspace_block();
	      Actual_Records++;
	      if(reclen == 0){ break; }
	    }
	  }else{
	    // Read forward
	    while(Actual_Records < TM_PB.Tape.Records){
	      reclen = tape_space_block();
	      Actual_Records++;
	      if(reclen == 0){ break; }
	    }
	  }
	  TM_PB.Tape.Return_Count = 0;
	  TM_Controller_State = 85;
	}
	break;

      case 0x90: // Drive Reset
        logmsgf(LT_TAPEMASTER,10,"TM: DRIVE RESET command\n");
        TM_PB.Tape.DR_Status.raw = 0;
	if(tape_fd > -1){
	  TM_PB.Tape.DR_Status.Ready = 1;
	  TM_PB.Tape.DR_Status.Load_Point = tape_bot;
	  TM_PB.Tape.DR_Status.Online = 1;
	}
        TM_PB.Tape.Return_Count = 0;
        TM_Controller_State = 85;
        break;

      case 0x04: // Overlapped Rewind
      case 0x08: // Set Page Register
      case 0x0C: // Exchange
      case 0x10: // Buffered Read
      case 0x14: // Buffered Write
      case 0x18: // Buffered Edit
      case 0x1C: // Read Foreign Tape
      case 0x20: // NOP
      case 0x38: // Offline/Unload
      case 0x3C: // Direct Edit
      case 0x4C: // Erase Fixed Length
      case 0x50: // Erase Tape
      case 0x54: // Short Memory Test
      case 0x58: // Long Memory Test
      case 0x74: // Tape Assign
      case 0x80: // Block Move
      case 0x8C: // Set Retry
      case 0x94: // Search Multiple File Marks
      case 0x9C: // Clear Interrupt
      default:
        logmsgf(LT_TAPEMASTER,0,"TM: Unknown Command 0x%X\n",TM_PB.Command);
        TM_Controller_State = 0;
	// BV: no reason to DIE, for all those known but unimplemented commands
	// ld_die_rq = 1;
      }
      break;

    case 29: // Streaming Read: Await Block Gate and Obtain Block Header
      {
	int x = 1;
	TM_SB_Header.word[0] = multibus_word_read(TM_Xfer_Addr);
	if(TM_SB_Header.Ready != 1){ break; }
	logmsgf(LT_TAPEMASTER,10,"TM: Block Gate Open: 0x%X\n",TM_SB_Header.word[0]);
	TM_Xfer_Addr.raw += 2;
	while(x < 4){
	  TM_SB_Header.word[x] = multibus_word_read(TM_Xfer_Addr);
	  TM_Xfer_Addr.raw += 2;
	  x++;
	}
	// We now have block header. Mark it busy and not ready
	TM_SB_Header.Ready = 0;
	TM_SB_Header.Busy = 1;
	multibus_write(SB_Header_Addr,TM_SB_Header.byte[0]);
	// TM_Xfer_Addr points after the header. We can read the block now.
	TM_Xfer_Count = 0;
	TM_Xfer_Index = 0;
	TM_Controller_State++;
      }
      break;
    case 30: // Read Tape Block
      {
	int reclen;
	if(TM_PB.Tape.Control.Reverse){
	  reclen = tape_reverse_read();
	}else{
	  reclen = tape_read();
	}
	if(reclen == 0){
	  // Something happened
	  logmsgf(LT_TAPEMASTER,1,"TM: Something happened\n");
	  TM_PB.Tape.DR_Status.raw = 0;
	  TM_PB.Tape.DR_Status.Ready = 1;
	  TM_PB.Tape.DR_Status.Load_Point = 0;
	  TM_PB.Tape.DR_Status.Online = 1;
	  TM_PB.Tape.DR_Status.File_Mark = tape_fm;
	  TM_PB.Tape.DR_Status.Load_Point = tape_bot;
	  TM_PB.Tape.DR_Status.End_Of_Tape = tape_eot;
	  if(tape_error != 0){ TM_PB.Tape.CD_Status.Error = tape_error; }
	  if(TM_PB.Command == 0x60){
	    // Streaming
	    // Mark block done with fault
	    TM_SB_Header.Ready = 0;
	    TM_SB_Header.Busy = 0;
	    TM_SB_Header.Complete = 1;
	    TM_SB_Header.Fault = 1;
	    multibus_write(SB_Header_Addr,TM_SB_Header.byte[0]);
	  }
	  // Write back PB
	  TM_Controller_State = 85;
          break;
	}
	// We got data!
	TM_Controller_State++;
      }
      break;
    case 31: // Read Tape Block: Copy Data
      if(TM_Xfer_Count < tape_reclen){
	if(TM_PB.Command == 0x60){
	  // Streaming
	  if(TM_Xfer_Index < TM_SB_Header.Byte_Count){
	    multibus_write(TM_Xfer_Addr,tape_block[TM_Xfer_Index]);
	  }
	}else{
	  // Not Streaming
	  if(TM_Xfer_Index < TM_PB.Tape.Buffer_Size){
	    multibus_write(TM_Xfer_Addr,tape_block[TM_Xfer_Index]);
	  }
	}
	TM_Xfer_Addr.raw++;
	TM_Xfer_Count++;
	TM_Xfer_Index++;
	break;
      }
      logmsgf(LT_TAPEMASTER,10,"TM: Block Read Done\n");
      if(TM_PB.Command == 0x60){
	// Mark block done.
	TM_SB_Header.Ready = 0;
	TM_SB_Header.Busy = 0;
	TM_SB_Header.Complete = 1;
	multibus_write(SB_Header_Addr,TM_SB_Header.byte[0]);
	// Is it the last block?
	if(TM_SB_Header.Last_Block != 1){
	  // No, follow pointer
	  uint16_t seg,off;
	  logmsgf(LT_TAPEMASTER,10,"TM: Next Streaming Read block!\n");
	  // Each block has a header
	  seg = TM_SB_Header.Pointer.Base;
	  off = TM_SB_Header.Pointer.Offset;
	  TM_Xfer_Addr.raw = seg;
	  TM_Xfer_Addr.raw <<= 4;
	  TM_Xfer_Addr.raw += off;
	  logmsgf(LT_TAPEMASTER,10,"TM: Xfer Addr = 0x%X\n",TM_Xfer_Addr.raw);
	  logmsgf(LT_TAPEMASTER,10,"TM: Buffer Size = %d\n",TM_PB.Tape.Buffer_Size);
	  SB_Header_Addr.raw = TM_Xfer_Addr.raw;
	  TM_Xfer_Count = 0;
	  TM_Xfer_Index = 0;
	  TM_Controller_State = 29;
	  break;
	}
	// Yes, we are done.
	logmsgf(LT_TAPEMASTER,10,"TM: Last Streaming Block completed\n");
      }
      // Update drive status
      TM_PB.Tape.DR_Status.raw = 0;
      TM_PB.Tape.DR_Status.Ready = 1;
      TM_PB.Tape.DR_Status.Load_Point = 0;
      TM_PB.Tape.DR_Status.Online = 1;
      TM_PB.Tape.DR_Status.File_Mark = tape_fm;
      // And PB status stuff
      if(tape_reclen > TM_PB.Tape.Buffer_Size){
	TM_PB.Tape.Return_Count = TM_PB.Tape.Buffer_Size;
      }else{
	TM_PB.Tape.Return_Count = tape_reclen;
      }
      TM_PB.Tape.Records = tape_reclen;
      // Write back PB
      TM_Controller_State = 85;
      // ld_die_rq = 1;
      break;

    case 35: // Search File Mark
      {
        int reclen;
	if(TM_PB.Tape.Control.Reverse){
	  reclen = tape_reverse_read();
	}else{
	  reclen = tape_read();
	}
        if(reclen == 0){
          // Something happened
	  TM_PB.Tape.DR_Status.raw = 0;
	  TM_PB.Tape.DR_Status.Ready = 1;
	  TM_PB.Tape.DR_Status.Online = 1;
	  TM_PB.Tape.DR_Status.File_Mark = tape_fm;
	  TM_PB.Tape.DR_Status.Load_Point = tape_bot;
	  TM_PB.Tape.DR_Status.End_Of_Tape = tape_eot;
	  if(tape_error != 0 && tape_error != 0x15){
	    TM_PB.Tape.CD_Status.Error = tape_error;
	  }else{
	    tape_error = 0; // Ensure clobber
	  }
          logmsgf(LT_TAPEMASTER,1,"TM: Something happened! DR_Status 0x%X tape_error 0x%X CD_Status.Error = 0x%X\n",TM_PB.Tape.DR_Status.raw,tape_error,TM_PB.Tape.CD_Status.Error);
	  // Write back PB
	  TM_Controller_State = 85;
          break;
        }
        // We got data, ignore it and come back
      }
      break;

    case 39: // Streaming Write: Await Block Gate and Obtain Block Header
      {
        int x = 1;
	SB_Gate_Wait_Time++;
        TM_SB_Header.word[0] = multibus_word_read(TM_Xfer_Addr);
	if(SB_Gate_Wait_Time >= 100){
	  SB_Gate_Wait_Time = 0;
	  logmsgf(LT_TAPEMASTER,10,"TM: Awaiting Block Gate Open: 0x%X\n",TM_SB_Header.word[0]);
	}
        if(TM_SB_Header.Ready != 1){ break; }
        logmsgf(LT_TAPEMASTER,10,"TM: Block Gate Open: 0x%X\n",TM_SB_Header.word[0]);
        TM_Xfer_Addr.raw += 2;
        while(x < 4){
          TM_SB_Header.word[x] = multibus_word_read(TM_Xfer_Addr);
          TM_Xfer_Addr.raw += 2;
          x++;
        }
        // We now have block header. Mark it busy and not ready
        TM_SB_Header.Ready = 0;
        TM_SB_Header.Busy = 1;
        multibus_write(SB_Header_Addr,TM_SB_Header.byte[0]);
        // TM_Xfer_Addr points after the header. We can read the block data now.
        TM_Xfer_Count = 0;
        TM_Xfer_Index = 0;
        TM_Controller_State++;
      }
      break;

    case 40: // Write Tape Block
      // Fill the buffer
      if(TM_Xfer_Count < tape_reclen){
	uint8_t Byte = multibus_read(TM_Xfer_Addr);
	tape_block[TM_Xfer_Index] = Byte;
	TM_Xfer_Addr.raw++;
	TM_Xfer_Count++;
	TM_Xfer_Index++;
	break;
      }
      // Write the block
      if(TM_PB.Tape.Control.Reverse){
	logmsgf(LT_TAPEMASTER,0,"TM: Reverse write?\n");
	ld_die_rq = 1;
      }else{
	tape_write();
      }
      logmsgf(LT_TAPEMASTER,10,"TM: Block Write Done\n");
      if(TM_PB.Command == 0x64){
	// Streaming
	// Write block size
	TM_SB_Header.Byte_Count = tape_reclen;
	TM_Xfer_Addr.raw = SB_Header_Addr.raw+2;
	multibus_word_write(TM_Xfer_Addr,TM_SB_Header.word[1]);
        // Mark block done.
        TM_SB_Header.Ready = 0;
        TM_SB_Header.Busy = 0;
        TM_SB_Header.Complete = 1;
	if(tape_error != 0){
	  TM_SB_Header.Fault = 1;
	}
        multibus_write(SB_Header_Addr,TM_SB_Header.byte[0]);
        // Is it the last block?
        if(TM_SB_Header.Last_Block != 1 && tape_error == 0){
          // No, follow pointer
          uint16_t seg,off;
          logmsgf(LT_TAPEMASTER,10,"TM: Next Streaming Write block!\n");
          // Each block has a header
          seg = TM_SB_Header.Pointer.Base;
          off = TM_SB_Header.Pointer.Offset;
          TM_Xfer_Addr.raw = seg;
          TM_Xfer_Addr.raw <<= 4;
          TM_Xfer_Addr.raw += off;
          logmsgf(LT_TAPEMASTER,10,"TM: Xfer Addr = 0x%X\n",TM_Xfer_Addr.raw);
          SB_Header_Addr.raw = TM_Xfer_Addr.raw;
          TM_Xfer_Count = 0;
          TM_Xfer_Index = 0;
	  SB_Gate_Wait_Time = 0;
          TM_Controller_State = 39;
          break;
        }
        // Yes, we are done.
        logmsgf(LT_TAPEMASTER,10,"TM: Last Streaming Block completed\n");
      }
      // Update drive status
      TM_PB.Tape.DR_Status.raw = 0;
      TM_PB.Tape.DR_Status.Ready = 1;
      TM_PB.Tape.DR_Status.Online = 1;
      // And PB status stuff
      TM_PB.Tape.Return_Count = tape_reclen;
      // Write back PB
      TM_Controller_State = 85;
      break;

    case 85: // Do completion actions for tape PB
      TM_PB.Tape.CD_Status.Complete = 1;
      TM_Controller_State++; // Fall into...
    case 86: // Completion actions finished, write back PB
      TM_Xfer_Addr.raw = TM_PB_Addr;
      TM_Xfer_Index = 0;
      if(TM_PB.Command == 0x80){
        // BLOCK MOVE PB
        TM_Xfer_Count = 28;
      }else{
        if(TM_PB.Command == 0x0C){
          // EXCHANGE PB
          TM_Xfer_Count = 22;
        }else{
          // TAPE PB
          TM_Xfer_Count = 22;
        }
      }
      TM_Controller_State++; // Fall into...
    case 87: // Write back PB
      logmsgf(LT_TAPEMASTER,10,"TM: PB SDU Addr 0x%X = 0x%X\n",TM_Xfer_Addr.raw,TM_PB.byte[TM_Xfer_Index]);
      multibus_write(TM_Xfer_Addr,TM_PB.byte[TM_Xfer_Index]);
      TM_Xfer_Index++;
      TM_Controller_State = 90; // Go to loop
      // Fall into
    case 90: // Loop
      if(TM_Xfer_Index < TM_Xfer_Count){
        TM_Xfer_Addr.raw++;
        TM_Controller_State = 87;
        break;
      }
      // All done!
      TM_Controller_State = 94;
      break;
    case 94: // Perform completion actions
      if(TM_PB.Tape.Control.Link != 0){
        logmsgf(LT_TAPEMASTER,0,"TM: LINK processing not implemented\n");
        TM_Controller_State = 0;
        ld_die_rq = 1;
        break;
      }else{
        if(TM_PB.Tape.Control.Interrupt != 0){
          if(TM_PB.Tape.Control.Mailbox != 0){
            logmsgf(LT_TAPEMASTER,0,"TM: MAILBOX INTERRUPT processing not implemented\n");
            TM_Controller_State = 0;
            ld_die_rq = 1;
            break;
          }else{
            logmsgf(LT_TAPEMASTER,10,"TM: MULTIBUS INTERRUPT\n");
	    multibus_interrupt(2);
          }
        }
      }
      TM_Controller_State++;
      /* falls through */
    case 95: // Operation Completed - Open the gate!
      TM_Xfer_Addr.raw = TM_CCB_Addr+1;
      logmsgf(LT_TAPEMASTER,10,"TM: Gate Addr = 0x%X\n",TM_Xfer_Addr.raw);
      multibus_write(TM_Xfer_Addr,0x00);
      logmsgf(LT_TAPEMASTER,10,"TM: Operation completed, stopping\n");
      TM_Controller_State = 0;
      // multibus_interrupt(2);
      break;

    default:
      logmsgf(LT_TAPEMASTER,0,"TM: Unknown execution state %d\n",TM_Controller_State);
      ld_die_rq=1;
    }
  }
}

// Tapemaster controller execution thread
void *tm_thread(void *arg __attribute__ ((unused))){
  while(ld_die_rq == 0){
    while(TM_Controller_State == 0 && ld_die_rq == 0){ usleep(1000); } // Wait for something to do.
    // We left the sleep loop! Do we have something to do?
    if(TM_Controller_State > 0){
      // Do it.
      tapemaster_clock_pulse();
    }
  }
  // If we got here, we are dying, so go and die.
  return(NULL);
}
