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

/* Interphase 2181 */

#include "config.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#ifdef HAVE_YAML_H
#include <yaml.h>
#endif

#include "ld.h"
#include "nubus.h"
#include "sdu.h"

// SMD controller command register
typedef union rSMD_RCmd_Reg {
  uint8_t raw;
  struct {
    uint8_t Go:1;
    uint8_t Clear_Int:1;
    uint8_t Padding:3;
    uint8_t Bus_Word_Mode:1; // 0 = byte mode, 1 = word mode
  } __attribute__((packed));
} SMD_RCmd_Reg;

// SMD controller status register
typedef union rSMD_RStatus_Reg {
  uint8_t raw;
  struct {
    uint8_t Busy:1;
    uint8_t Int_Pending:1;
    uint8_t Padding:2;
    uint8_t Unit1Ready:1;
    uint8_t Unit2Ready:1;
    uint8_t Unit3Ready:1;
    uint8_t Unit4Ready:1;
  } __attribute__((packed));
} SMD_RStatus_Reg;

// SMD controller IOPB Base register
typedef union rSMD_IOPB_Base_Reg {
  uint32_t raw;
  uint8_t byte[4];
} SMD_IOPB_Base_Reg;

// SMD controller IOPB
typedef union rSMD_IOPB {
  uint32_t word[6];
  uint16_t hword[12];
  uint8_t byte[24];
  struct {
    uint8_t  Command;
    uint8_t  Buffer_WordMode:1; // 0 = byte, 1 = word
    uint8_t  Buffer_RelativeMode:1;
    uint8_t  Padding0:1;
    uint8_t  Reserve:1; // 1 = drive reserved after cmd complete
    uint8_t  IOPB_WordMode:1;
    uint8_t  IOPB_RelativeMode:1;
    uint8_t  Padding1:1;
    uint8_t  IOPB_Link:1;
    uint8_t  Status;
    uint8_t  Error;
    uint8_t  Unit;
    uint8_t  Head;
    uint16_t Cylinder;
    uint16_t Sector;
    uint16_t SectorCount;
    uint8_t  DMA_Burst_Size;
    uint32_t Buffer_Address:24;
    uint16_t IO_Address; // Should be equal to address of controller?
    uint16_t IO_Segment;
    uint8_t  Padding2;
    uint32_t Next_IOPB:24;
  } __attribute__((packed));
} SMD_IOPB_U;

// INITIALIZE command UIB
typedef union rSMD_UIB {
  uint8_t byte[17]; // Yes, 17 bytes, even though it occupies 18
  uint16_t hword[9];
  struct {
    uint8_t Tracks;  // Per Cylinder
    uint8_t Sectors; // Per Track
    uint16_t Sector_Size;
    uint16_t Gap_Size;
    uint8_t Interleave;
    uint8_t Retries;
    uint8_t ECC_Enabled;
    uint8_t Reseek_Enabled;
    uint8_t Replace_Enabled; // Move Bad Blocks
    uint8_t IncByHead_Enabled; // Increment By Head
    uint8_t Dual_Port;
    uint8_t Int_On_Status_Change;
    uint8_t Spiral;
    uint16_t Spare;
  } __attribute__((packed));
} SMD_UIB_U;

uint8_t SMD_BUFFER_RAM[1024];
SMD_RStatus_Reg SMD_RStatus;
SMD_RCmd_Reg SMD_RCmd;
SMD_IOPB_Base_Reg SMD_IOPB_Base;
SMD_IOPB_U SMD_IOPB;
volatile int SMD_Go = 0;
mbAddr SMD_Xfer_Addr;
nuAddr NB_Addr;
nuData NB_Data;
int SMD_Xfer_Mode = 0;
int SMD_Xfer_Count = 0;
int SMD_Xfer_Size = 0;
int SMD_Sector_Counter = 0;
int SMD_Burst_Counter = 0;
uint32_t SMD_LBA;
uint32_t SMD_Sector;
int SDU_disk_trace=0;

// Disk sharing
int SDU_Shared_Disk_Mode = 0;
share_struct_qs * share_struct = NULL;
unsigned int Active_SIOPB = 0;
i8086_Addr Share_i8086_Addr;
mbAddr Share_Xfer_Addr;
nuAddr Share_NB_Addr;
uint32_t Share_Runme_Addr;
int SMD_Share_Interrupt = 0;

// Disk storage interface
int disk_fd[4] = { -1,-1,-1,-1 }; // fd of disk file
SMD_UIB_U SMD_UIB[4];
off_t seek_res; // Result of seek operations
ssize_t io_res; // Result of read/write operations
int SMD_Retries; // Retry counter

// Externals
extern volatile int ld_die_rq;
// extern int disk_geometry_sph;
// extern int disk_geometry_spc;

// Filenames
char disk_fn[4][64] = { "disks/disk.img",{0},{0},{0} };

int smd_init(){
  int x=0,y=0;
  while(x < 4){
    if(disk_fn[x][0] != 0){
      disk_fd[x] = open(disk_fn[x],O_RDWR);
      if(disk_fd[x] < 0){
	perror("Disk:open");
	disk_fd[x] = -1;
      }else{
	y++;
      }
    }
    x++;
  }
  return(y);
}

void smd_reset(){
  SMD_RStatus.raw = 0;
  if(disk_fd[0] > 0){ SMD_RStatus.Unit1Ready = 1; }
  if(disk_fd[1] > 0){ SMD_RStatus.Unit2Ready = 1; }
  if(disk_fd[2] > 0){ SMD_RStatus.Unit3Ready = 1; }
  if(disk_fd[3] > 0){ SMD_RStatus.Unit4Ready = 1; }
}

// SMD read operation
void smd_exec_read_command(){
  // READ SETUP
  // PC uses 512 byte sectors, Lambda uses 1024
  if(disk_fd[SMD_IOPB.Unit] < 0){
    logmsgf(LT_SMD,1,"SMD: UNIT %d NOT READY ERROR\n",SMD_IOPB.Unit);
    SMD_IOPB.Error = 0x10; // DISK NOT READY
    SMD_IOPB.Status = 0x82; // OPERATION FAILED
    // Return and free IOPB
    return;
  }
  SMD_Sector = (SMD_IOPB.Cylinder*(SMD_UIB[SMD_IOPB.Unit].Sectors*SMD_UIB[SMD_IOPB.Unit].Tracks))+
    (SMD_IOPB.Head*SMD_UIB[SMD_IOPB.Unit].Sectors)+
    SMD_IOPB.Sector;
  SMD_Retries = 0;
  SMD_Sector_Counter = 0;
  SMD_Xfer_Addr.raw = SMD_IOPB.Buffer_Address;
  SMD_Xfer_Size = SMD_IOPB.DMA_Burst_Size;
  // DISK READ SECTOR
  switch(SMD_IOPB.Unit){
  case 0:
    SMD_RStatus.Unit1Ready = 0; break; // Drive is busy
  case 1:
    SMD_RStatus.Unit2Ready = 0; break; // Drive is busy
  case 2:
    SMD_RStatus.Unit3Ready = 0; break; // Drive is busy
  case 3:
    SMD_RStatus.Unit4Ready = 0; break; // Drive is busy
  }
  // Reposition the file pointer.
  seek_res = lseek(disk_fd[SMD_IOPB.Unit],(SMD_Sector*0x400),SEEK_SET);
  if(seek_res < 0){
    // Seek error!
    logmsgf(LT_SMD,1,"SMD: SEEK ERROR!\n");
    SMD_IOPB.Error = 0x12; // SEEK ERROR
    SMD_IOPB.Status = 0x82; // OPERATION FAILED
    return;
  }
  while(SMD_Sector_Counter < SMD_IOPB.SectorCount){
    // Read in a sector.
    SMD_Retries = 0;
    io_res = 0;
    while(io_res == 0){
      io_res = read(disk_fd[SMD_IOPB.Unit],SMD_BUFFER_RAM,1024);
      // DISK READ SECTOR: OPERATION COMPLETE
      if(io_res < 0){
	// There was an error
	// FIXME: USE MAX RETRY COUNT FROM UIB
	if(SMD_Retries < 3){
	  // retry the operation
	  SMD_Retries++;
	  io_res = 0;
	  continue;
	}else{
	  // Fail
	  logmsgf(LT_SMD,1,"SMD: READ ERROR!\n");
	  SMD_IOPB.Error = 0x23; // UNCORRECTABLE READ ERROR
	  SMD_IOPB.Status = 0x82; // OPERATION FAILED
	  return;
	}
      }
    }
    // logmsgf(LT_SMD,,"SMD: Read completed!\n");
    switch(SMD_IOPB.Unit){
    case 0:
      SMD_RStatus.Unit1Ready = 1; break; // Drive is busy
    case 1:
      SMD_RStatus.Unit2Ready = 1; break; // Drive is busy
    case 2:
      SMD_RStatus.Unit3Ready = 1; break; // Drive is busy
    case 3:
      SMD_RStatus.Unit4Ready = 1; break; // Drive is busy
    }
    // Op done! Setup transfer
    SMD_Burst_Counter = 0;
    SMD_Xfer_Count = 0;
    SMD_Xfer_Mode = SMD_IOPB.Buffer_WordMode;
    if(SMD_Xfer_Mode == 1 && ((SMD_Xfer_Addr.raw&0x01) != 0)){ SMD_Xfer_Mode = 0; } // Not on word boundary
    // Burst writing loop
    while(SMD_Burst_Counter*SMD_IOPB.DMA_Burst_Size < 1024){
      // Write Loop
      while(SMD_Xfer_Count < SMD_Xfer_Size){
	uint16_t BurstOffset = SMD_Xfer_Size*SMD_Burst_Counter;
	if(SMD_Xfer_Mode == 1 && (SMD_Xfer_Size-SMD_Xfer_Count) > 1){
	  multibus_word_write(SMD_Xfer_Addr,*(uint16_t *)(SMD_BUFFER_RAM+(BurstOffset+SMD_Xfer_Count)));
	  SMD_Xfer_Count += 2;
	  SMD_Xfer_Addr.raw += 2;
	}else{
	  multibus_write(SMD_Xfer_Addr,SMD_BUFFER_RAM[BurstOffset+SMD_Xfer_Count]);
	  SMD_Xfer_Count++;
	  SMD_Xfer_Addr.raw++;
	}
      }
      // Done with this burst.
      SMD_Xfer_Count = 0;
      SMD_Burst_Counter++;
    }
    // Sector completed!
    SMD_Sector_Counter++;
    // Next sector!
    SMD_Sector++;
  }
  // Operation completed
  if(SDU_disk_trace){
    logmsgf(LT_SMD,10,"SMD: READ OPERATION COMPLETE: 0x%X bursts, 0x%X sectors of %X completed.\n",
	    SMD_Burst_Counter,SMD_Sector_Counter,SMD_IOPB.SectorCount);
  }
  SMD_IOPB.Status = 0x80; // OPERATION COMPLETED SUCCESSFULLY
}

// SMD write operation
void smd_exec_write_command(){
  // WRITE SETUP
  // PC uses 512 byte sectors
  if(disk_fd[SMD_IOPB.Unit] < 0){
    logmsgf(LT_SMD,1,"SMD: UNIT %d NOT READY ERROR\n",SMD_IOPB.Unit);
    SMD_IOPB.Error = 0x10; // DISK NOT READY
    SMD_IOPB.Status = 0x82; // OPERATION FAILED
    return;
  }
  SMD_Sector = (SMD_IOPB.Cylinder*(SMD_UIB[SMD_IOPB.Unit].Sectors*SMD_UIB[SMD_IOPB.Unit].Tracks))+
    (SMD_IOPB.Head*SMD_UIB[SMD_IOPB.Unit].Sectors)+
    SMD_IOPB.Sector;
  SMD_Xfer_Count = 0;
  SMD_Sector_Counter = 0;
  SMD_Xfer_Addr.raw = SMD_IOPB.Buffer_Address;
  SMD_Xfer_Size = SMD_IOPB.DMA_Burst_Size;

  // Write sectors
  while(SMD_Sector_Counter < SMD_IOPB.SectorCount){
    SMD_Burst_Counter = 0;
    SMD_Retries = 0;
    // Read a sector worth of data from host
    while(SMD_Burst_Counter*SMD_IOPB.DMA_Burst_Size < 1024){
      while(SMD_Xfer_Count < SMD_Xfer_Size){
	uint16_t BurstOffset = SMD_Xfer_Size*SMD_Burst_Counter;
	SMD_BUFFER_RAM[BurstOffset+SMD_Xfer_Count] = multibus_read(SMD_Xfer_Addr);
	SMD_Xfer_Count++; SMD_Xfer_Addr.raw++;
      }
      // Done with this burst.
      // logmsgf(LT_SMD,,"SMD: DMA BURST 0x");
      // writeH32(SMD_Burst_Counter);
      // logmsgf(LT_SMD,," COMPLETE\n");
      SMD_Xfer_Count = 0;
      SMD_Burst_Counter++;
    }
    // Sector obtained! Write it.
    // WRITE SECTOR
    switch(SMD_IOPB.Unit){
    case 0:
      SMD_RStatus.Unit1Ready = 0; break; // Drive is busy
    case 1:
      SMD_RStatus.Unit2Ready = 0; break; // Drive is busy
    case 2:
      SMD_RStatus.Unit3Ready = 0; break; // Drive is busy
    case 3:
      SMD_RStatus.Unit4Ready = 0; break; // Drive is busy
    }
    // logmsgf(LT_SMD,,"SMD: Writing 2 blocks at LBA 0x");
    // writeH32(SMD_LBA);
    // logmsgf(LT_SMD,,"\n");
    // Reposition the file pointer.
    seek_res = lseek(disk_fd[SMD_IOPB.Unit],(SMD_Sector*0x400),SEEK_SET);
    if(seek_res < 0){
      // Seek error!
      logmsgf(LT_SMD,1,"SMD: SEEK ERROR!\n");
      SMD_IOPB.Error = 0x12; // SEEK ERROR
      SMD_IOPB.Status = 0x82; // OPERATION FAILED
      return;
    }
    io_res = 0;
    while(io_res == 0){
      io_res = write(disk_fd[SMD_IOPB.Unit],SMD_BUFFER_RAM,1024);
      // DISK WRITE SECTOR: OPERATION COMPLETE
      if(io_res < 0){
	// There was an error
	// FIXME: USE MAX RETRY COUNT FROM UIB
	if(SMD_Retries < 3){
	  // retry the operation
	  SMD_Retries++;
	  io_res = 0;
	  continue;
	}else{
	  // Fail
	  logmsgf(LT_SMD,1,"SMD: WRITE ERROR!\n");
	  SMD_IOPB.Error = 0x1E; // DRIVE FAULTED
	  SMD_IOPB.Status = 0x82; // OPERATION FAILED
	  return;
	}
      }
    }
    // logmsgf(LT_SMD,,"SMD: Write completed!\n");
    switch(SMD_IOPB.Unit){
    case 0:
      SMD_RStatus.Unit1Ready = 1; break; // Drive is busy
    case 1:
      SMD_RStatus.Unit2Ready = 1; break; // Drive is busy
    case 2:
      SMD_RStatus.Unit3Ready = 1; break; // Drive is busy
    case 3:
      SMD_RStatus.Unit4Ready = 1; break; // Drive is busy
    }
    // Sector completed!
    SMD_Sector_Counter++;
    SMD_Sector++;
  }
  // Done with write command!
  if(SDU_disk_trace){
    logmsgf(LT_SMD,10,"SMD: WRITE OPERATION COMPLETE: 0x%X bursts, 0x%X sectors of %X completed.\n",
	    SMD_Burst_Counter,SMD_Sector_Counter,SMD_IOPB.SectorCount);
  }
  SMD_IOPB.Status = 0x80; // OPERATION COMPLETED SUCCESSFULLY
}

void smd_exec_initialize_command(){
  // SETUP FOR UIB READ
  SMD_Xfer_Addr.raw = SMD_IOPB.Buffer_Address;
  SMD_Xfer_Size = 17;
  SMD_Xfer_Count = 0;
  SMD_Xfer_Mode = SMD_IOPB.Buffer_WordMode;
  if(SMD_Xfer_Mode == 1 && ((SMD_Xfer_Addr.raw&0x01) != 0)){
    SMD_Xfer_Mode = 0; // Not on word boundary
  }
  // OBTAIN UIB
  while(SMD_Xfer_Count < SMD_Xfer_Size){
    if(SMD_Xfer_Mode == 1){
      SMD_UIB[SMD_IOPB.Unit].hword[SMD_Xfer_Count>>1] =
	multibus_word_read(SMD_Xfer_Addr);
      SMD_Xfer_Count += 2; SMD_Xfer_Addr.raw += 2;
    }else{
      SMD_UIB[SMD_IOPB.Unit].byte[SMD_Xfer_Count] =
	multibus_read(SMD_Xfer_Addr);
      SMD_Xfer_Count++; SMD_Xfer_Addr.raw++;
    }
  }
  // PARSE UIB
  logmsgf(LT_SMD,10,"SMD: Unit %d: %d tracks per cylinder, %d sectors per track\n",
	  SMD_IOPB.Unit,
	  SMD_UIB[SMD_IOPB.Unit].Tracks,
	  SMD_UIB[SMD_IOPB.Unit].Sectors);
  SMD_IOPB.Status = 0x80; // OPERATION COMPLETED SUCCESSFULLY
}

// SMD controller execution thread
void *smd_thread(void *arg __attribute__ ((unused))){
  while(ld_die_rq == 0){
    while(SMD_Go == 0 && ld_die_rq == 0){ usleep(1000); } // Wait for something to do.
    if(ld_die_rq != 0){ break; } // Exit if we are dying.

    if(SDU_disk_trace){
      logmsgf(LT_SMD,10,"SMD: Controller Go! SMD_IOPB_Base = 0x%X\n",SMD_IOPB_Base.raw);
    }
    // Set up transfer
    SMD_Xfer_Addr.raw = SMD_IOPB_Base.raw;
    SMD_Xfer_Size = 24;
    SMD_Xfer_Count = 0;
    SMD_Xfer_Mode = SMD_RCmd.Bus_Word_Mode;
    if(SMD_Xfer_Mode == 1 && ((SMD_Xfer_Addr.raw&0x01) != 0)){ SMD_Xfer_Mode = 0; } // Not on word boundary
    // Do read
    while(SMD_Xfer_Count < SMD_Xfer_Size){
      if(SMD_Xfer_Mode == 1){
	SMD_IOPB.hword[SMD_Xfer_Count>>1] = multibus_word_read(SMD_Xfer_Addr);
	SMD_Xfer_Count += 2; SMD_Xfer_Addr.raw += 2;
      }else{
	SMD_IOPB.byte[SMD_Xfer_Count] = multibus_read(SMD_Xfer_Addr);
	SMD_Xfer_Count++; SMD_Xfer_Addr.raw++;
      }
    }
    // Parse and update IOPB
    // Swap bytes of words
    // This happens regardless of bus width
    {
      uint8_t tmp = SMD_IOPB.byte[6];
      // Cyl
      SMD_IOPB.byte[6] = SMD_IOPB.byte[7];
      SMD_IOPB.byte[7] = tmp; tmp = SMD_IOPB.byte[8];
      // Sect
      SMD_IOPB.byte[8] = SMD_IOPB.byte[9];
      SMD_IOPB.byte[9] = tmp; tmp = SMD_IOPB.byte[10];
      // Scnt
      SMD_IOPB.byte[10] = SMD_IOPB.byte[11];
      SMD_IOPB.byte[11] = tmp; tmp = SMD_IOPB.byte[13];
      // Buffer Addr
      SMD_IOPB.byte[13] = SMD_IOPB.byte[15];
      SMD_IOPB.byte[15] = tmp; tmp = SMD_IOPB.byte[16];
      // IOA
      SMD_IOPB.byte[16] = SMD_IOPB.byte[17];
      SMD_IOPB.byte[17] = tmp; tmp = SMD_IOPB.byte[18];
      // Seg
      SMD_IOPB.byte[18] = SMD_IOPB.byte[19];
      SMD_IOPB.byte[19] = tmp; tmp = SMD_IOPB.byte[21];
      // Link
      SMD_IOPB.byte[21] = SMD_IOPB.byte[23];
      SMD_IOPB.byte[23] = tmp;
    }
    if(SDU_disk_trace){
      logmsgf(LT_SMD,10,"SMD: IOPB read completed!\n");
      logmsgf(LT_SMD,10,"Command: 0x%X\n",SMD_IOPB.Command);
      logmsgf(LT_SMD,10,"BUF_WM %X BUF_RM %X RSV %X IOPB_WM %X IOPB_RM %X IOPB_Link %X\n",
	      SMD_IOPB.Buffer_WordMode,SMD_IOPB.Buffer_RelativeMode,SMD_IOPB.Reserve,
	      SMD_IOPB.IOPB_WordMode,SMD_IOPB.IOPB_RelativeMode,SMD_IOPB.IOPB_Link);
      logmsgf(LT_SMD,10,"Status: 0x%X Error 0x%X Unit 0x%X Head 0x%X\n",
	      SMD_IOPB.Status,SMD_IOPB.Error,SMD_IOPB.Unit,SMD_IOPB.Head);
      logmsgf(LT_SMD,10,"Cylinder 0x%X Sector 0x%X SectorCount 0x%X DMA_Burst_Size 0x%X\n",
	      SMD_IOPB.Cylinder,SMD_IOPB.Sector,SMD_IOPB.SectorCount,SMD_IOPB.DMA_Burst_Size);
      logmsgf(LT_SMD,10,"Buffer_Address 0x%X IO_Address 0x%X IO_Segment 0x%X Next_IOPB 0x%X\n",
	      SMD_IOPB.Buffer_Address,SMD_IOPB.IO_Address,SMD_IOPB.IO_Segment,SMD_IOPB.Next_IOPB);
    }
    // Halt for investigation of these
    // if(SMD_IOPB.Unit != 0){ logmsgf(LT_SMD,,"SMD: Not unit 0?\n"); ld_die_rq = 1; }
    // if(SMD_IOPB.IOPB_Link != 0){ logmsgf(LT_SMD,,"SMD: Link bit set?\n"); ld_die_rq = 1; }
    if(SMD_IOPB.IOPB_RelativeMode != 0 && SMD_IOPB.Buffer_RelativeMode){
      logmsgf(LT_SMD,0,"SMD: Relative mode?\n");
      ld_die_rq = 1;
      break;
    }
    // if(SMD_IOPB.Buffer_WordMode == 0){ logmsgf(LT_SMD,,"SMD: Buffer not word mode?\n"); ld_die_rq = 1; }
    // Set up transfer to write status
    SMD_Xfer_Addr.raw = SMD_IOPB_Base.raw+2;
    // Mark IOPB busy
    SMD_IOPB.Status = 0x81; // COMMAND BUSY
    // Write back status
    // nubus_io_request(VM_WRITE,0xFF,NB_Addr.raw,SMD_IOPB.word[0]);
    if(SMD_IOPB.IOPB_WordMode != 0 && ((SMD_Xfer_Addr.raw&0x01) == 0)){
      multibus_word_write(SMD_Xfer_Addr,SMD_IOPB.hword[1]); SMD_Xfer_Addr.raw += 2;
    }else{
      multibus_write(SMD_Xfer_Addr,SMD_IOPB.byte[2]); SMD_Xfer_Addr.raw++;
      multibus_write(SMD_Xfer_Addr,SMD_IOPB.byte[3]); SMD_Xfer_Addr.raw++;
    }

    // Process command
    switch(SMD_IOPB.Command){
    case 0x56: // Cold load / LAM does this for some reason.
      // TREATING THIS AS AN ERROR CORRUPTED MY DISK
      // Maybe it's read?
      logmsgf(LT_SMD,0,"SMD: LAM/COLD UNKNOWN COMMAND: Interpreting as READ\n");
      // Falls thru
    case 0x81: // READ
      smd_exec_read_command(); // Perform read
      break;
    case 0x82: // WRITE
      smd_exec_write_command();
      break;
    case 0x8F: // RESET
      logmsgf(LT_SMD,10,"SMD: DRIVE RESET\n");
      SMD_IOPB.Status = 0x80; // OPERATION COMPLETED SUCCESSFULLY
      break;
    case 0x87: // INITIALIZE
      logmsgf(LT_SMD,10,"SMD: INITIALIZE\n");
      // This wrote a UIB into the controller so it could talk to the drive.
      smd_exec_initialize_command();
      break;
    case 0x89: // RESTORE
      // Seek to track zero
      logmsgf(LT_SMD,10,"SMD: RESTORE\n");
      if(disk_fd[SMD_IOPB.Unit] < 0){
	logmsgf(LT_SMD,1,"SMD: UNIT %d NOT READY ERROR\n",SMD_IOPB.Unit);
	SMD_IOPB.Error = 0x10; // DISK NOT READY
	SMD_IOPB.Status = 0x82; // OPERATION FAILED
	// ld_die_rq = 1;
	break;
      }
      SMD_IOPB.Status = 0x80; // OPERATION COMPLETED SUCCESSFULLY
      break;
    case 0x83: // VERIFY FORMAT
      // Not sure what exactly this is supposed to do.
      // The data appears to be unused, or 2181 doesn't care if I don't touch it.
      // logmsgf(LT_SMD,,"SMD: VERIFY\n");
      if(disk_fd[SMD_IOPB.Unit] < 0){
	logmsgf(LT_SMD,1,"SMD: UNIT %d NOT READY ERROR\n",SMD_IOPB.Unit);
	SMD_IOPB.Error = 0x10; // DISK NOT READY
	SMD_IOPB.Status = 0x82; // OPERATION FAILED
	// ld_die_rq = 1;
	break;
      }
      SMD_IOPB.Status = 0x80; // OPERATION COMPLETED SUCCESSFULLY
      break;
    case 0x84: // FORMAT TRACK
    case 0x85: // MAP
    case 0x8A: // SEEK
    case 0x8B: // ZERO SECTOR
    default:
      logmsgf(LT_SMD,0,"SMD: Unknown command 0x%X\n",SMD_IOPB.Command);
      ld_die_rq = 1;
      break;
    }
    if(ld_die_rq != 0){ break; } // Bail immediately if we are dying.
    // Report command completion
    // Set up transfer to write status back
    SMD_Xfer_Addr.raw = SMD_IOPB_Base.raw+2;
    // Issue writes
    if(SMD_IOPB.IOPB_WordMode != 0 && ((SMD_Xfer_Addr.raw&0x01) == 0)){
      multibus_word_write(SMD_Xfer_Addr,SMD_IOPB.hword[1]); SMD_Xfer_Addr.raw += 2;
    }else{
      multibus_write(SMD_Xfer_Addr,SMD_IOPB.byte[2]); SMD_Xfer_Addr.raw++;
      multibus_write(SMD_Xfer_Addr,SMD_IOPB.byte[3]); SMD_Xfer_Addr.raw++;
    }
    // If the operation was a success and the link bit is set, we should follow it here.
    if(SMD_IOPB.Status == 0x80 && SMD_IOPB.IOPB_Link != 0){
      SMD_IOPB_Base.raw = SMD_IOPB.Next_IOPB;
      logmsgf(LT_SMD,10,"SMD: Following link to 0x%X\n",SMD_IOPB.Next_IOPB);
      continue; // Loop!
    }
    // Otherwise, set interrupt pending bit
    SMD_RStatus.Int_Pending = 1;
    // Stop the controller.
    SMD_Go = 0;
    // and generate the interrupt
    multibus_interrupt(4);
  }
  // If we got here, we are dying, so go and die.
  return(NULL);
}


/*
// SMD controller execution thread
void *smd_thread(void *arg __attribute__ ((unused))){
  while(ld_die_rq == 0){
    while(SMD_Controller_State == 0 && ld_die_rq == 0){ usleep(1000); } // Wait for something to do.
    // We left the sleep loop! Do we have something to do?
    if(SMD_Controller_State > 0){
      // Do it.
      smd_clock_pulse();
    }
  }
  // If we got here, we are dying, so go and die.
  return(NULL);
}
*/

uint8_t smd_read(uint8_t addr){
  switch(addr){
  case 0: // Status Reg
    return(SMD_RStatus.raw);
    break;
  default:
    logmsgf(LT_SMD,0,"SMD: Unknown read addr %d\n",addr);
    ld_die_rq=1;
  }
  return(0);
}

void smd_write(uint8_t addr,uint8_t data){
  switch(addr){
  case 0: // Command Reg
    SMD_RCmd.raw = data;
    if(SMD_RCmd.Clear_Int != 0){ SMD_RStatus.Int_Pending = 0; } // Clear the interrupt pending bit
    if(SMD_RCmd.Go != 0 && SMD_Go == 0){ SMD_Go = 1; } // Make controller go
    break;
  case 1: // IOPB Base (hi)
    SMD_IOPB_Base.byte[3] = 0;
    SMD_IOPB_Base.byte[2] = data;
    break;
  case 2: // IOPB Base (mid)
    SMD_IOPB_Base.byte[1] = data;
    break;
  case 4: // IOPB Base (lower)
    // Lisp does this; Don't know why.
    SMD_IOPB_Base.raw <<= 8; // Move the other two up
    // Falls thru
  case 3: // IOPB Base (lo)
    SMD_IOPB_Base.byte[0] = data;
    break;
  default:
    logmsgf(LT_SMD,0,"SMD: Unknown write addr %d\n",addr);
    ld_die_rq=1;
  }
}

#ifdef HAVE_YAML_H
int yaml_disk_sequence_loop(yaml_parser_t *parser){
  char key[128];
  char value[128];
  yaml_event_t event;
  int sequence_done = 0;
  int unit = 0;
  char fname[64];

  key[0] = 0;
  value[0] = 0;
  while(sequence_done == 0){
    if(!yaml_parser_parse(parser, &event)){
      if(parser->context != NULL){
        logmsgf(LT_SMD,0,"YAML: Parser error %d: %s %s\n", parser->error,parser->problem,parser->context);
      }else{
        logmsgf(LT_SMD,0,"YAML: Parser error %d: %s\n", parser->error,parser->problem);
      }
      return(-1);
    }
    switch(event.type){
    case YAML_NO_EVENT:
      logmsgf(LT_SMD,0,"No event?\n");
      break;
    case YAML_STREAM_START_EVENT:
    case YAML_DOCUMENT_START_EVENT:
      // logmsgf(LT_SMD,,"STREAM START\n");
      logmsgf(LT_SMD,0,"Unexpected stream/document start\n");
      break;
    case YAML_STREAM_END_EVENT:
    case YAML_DOCUMENT_END_EVENT:
      // logmsgf(LT_SMD,,"[End Document]\n");
      logmsgf(LT_SMD,0,"Unexpected stream/document end\n");
      break;
    case YAML_SEQUENCE_START_EVENT:
      logmsgf(LT_SMD,0,"Unexpected sequence start\n");
      return(-1);
      break;
    case YAML_MAPPING_START_EVENT:
      // Map entry start. Reinitialize.
      unit = 0;
      fname[0] = 0;
      break;
    case YAML_SEQUENCE_END_EVENT:
      // We are done
      sequence_done = 1;
      break;
    case YAML_MAPPING_END_EVENT:
      // Map entry end. Do it.
      if(unit > 3){ unit = 3; }
      if(unit < 0){ unit = 0; }
      strncpy(disk_fn[unit],fname,64);
      logmsgf(LT_SMD,0,"Using disk image %s for unit %d\n",disk_fn[unit],unit);
      break;
    case YAML_ALIAS_EVENT:
      logmsgf(LT_SMD,0,"Unexpected alias (anchor %s)\n", event.data.alias.anchor);
      return(-1);
      break;
    case YAML_SCALAR_EVENT:
      if(key[0] == 0){
        strncpy(key,(const char *)event.data.scalar.value,128);
      }else{
        strncpy(value,(const char *)event.data.scalar.value,128);
        if(strcmp(key,"unit") == 0){
          unit = atoi(value);
          goto value_done;
        }
        if(strcmp(key,"file") == 0){
          strncpy(fname,value,64);
          goto value_done;
        }
        logmsgf(LT_SMD,0,"disk: Unknown key %s (value %s)\n",key,value);
        return(-1);
        // Done
      value_done:
        key[0] = 0;
        break;
      }
      break;
    }
    yaml_event_delete(&event);
  }
  return(0);
}

int yaml_disk_mapping_loop(yaml_parser_t *parser){
  char key[128];
  char value[128];
  yaml_event_t event;
  int rv = 0;
  int mapping_done = 0;
  key[0] = 0;
  value[0] = 0;
  while(mapping_done == 0){
    if(!yaml_parser_parse(parser, &event)){
      if(parser->context != NULL){
        logmsgf(LT_SMD,0,"YAML: Parser error %d: %s %s\n", parser->error,parser->problem,parser->context);
      }else{
        logmsgf(LT_SMD,0,"YAML: Parser error %d: %s\n", parser->error,parser->problem);
      }
      return(-1);
    }
    switch(event.type){
    case YAML_NO_EVENT:
      logmsgf(LT_SMD,0,"No event?\n");
      break;
    case YAML_STREAM_START_EVENT:
    case YAML_DOCUMENT_START_EVENT:
      // logmsgf(LT_SMD,,"STREAM START\n");
      logmsgf(LT_SMD,0,"Unexpected stream/document start\n");
      break;
    case YAML_STREAM_END_EVENT:
    case YAML_DOCUMENT_END_EVENT:
      // logmsgf(LT_SMD,,"[End Document]\n");
      logmsgf(LT_SMD,0,"Unexpected stream/document end\n");
      break;
    case YAML_SEQUENCE_START_EVENT:
      if(strcmp(key,"units") == 0){
        rv = yaml_disk_sequence_loop(parser);
        goto seq_done;
      }
      logmsgf(LT_SMD,0,"Unexpected sequence key: %s\n",key);
      return(-1);
    seq_done:
      if(rv < 0){ return(rv); }
      key[0] = 0;
      break;
    case YAML_MAPPING_START_EVENT:
      logmsgf(LT_SMD,0,"Unexpected mapping start\n");
      return(-1);
      break;
    case YAML_SEQUENCE_END_EVENT:
      logmsgf(LT_SMD,0,"Unexpected sequence end\n");
      return(-1);
      break;
    case YAML_MAPPING_END_EVENT:
      mapping_done = 1;
      break;
    case YAML_ALIAS_EVENT:
      logmsgf(LT_SMD,0,"Unexpected alias (anchor %s)\n", event.data.alias.anchor);
      return(-1);
      break;
    case YAML_SCALAR_EVENT:
      if(key[0] == 0){
        strncpy(key,(const char *)event.data.scalar.value,128);
      }else{
        strncpy(value,(const char *)event.data.scalar.value,128);
        if(strcmp(key,"image") == 0){
	  int dsk = 0;
	  char *tok = strtok(value," \t\r\n");
	  if(tok != NULL){
	    int val = atoi(tok);
	    dsk = val;
	    tok = strtok(NULL," \t\r\n");
	    if(tok != NULL){
	      strncpy(disk_fn[dsk],tok,64);
	      logmsgf(LT_SMD,0,"Using disk image %s for unit %d\n",tok,dsk);
	    }
	  }else{
	    logmsgf(LT_SMD,0,"Missing disk image name\n");
	    return(-1);
	  }
	  goto value_done;
        }
        logmsgf(LT_SMD,0,"disk: Unknown key %s (value %s)\n",key,value);
        return(-1);
        // Done
      value_done:
        key[0] = 0;
        break;
      }
      break;
    }
    yaml_event_delete(&event);
  }
  return(0);
}

#endif
