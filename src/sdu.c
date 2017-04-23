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
#include <string.h>
#include <strings.h>

#include "ld.h"
#include "nubus.h"
#include "lambda_cpu.h"
#include "sdu.h"
#include "sdu_hw.h"
#include "tapemaster.h"
#include "3com.h"
#include "smd.h"

#define RAM_TOP 1024*64

/* Motorola MC146818 RTC */
uint32_t rtc_cycle_count;
uint32_t rtc_addr; // Address Register

uint8_t RTC_Counter[10];
// Counter map
#define RTC_SECONDS 0
#define RTC_SECONDS_ALARM 1
#define RTC_MINUTES 2
#define RTC_MINUTES_ALARM 3
#define RTC_HOURS 4
#define RTC_HOURS_ALARM 5
#define RTC_DAY_OF_WEEK 6
#define RTC_DATE_OF_MONTH 7
#define RTC_MONTH 8
#define RTC_YEAR 9

// Registers

typedef union rRTC_REGA_Reg {
  uint8_t byte;
  struct {
    uint8_t Rate_Select:4;
    uint8_t Divider_Select:3;
    uint8_t Update_In_Progress:1;
  } __attribute__((packed));
} RTC_REGA_Reg;

typedef union rRTC_REGB_Reg {
  uint8_t byte;
  struct {
    uint8_t DST_Enable:1;
    uint8_t Format:1; // 1 = 24 hours, 0 = 12 hours
    uint8_t Data_Mode:1; // 1 = binary, 0 = BCD
    uint8_t SQWE:1;
    uint8_t Update_Int_Enable:1;
    uint8_t Alarm_Int_Enable:1;
    uint8_t Periodic_Int_Enable:1;
    uint8_t Set:1;
  } __attribute__((packed));
} RTC_REGB_Reg;

typedef union rRTC_REGC_Reg {
  uint8_t byte;
  struct {
    uint8_t Spare:4;
    uint8_t Update_Ended:1;
    uint8_t Alarm:1;
    uint8_t Periodic:1;
    uint8_t IRQ:1;
  } __attribute__((packed));
} RTC_REGC_Reg;

typedef union rRTC_REGD_Reg {
  uint8_t byte;
  struct {
    uint8_t Spare:7;
    uint8_t Valid_RAM:1;
  } __attribute__((packed));
} RTC_REGD_Reg;

RTC_REGA_Reg RTC_REGA;
RTC_REGB_Reg RTC_REGB;
RTC_REGC_Reg RTC_REGC;
RTC_REGD_Reg RTC_REGD;

// User RAM
uint8_t RTC_RAM[50];

#ifdef BURR_BROWN
/* Burr-Brown Parallel Interface */
nuAddr BB_Remote_Addr;
nuData BB_Remote_Data;
uint8_t BB_Remote_Result;
nuData BB_Data;
uint8_t BB_Drive_Data_Lines;
uint8_t BB_Reg;
uint8_t BB_Mode_Reg;
#endif

/* Memories and registers */
uint8_t SDU_RAM[RAM_TOP];
uint8_t SDU_ROM[1024*64];
uint8_t CMOS_RAM[2048]; // 2K
MNAMap_Ent MNA_MAP[1024];

// 8088 stuff
i8259 PIC[3];
i8253 PIT[2];
extern volatile uint8_t sdu_rx_ptr;
extern volatile uint8_t sdu_rx_bot;
extern volatile uint8_t sdu_rx_buf[64];
extern volatile int sdu_conn_fd;
uint8_t sducons_tx_buf[64];
uint8_t sducons_tx_top = 0;
uint8_t sducons_tx_bot = 0;
uint32_t pit_cycle_counter = 0;
int sdu_nubus_enable = 0;
int sdu_multibus_enable = 0;
uint8_t nubus_timeout_reg = 0;

// Shared memory structure bases
uint32_t sysconf_base = 0;
uint32_t proc0_conf_base = 0;
uint32_t proc1_conf_base = 0;
uint32_t chaos_share_base[4];
uint32_t share_struct_base = 0;

// Disk sharing
extern share_struct_qs * share_struct;

// Externals
extern int ld_die_rq;
extern int cp_state[2];
extern struct lambdaState pS[2];
// Configuration items from kernel
extern int disk_geometry_sph;
// Band names
extern uint8_t load_band[2][5];
extern uint8_t page_band[2][5];
extern uint8_t ulod_band[2][5];
extern uint8_t lmfs_band[2][5];
// Chaos address
extern uint32_t chaos_address[2];
// Bootstrap processor
extern uint8_t bsp;

// Globals
int SDU_state=0;

// Temporary - for debugging
uint8_t SDU_RAM_trace = 0;
char sysconf_q_names[sizeof(system_configuration_qs)/4][64] = {
  "version_number",
  "size",
  "number_of_processors",
  "processor_block_size",
  "share_struct_pointer",
  "debug_level",
  "lock",
  "ethernet_owner",
  "tapemaster_owner",
  "mti_8_line_owner",
  "mti_16_line_owner",
  "quarter_inch_tape_owner",
  "sdu_serial_a_owner",
  "sdu_serial_b_owner",
  "share_tty_0",
  "share_tty_1",
  "share_tty_2",
  "grey_owner",
  "grey_slot",
  "number_of_share_ttys",
  "number_of_share_tty_pages",
  "global_shared_base",
  "global_shared_size",
  "excelan_owner",
  "excelan_2_owner",
  "shared_excelan_pointer",
  "excelan_2_initted",
  "sdu_interrupt_map",
  "tapemaster_base_multibus_map",
  "tapemaster_multibus_map_size",
  "titn_owner",
  "system_console",
  "sdu_nubus_base",
  "sdu_nubus_size",
  "multibus_tapemaster_parameter_block",
  "excelan_base_multibus_map_block",
  "excelan_multibus_map_size",
  "cmos_clock_chip_owner",
  "user_base_multibus_map",
  "user_multibus_map_size",
  "second_grey_owner",
  "second_grey_slot",
  "default_grey_owner",
  "default_second_grey_owner",
  "flavors_bus_link_owner",
  "flavors_bus_link_slot",
  "second_flavors_bus_link_owner",
  "second_flavors_bus_link_slot",
  "newboot_version_number",
  "sdu_rom_version_number",
  "burr_brown_owner",
  "second_burr_brown_owner",
  "interphase_2181_owner",
  "disk_unit_0_initialized",
  "disk_unit_1_initialized",
  "disk_unit_2_initialized",
  "disk_unit_3_initialized",
  "disk_unit_4_initialized",
  "disk_unit_5_initialized",
  "disk_unit_6_initialized",
  "disk_unit_7_initialized",
  "nubus_disk_owner",
  "newboot_idle_count",
  "chaos_sharedev_buffer_size_in_bytes",
  "lmi_debug_board_owner",
  "lmi_debug_board_slot",
  "second_lmi_debug_board_owner",
  "second_lmi_debug_board_slot"
};

char proc_conf_q_names[sizeof(processor_configuration_qs)/4][64] = {
  "sys_conf_ptr",
  "slot_number",
  "major_version",
  "minor_version",
  "starting_processor_switches",
  "share_runme",
  "share_slot",
  "share_type",
  "share_iopb",
  "share_interrupt_addr",
  "share_spare_1",
  "share_spare_2",
  "share_spare_3",
  "share_spare_4",
  "chaos_address",
  "send_chaos_share_dev",
  "rcv_chaos_share_dev",
  "memory_base_0",
  "memory_base_1",
  "memory_base_2",
  "memory_base_3",
  "memory_base_4",
  "memory_base_5",
  "memory_base_6",
  "memory_base_7",
  "memory_base_8",
  "memory_base_9",
  "memory_bytes_0",
  "memory_bytes_1",
  "memory_bytes_2",
  "memory_bytes_3",
  "memory_bytes_4",
  "memory_bytes_5",
  "memory_bytes_6",
  "memory_bytes_7",
  "memory_bytes_8",
  "memory_bytes_9",
  "vcmem_slot",
  "processor_type",
  "micro_band",
  "load_band",
  "paging_band",
  "file_band",
  "base_multibus_mapping_register",
  "boot_status",
  "chaos_share_0",
  "chaos_share_1",
  "chaos_share_2",
  "chaos_share_3",
  "chaos_share_4",
  "parity_enables",
  "vcmem_words_per_line",
  "number_of_multibus_maps",
  "boot_command",
  "boot_mode",
  "console",
  "console_baud_rate",
  "watchdog",
  "intmap_multibus_map",
  "n_aux_devs",
  "aux_dev_0",
  "aux_dev_1",
  "excelan_multibus_map_base",
  "excelan_multibus_map_size"
};

// Functions
void sdu_init(){
  // Kernel's RTC time
  extern uint32_t rtc_sec,rtc_min,rtc_hour;
  extern uint32_t rtc_dow,rtc_date,rtc_month;
  extern uint32_t rtc_year;
  // Clobber RAM
  bzero(SDU_RAM,RAM_TOP);
  // Initialize RTC
  RTC_REGA.Rate_Select = 2; // 32 KHz
  RTC_REGA.Divider_Select = 2; 
  RTC_REGB.Data_Mode = 1; // Binary counters
  RTC_REGB.SQWE = 0;
  RTC_REGB.Format = 1; // 24 hour format
  RTC_REGB.DST_Enable = 1; // DST Enabled
  rtc_cycle_count = 0;
  // Read time from kernel, which should have already pulled it from the rtc (or unix)
  RTC_Counter[RTC_SECONDS] = rtc_sec;
  RTC_Counter[RTC_MINUTES] = rtc_min;
  RTC_Counter[RTC_HOURS] = rtc_hour;
  RTC_Counter[RTC_DAY_OF_WEEK] = rtc_dow;
  RTC_Counter[RTC_DATE_OF_MONTH] = rtc_date;
  RTC_Counter[RTC_MONTH] = rtc_month;
  RTC_Counter[RTC_YEAR] = rtc_year;
  RTC_REGD.Valid_RAM = 1;
  // This stuff is what Lisp puts there
  /*
  RTC_RAM[0] = 0x2C; // TZ Low
  RTC_RAM[1] = 0x01; // TZ Hi
  RTC_RAM[2] = 'C'; // Cookie
  RTC_RAM[3] = '\'';
  RTC_RAM[4] = 'e';
  RTC_RAM[5] = 's';
  RTC_RAM[6] = 't';
  RTC_RAM[7] = ' '; 
  RTC_RAM[8] = 'v';
  RTC_RAM[9] = 'r';
  RTC_RAM[10] = 'a';
  RTC_RAM[11] = 'i';
  RTC_RAM[12] = '.';
  RTC_RAM[13] = 0;
  */
  // Clobber state
  SDU_state=0;
  // Initialize CPU state. The SDU would have done this at startup?
  boot_lambda(0,0);
  boot_lambda(1,0);
  // Initialize 8088
  reset86();
  // Reset PIC state
  {
    int x=0;
    while(x < 3){
      PIC[x].ISR = 0;
      PIC[x].IMR = 0;
      PIC[x].IRR = 0;
      PIC[x].State = 0;
      PIC[x].Last_IRQ = 0;
      x++;
    }
  }
}

void sdu_ram_write(uint32_t addr, uint32_t wd){
  nuData data;
  data.word = wd;
  SDU_RAM[addr] = data.byte[0];
  SDU_RAM[addr+1] = data.byte[1];
  SDU_RAM[addr+2] = data.byte[2];
  SDU_RAM[addr+3] = data.byte[3];
}

uint32_t sdu_ram_read(uint32_t addr){
  nuData data;
  data.byte[0] = SDU_RAM[addr];
  data.byte[1] = SDU_RAM[addr+1];
  data.byte[2] = SDU_RAM[addr+2];
  data.byte[3] = SDU_RAM[addr+3];
  return(data.word);
}

void boot_lambda(int cp,int step){
  if(step == 0){
    // Stop the processor if it's running
    pS[cp].cpu_die_rq = 1;
    // Clobber PMR
    pS[cp].PMR.raw = 0;    
    pS[cp].PMR.Fast_Clock_Enable = 1;
    // Clear WCS
    bzero(pS[cp].WCS,16384*sizeof(uint64_t));
    // Initialize WCS map
    {
      int i;
      for(i=0; i<4096; i++){
	pS[cp].CRAM_map[i] = i;
      }
    }
    // Enable bits in CONREG
    pS[cp].ConReg.word = 0;
    pS[cp].ConReg.Enable_SM_Clock = 1;
    pS[cp].ConReg.Enable_NU_Master = 1;
    pS[cp].ConReg.any_parity_error_synced_l = 1; // No parity error
    pS[cp].ConReg.t_hold_l = 1; // Not holding
    pS[cp].ConReg.LED = 1; // Light LED
    // Clobber uPC
    pS[cp].loc_ctr_reg.raw=0;
    pS[cp].loc_ctr_nxt=-1;
    pS[cp].loc_ctr_cnt=0;
    // Set DP mode bits
    pS[cp].DP_Mode.PDL_Addr_Hi = 1;
    // Clobber PDL pointer and index
    pS[cp].pdl_ptr_reg = 0x7FF;
    pS[cp].pdl_index_reg = 0x7FF;
  }
  // Later steps are vestigial
  if(step == 2){
    /* *********  Patches to Microcode go here ********* */
    printf("Patching loaded microcode...\n");
    pS[cp].WCS[016047].ASource = 01114;    // Fix to XGCD1+2 - Add1 to Rotate Field Sub1 from Length
    // All done, continue
    printf("Starting loaded microcode...\n");
    // Write conf pointer to Qreg
    if(cp == 0){
      pS[cp].Qregister = 0xFF000000+proc0_conf_base; // Points to proc conf
    }
    if(cp == 1){
      pS[cp].Qregister = 0xFF000000+proc1_conf_base; // Points to proc conf
    }
    // Set PC to 2 (cold start w/ conf pointer)
    pS[cp].loc_ctr_reg.raw=02;
    pS[cp].loc_ctr_nxt=-1;
    pS[cp].loc_ctr_cnt=02;
    // Enabling of cache and such would go here
    // Enable tracing
    // pS[bsp].microtrace = true;
    // NUbus_trace = 1;
    // Run!
    pS[cp].cpu_die_rq = 0;
  }
}

// i8259 PIC
void pic_write(int pic,int adr, uint8_t data){
  switch(adr){
  case 0: // COMMAND
    // What did we get?
    if(data&0x10){
      // ICW1
      PIC[pic].State = 1;
      // The edge sense circuit is reset, meaning that all pending and outstanding interrupt requests are ignored.
      PIC[pic].Last_IRQ = 0xFF;
      // Interrupt Mask Register is cleared
      PIC[pic].IMR = 0;
      // IR7 input is assigned priority 7
      // Slave Mode Address is set to 7
      // Special Mask Mode is cleared and Status Read is set to IRR
      // If IC4 = 0, everything in ICW4 is zeroed.
      printf("SDU: PIC %d ICW1 [ ",pic);
      if(data&0x01){ PIC[pic].ICW4 = 1; printf("ICW4 "); }else{ PIC[pic].ICW4 = 0; }
      if(data&0x02){ printf("SINGLE "); }else{ printf("CASCADE "); }
      if(data&0x04){ printf("ADI4 "); }else{ printf("ADI8 "); }
      if(data&0x08){ printf("LEVEL "); }else{ printf("EDGE "); }
      printf("]\n");
      break;
    }else{
      switch(PIC[pic].State){
      case 4: // OCW2/3
	switch(data&0x18){
	case 0x00: // OCW2
	  {
	    /*
	    if(pic == 2){
	      printf("SDU: PIC ");
	      writeDec(pic);
	      printf(" OCW2: ");
	    }
	    */
	    uint8_t ir = data&0x07;
	    uint8_t op = data&0xe0;
	    int x = 0x01,y = 0;
	    switch(op){
	    case 0x20: // NON-SPECIFIC EOI
	      // Dismiss highest priority request
	      /* if(pic == 2){
		printf("NON-SPEC EOI ");
		} */
	      while(x < 0x100){
		if((PIC[pic].ISR&x) != 0){
		  PIC[pic].ISR &= ~x;
		  PIC[pic].IRR &= ~x;
		  PIC[pic].IRQ &= ~x;	
		  /* if(pic == 2){
		    writeDec(y);
		    } */
		  break;
		}
		x <<= 1;
		y++;
	      }
	      break;
	    case 0x60: // SPECIFIC EOI
	    case 0xA0: // ROTATE ON NON-SPECIFIC EOI
	    case 0x80: // ROTATE IN AUTO EOI MODE (SET)
	    case 0x00: // ROTATE IN AUTO EOI MODE (CLEAR)
	    case 0xE0: // ROTATE ON SPECIFIC EOI
	    case 0xC0: // SET PRIORITY CMD
	    case 0x40: // NO-OP
	    default:
	      printf("UNKNOWN PIC OCW2 OP 0x%X IR %X\n",op,ir);
	      ld_die_rq = 1;
	    }
	    /* if(pic == 2){
	      printf("\n");
	      } */
	  }
	  break;
	case 0x08: // OCW3
	  {
	    uint8_t rdr = data&0x03;
	    if(rdr == 1 || rdr == 3){
	      PIC[pic].ReadReg = rdr;
	    }
	    /* if(pic == 2){
	      uint8_t smm = data&0x60;
	      printf("SDU: PIC ");
	      writeDec(pic);
	      printf(" OCW3: RDR 0x");
	      writeH8(rdr);
	      if(data&0x40){ printf(" POLL"); }
	      printf(" SMM 0x");
	      writeH8(smm);
	      printf("\n");
	      } */
	  }
	  break;
	default:
	  printf("SDU: PIC %d CMD WRITE 0x%X WSEL 0x%X\n",pic,data,(data&0x18));
	  ld_die_rq = 1;
	}
	break;
      default:
	printf("SDU: PIC %d CMD WRITE 0x%X STATE %d\n",pic,data,PIC[pic].State);
	ld_die_rq = 1;
      }
    }
    break;
  case 1: // DATA
    switch(PIC[pic].State){
    case 1: // ICW2
      printf("SDU: PIC %d ICW2: Vec 0x%X\n",pic,(data&0xF8));
      PIC[pic].Base = data&0xF8;
      PIC[pic].State++;
      break;
    case 2: // ICW3
      // Initialize not-slave mask register
      PIC[pic].NSMR = 0xFF;
      // Process ICW3
      if(pic > 0){
	// Slave
        printf("SDU: PIC %d SLAVE ICW3: Slave ID 0x%X\n",pic,data);
        PIC[pic].State++;
      }else{
	// Master
	printf("SDU: PIC %d MASTER ICW3: Slave Mask 0x%X\n",pic,data);
	// If we are in SFNM, we need to mask this out of the NSMR!
	if(PIC[pic].SFNM == 1){
	  PIC[pic].NSMR ^= data;
	}
	PIC[pic].State++;
      }
      if(PIC[pic].ICW4 == 0){
	// Skip ICW4
	PIC[pic].ReadReg = 1;
	PIC[pic].State++;
      }
      break;
    case 3: // ICW4
      printf("SDU: PIC %d ICW4 [",pic);
      if(data&0x01){ printf(" 8088"); }else{ printf(" MCS"); }
      if(data&0x02){ printf(" AUTO"); }else{ printf(" NORM"); }
      if(data&0x08){ printf(" BUF-"); 
	if(data&0x04){ printf("MASTER"); }else{ printf("SLAVE"); }
      }else{
	printf(" NONBUF");
      }
      if(data&0x010){ 
	// SPECIAL FULLY NESTED MODE
	printf(" SFNM"); 
	PIC[pic].SFNM = 1;
      }else{
	PIC[pic].SFNM = 0;
      }
      printf(" ]\n");
      PIC[pic].ReadReg = 1;
      PIC[pic].State++;
      break;
    case 4: // OCW1
      printf("SDU: PIC %d OCW1: IMR = 0x%X\n",pic,data);
      PIC[pic].IMR = data;
      break;

    default:
      printf("SDU: PIC %d DATA WRITE 0x%X STATE %d\n",pic,data,PIC[pic].State);
      ld_die_rq = 1;
    }
    break;
  }
}

uint8_t pic_read(int pic,int adr){
  switch(adr){
  case 0: // COMMAND 
    switch(PIC[pic].ReadReg){
    case 1: // IRR
      /* if(pic == 2){
	printf("SDU: PIC ");
	writeDec(pic);
	printf(" IRR READ = 0x");
	writeH8(PIC[pic].IRR);
	printf("\n");
	} */
      return(PIC[pic].IRR);
      break;
    case 3: // ISR
      return(PIC[pic].ISR);
      break;
    default:
      printf("SDU: PIC %d REG %d READ\n",pic,PIC[pic].ReadReg);
      ld_die_rq = 1;
      break;
    }
    break;
  case 1: // DATA    
    // IMR
    /* if(pic == 2){
      printf("SDU: PIC ");
      writeDec(pic);
      printf(" IMR READ = 0x");
      writeH8(PIC[pic].IMR);
      printf("\n");
      } */
    return(PIC[pic].IMR);
    break;
  }
  return(0);
}

uint16_t evaluate_pic(int pic){
  // Evaluate the state of a PIC
  uint16_t status = 0;
  uint8_t intr = 0;
  if(PIC[pic].State == 4){
    int x = 0x01,y = 0;
    // Perform edge detection
    while(x < 0x100){
      // Edge detected?
      if((PIC[pic].IRQ&x) == x){
	if((PIC[pic].Last_IRQ&x) == 0){
	  intr = 1;
	  /* if(pic == 2){
	    printf("PIC ");
	    writeDec(pic);
	    printf(": EDGE DETECTED - IRQ ");
	    writeDec(y);
	    if((PIC[pic].IMR&x) != 0){
	      printf(" (MASKED!)");
	    }
	    printf("\n");
	    } */
	  PIC[pic].Last_IRQ |= x;	  
	  break;
	}
      }else{
	// Turn off bit
	PIC[pic].Last_IRQ &= ~x; // PIC[pic].IRQ;
      }
      x <<= 1;
      y++;
    }
    // Update IRR
    PIC[pic].IRR = PIC[pic].IRQ;
    // PIC[pic].Last_IRQ = PIC[pic].IRQ;
    // If we found a new int, handle it
    if(intr != 0){
      x = 0x01,y = 0;
      // CHECK FROM HIGH TO LOW, 0 = HIGHEST
      while(x < 0x100){
	// If we're busy at this level, we have nothing more to do.
	if((PIC[pic].ISR&x) != 0){ 
	  /* if(pic == 2){
	    printf("PIC ");
	    writeDec(pic);
	    printf(" BUSY WITH IRQ ");
	    writeDec(y);
	    if((PIC[pic].IMR&x) != 0){
	      printf(" (MASKED!)");
	    }
	    printf("\n");
	    } */
	  return(status); 
	}
	// Otherwise...
	if((PIC[pic].IRR&x) != 0                     // If we have a request
	   && (PIC[pic].IMR&x) == 0                  // Not masked off
	   && (PIC[pic].ISR&x) == 0){                // And not already in service
	  status = 0x8000|y;                         // Then here is our output
	  /* if(pic == 2){
	    printf("PIC ");
	    writeDec(pic);
	    printf(" INITIATING IRQ ");
	    writeDec(y);
	    printf("\n");
	    } */
	  return(status);                            // Go tell someone
	}
	x <<= 1;                                     // Otherwise loop
	y++;
      }
      // If we got here, we found an interrupt but it was masked.
      // So we return nothing.
      return(0);
    }
  }
  return(status);
}

uint16_t pic_chk(){
  uint16_t status = 0;
  uint16_t pic_status[3] = { 0,0,0 };
  // Check for interrupt needing service
  // First, evaluate slave states to generate PIC0 request lines
  pic_status[1] = evaluate_pic(1);
  pic_status[2] = evaluate_pic(2);
  if(pic_status[1]&0x8000 || PIC[1].ISR != 0){ PIC[0].IRQ |= 0x80; }else{ PIC[0].IRQ &= 0x7F; }
  if(pic_status[2]&0x8000 || PIC[2].ISR != 0){ PIC[0].IRQ |= 0x40; }else{ PIC[0].IRQ &= 0xBF; }
  // Now evaluate master state
  pic_status[0] = evaluate_pic(0);
  if((pic_status[0]&0x8000) == 0 && PIC[0].SFNM == 1){
    // If SFNM, we can take additional slave interrupts even if the slave line is still busy
    // Allow PIC 2 on IRQ 6 to clobber 7 if they are both active
    if(pic_status[1]&0x8000){ pic_status[0] = 0x8007; }
    if(pic_status[2]&0x8000){ pic_status[0] = 0x8006; }
  }
  /*
  if(pic_status[0] != 0 || pic_status[1] != 0 || pic_status[2] != 0){
    printf("PIC IRRs: 0x");
    writeH8(PIC[0].IRR); printf(" ");
    writeH8(PIC[1].IRR); printf(" ");
    writeH8(PIC[2].IRR); printf("\n");
    printf("PIC IMRs: 0x");
    writeH8(PIC[0].IMR); printf(" ");
    writeH8(PIC[1].IMR); printf(" ");
    writeH8(PIC[2].IMR); printf("\n");
    printf("PIC ISRs: 0x");
    writeH8(PIC[0].ISR); printf(" ");
    writeH8(PIC[1].ISR); printf(" ");
    writeH8(PIC[2].ISR); printf("\n");
  }
  */
  // If PIC0 returned an interrupt...
  if(pic_status[0]&0x8000){
    // Who did it?
    int irq = pic_status[0]&0x0F;
    int subirq = -1;
    int base = PIC[0].Base;
    // Light the ISR bit in PIC0
    PIC[0].ISR |= (0x01<<irq);
    // Was it a slave?
    switch(irq){
    case 6: // PIC-2
      if(pic_status[2]&0x8000){
	// Take info
	subirq = pic_status[2]&0x0F;
	base = PIC[2].Base;
	// Light the ISR bit in the slave PIC
	PIC[2].ISR |= (0x01<<subirq);
      }
      break;
    case 7: // PIC-1
      if(pic_status[1]&0x8000){
	// Take info
	subirq = pic_status[1]&0x0F;
	base = PIC[1].Base;
	// Light the ISR bit in the slave PIC
	PIC[1].ISR |= (0x01<<subirq);
      }
      break;
    }
    // ISR bits and IRQ info should be all set!
    /*
    printf("PIC 0: Servicing irq ");
    writeDec(irq);
    if((irq == 6 || irq == 7) && subirq > -1){
      printf(" subirq ");
      writeDec(subirq);
    }
    printf(" via vector 0x");
    writeH8(base+irq);
    printf("\n");
    */
    if((irq == 6 || irq == 7) && subirq > -1){
      irq = subirq;
    }    
    status = 0x8000|(base+irq);
  }
  // We are done!
  return(status);
}

// 8253 PIT
uint8_t pit_read(int pit,int adr){
  switch(adr){
  default:
    printf("SDU: PIT %d REG %d READ\n",pit,adr);
    ld_die_rq = 1;
    return(0xFF);
    break;
  }
  return(0x00);
}

// PIT_RATE is nominally 1.2288 MHz
#define PIT_RATE (1.2288/5)

void pit_clockpulse(){
  int x = 0;
  while(x < 2){
    int y = 0;
    while(y < 3){
      if(PIT[x].State[y] == 1){
	// Track ticks in state
	if(PIT[x].Output_Ticks[y] < 5){ PIT[x].Output_Ticks[y]++; }
	// Clock running
	switch(PIT[x].Mode[y]){
	case 2: // Rate Generator
	  PIT[x].RCounter[y] -= PIT_RATE;
	  if(PIT[x].RCounter[y] <= 0){ PIT[x].Counter[y] = 0; }else{ PIT[x].Counter[y] = PIT[x].RCounter[y]; }
	  if(PIT[x].Counter[y] == 0){
	    PIT[x].Output[y] = 1;
	    // Interrupt
	    if(x == 1){
	      uint8_t IRQ = 0x10;
	      IRQ <<= y;
	      if((PIC[1].IRQ&IRQ) == 0){
		PIC[1].IRQ |= IRQ;
		PIT[x].Output_Ticks[y] = 0;
	      }else{
		printf("PIT: CTR %d FIRED WHILE IRQ ALREADY SET\n",y);
	      }
	      /*
	      if(x == 1 && y == 2){
		extern uint8_t dotrace;
		printf("PIT: DEBUG TRIGGER FIRED\n");
		dotrace = 1;
	      }
	      */
	    }
	    // Reload
	    PIT[x].RCounter[y] = PIT[x].Counter[y] = PIT[x].IV[y];
	  }else{
	    PIT[x].Output[y] = 0;
	    // Don't Interrupt
	    if(x == 1){
	      uint8_t IRQ = 0x10;
	      IRQ <<= y;
	      if((PIC[1].IRQ&IRQ) == IRQ && PIT[x].Output_Ticks[y] > 4){
		PIC[1].IRQ ^= IRQ;
		/*
		if(x == 1 && y == 2){
		  // extern uint8_t dotrace;
		  printf("PIT: DEBUG TRIGGER UN-FIRED\n");
		  // dotrace = 1;
		}
		*/
	      }
	    }
	  }
	  break;

	case 3: // Square Wave
	  // Accumulate real time
	  PIT[x].RCounter[y] += PIT_RATE;
	  // If we have enough for a pulse
	  while(PIT[x].RCounter[y] > 1){ 
	    // Decrement counter at twice the normal rate.
	    // See the datasheet for how/why.
	    if(PIT[x].Counter[y]&0x01){
	      if(PIT[x].Output[y] == 1){
		PIT[x].Output[y]--;
	      }else{
		PIT[x].Output[y] -= 3;
	      }
	    }else{
	      PIT[x].Counter[y] -= 2;
	    }
	    // When count runs out
	    if(PIT[x].Counter[y] == 0){
	      // Toggle output
	      PIT[x].Output[y] ^= 1;
	      // Reload
	      PIT[x].Counter[y] = PIT[x].IV[y];
	    }
	    // Use up real time
	    PIT[x].RCounter[y] -= 1;
	  }
	  // Set/Unset IRQ
	  if(PIT[x].Output[y] == 1){
            // Interrupt
            if(x == 1){
              uint8_t IRQ = 0x10;
              IRQ <<= y;
              if((PIC[1].IRQ&IRQ) == 0){
                PIC[1].IRQ |= IRQ;
              }
            }
          }else{
            // Don't Interrupt
            if(x == 1){
              uint8_t IRQ = 0x10;
              IRQ <<= y;
              if((PIC[1].IRQ&IRQ) == IRQ){
                PIC[1].IRQ ^= IRQ;
              }
            }
          }
          break;
	  
	default:
	  printf("pit_clockpulse(): Unknown mode %d\n",PIT[x].Mode[y]);
	  ld_die_rq = 1;
	}
      }
      y++;
    }
    x++;
  }
}

void pit_write(int pit, int adr, uint8_t data){
  switch(adr){
  case 0: // COUNTER 0
  case 1: // COUNTER 1
  case 2: // COUNTER 2    
    printf("SDU: PIT #%d Counter %d Reg = 0x%X\n",pit,adr,data);
    if(PIT[pit].LoadHW[adr] == 0){
      // Load low bits then optionally hi bits      
      PIT[pit].Counter[adr] &= 0xFF00;
      PIT[pit].Counter[adr] |= data;
      if(PIT[pit].Format[adr] == 3){
	PIT[pit].LoadHW[adr]++;
      }else{
	PIT[pit].RCounter[adr] = PIT[pit].Counter[adr];
	PIT[pit].IV[adr] = PIT[pit].Counter[adr];
	PIT[pit].State[adr] = 1; // Run
      }
      break;
    }
    if(PIT[pit].LoadHW[adr] == 1){
      // Load hi bits
      PIT[pit].Counter[adr] &= 0x00FF;
      PIT[pit].Counter[adr] |= (data<<8);
      if(PIT[pit].Format[adr] == 3){
	PIT[pit].LoadHW[adr] = 0;
      }
      PIT[pit].RCounter[adr] = PIT[pit].Counter[adr];
      PIT[pit].IV[adr] = PIT[pit].Counter[adr];
      PIT[pit].State[adr] = 1; // Run
      break;
    }
    break;

  case 3: // Control Word
    {
      uint8_t ctr = ((data&0xC0)>>6);
      uint8_t fmt = ((data&0x30)>>4);
      uint8_t mod = ((data&0x0E)>>1);
      printf("SDU: PIT #%d MODE: CTR %d FMT %d MODE %d\n",pit,ctr,fmt,mod);
      // Make it so
      PIT[pit].Format[ctr] = fmt;
      PIT[pit].Mode[ctr] = mod;
      PIT[pit].Control[ctr] = (data&0x3F);
      PIT[pit].State[ctr] = 0; // Load
      switch(fmt){
      case 0: // LATCH
	printf("LATCH NI\n"); ld_die_rq=1; break;
      case 1: // LOW
      case 3: // LOW/HI
	PIT[pit].LoadHW[ctr] = 0; break;
      case 2: // HI
	PIT[pit].LoadHW[ctr] = 1; break;
      }
    }
    break;

  default:
    printf("SDU: PIT %d REG %d WRITE\n",pit,adr);
    ld_die_rq = 1;
    break;
  }
}

// THIS IS THE SDU ADDRESS SPACE LAYOUT (AS SEEN FROM THE 8088)
// 0x000000 - 0x00FFFF = SDU RAM
// 0x018000 - 0x018FFF = Multibus -> NUbus map control registers
// 0x01C000 - 0x01C3FF = SDU register page (serial ports, etc)
// 0x01E000 - 0x01FFFF = CMOS RAM
// 0x02F800 - 0x02FBFF = Second burr-brown
// 0x02FC00 - 0x02FFFF = First burr-brown
// 0x030000 - 0x031FFF = 3Com ethernet
// 0x040000 - 0x0EFFFF = DYNAMICALLY ALLOCATED MAPPED AREA (MAPPED TO NUBUS!)
// 0x0F0000 - 0x0FFFFF = SDU ROM

void sducons_rx_int(){
  printf("SDUCONS: RXINT\n");
  // if(PIC[1].IMR&0x01){ PIC[1].IMR ^= 0x11; } // HACK HACK
  PIC[1].IRQ |= 0x01; // SERIO IN
}

// Peripherals use this. The 8088's 8-bit BIU cannot generate 16-bit bus cycles.
uint16_t multibus_word_read(mbAddr addr){
  // HANDLE NUBUS MAP
  if(MNA_MAP[addr.Page].Enable != 0){
    nuAddr MNB_Addr;
    MNB_Addr.Page = MNA_MAP[addr.Page].NUbus_Page;
    MNB_Addr.Offset = addr.Offset;
    if(MNB_Addr.Card != 0xFA && MNB_Addr.Card != 0xF9 &&
       MNB_Addr.Card != 0xF8 && MNB_Addr.Card != 0xFC){
      printf("MULTIBUS Word Addr 0x%X => NUBUS Addr 0x%X\n",addr.raw,MNB_Addr.raw);
    }
    // Obtain bus if we don't already have it
    while(NUbus_Busy != 0 && NUbus_master != 0xFF){
      nubus_cycle(1);
    }
    // Place request on bus, lighting the low bit to indicate halfword-ness
    nubus_io_request(VM_READ,0xFF,MNB_Addr.raw|1,0);
    // Await completion or error
    while(NUbus_Busy != 0 && NUbus_error == 0 && NUbus_acknowledge == 0){
      nubus_cycle(1);
    }
    // What did we get?
    if(NUbus_error != 0){
      // Light bus error reg
      nubus_timeout_reg = 0xFF;
      // Fire interrupt
      PIC[0].IRQ |= 0x02;
    }else{
      nubus_timeout_reg = 0;
    }
    if(NUbus_acknowledge != 0){
      // If this address wasn't on a halfword boundary this fails.
      if(NUbus_Address.Byte == 1){
	return(NUbus_Data.hword[0]);
      }else{
	return(NUbus_Data.hword[1]);
      }
    }
    return(0xFFFF);
  }

  switch(addr.raw){
  case 0x000000 ... 0x00FFFF: // SDU RAM
    return(*(uint16_t *)(SDU_RAM+addr.raw));
    break;

  default:
    printf("multibus_word_read: Unknown addr 0x%X\n",addr.raw);
    ld_die_rq = 1;
    return(0xFFFF);
    break;
  }
}

uint8_t multibus_read(mbAddr addr){
  // HANDLE NUBUS MAP
  if(MNA_MAP[addr.Page].Enable != 0){
    nuAddr MNB_Addr;
    MNB_Addr.Page = MNA_MAP[addr.Page].NUbus_Page;
    MNB_Addr.Offset = addr.Offset;
    if(MNB_Addr.Card != 0xFA && MNB_Addr.Card != 0xF9 &&
       MNB_Addr.Card != 0xF8 && MNB_Addr.Card != 0xFC){
      printf("MULTIBUS Addr 0x%X => NUBUS Addr 0x%X\n",addr.raw,MNB_Addr.raw);
    }
    // Obtain bus if we don't already have it
    while(NUbus_Busy != 0 && NUbus_master != 0xFF){
      nubus_cycle(1);
    }
    // Place request on bus
    nubus_io_request(VM_BYTE_READ,0xFF,MNB_Addr.raw,0);
    // Await completion or error
    while(NUbus_Busy != 0 && NUbus_error == 0 && NUbus_acknowledge == 0){
      nubus_cycle(1);
    }
    // What did we get?
    if(NUbus_error != 0){
      // Light bus error reg
      nubus_timeout_reg = 0xFF;
      // Fire interrupt
      PIC[0].IRQ |= 0x02;
    }else{
      nubus_timeout_reg = 0;
    }
    if(NUbus_acknowledge != 0){
      return(NUbus_Data.byte[NUbus_Address.Byte]);
    }
    return(0xFF);
  }

  switch(addr.raw){
  case 0x000000 ... 0x00FFFF: // SDU RAM
    return(SDU_RAM[addr.raw]);
    break;

  case 0x1c084: // Switch Setting / Slot Number
    {
      // high four bits are slot number of SDU.
      // low four bits give switch setting.
      // The switch is rotary
      // Position 0 = F
      // Position 1 = E
      // Position 2 = D
      // Position 3 = B
      // Position 4 = 7
      uint8_t data = 0xF0;
      extern uint8_t sdu_rotary_switch;
      switch(sdu_rotary_switch){
      case 0:
      default:
	data |= 0x0F; break;
      case 1:
	data |= 0x0E; break;
      case 2:
	data |= 0x0D; break;
      case 3:
	data |= 0x0B; break;
      case 4:
	data |= 0x07; break;
      }
      return(data);
    }
    break;

  case 0x1c088: // SDU CSR0
    {
      uint8_t data = 0x00;
      printf("SDUCSR0 0x%X\n",data);
      return(data);
    }
    break;

  case 0x1c08c: // SDU CSR1
    {
      uint8_t data = 0;
      if(sdu_multibus_enable != 0){ data |= 0x01; }
      if(sdu_nubus_enable != 0){ data |= 0x02; }
      printf("SDUCSR1 0x%X\n",data);
      return(data);
    }
    break;

  case 0x1c120: // RTC Data Register
    {
      uint8_t RTC_Data = 0;
      switch(rtc_addr){
      case 0x00 ... 0x09:
	if(RTC_REGA.Update_In_Progress == 0){
	  RTC_Data = RTC_Counter[rtc_addr];
	}else{
	  RTC_Data = 0;
	}
	break;
      case 0x0A:
	RTC_Data = RTC_REGA.byte;
	break;
      case 0x0B:
	RTC_Data = RTC_REGB.byte;
	break;
      case 0x0C:
	RTC_Data = RTC_REGC.byte;
	break;
      case 0x0D:
	RTC_Data = RTC_REGD.byte;
	RTC_REGD.Valid_RAM = 1; // Reading Reg D sets this bit
	break;
      case 0x0E ... 0x3F:
	RTC_Data = RTC_RAM[rtc_addr-0x0E];
	break;
      default:
	RTC_Data = 0;
      }
      return(RTC_Data);
    }
    break;

  case 0x1c124: // RTC Address Register
    return(rtc_addr);
    break;

  case 0x1c150: // console serial data register
    {
      uint8_t data = 0;
      if(sdu_rx_ptr > 0){
	data = sdu_rx_buf[sdu_rx_bot];
	sdu_rx_bot++;
	if(sdu_rx_bot == sdu_rx_ptr){
	  sdu_rx_ptr = sdu_rx_bot = 0;
	  PIC[1].IRQ &= ~0x01; // SERIO IN
	}
      }
      printf("SDUCONS: IN ");
      if(data >= 0x20 && data <= 0x7E){
	printf("%c",data);
      }else{
	printf("0x%X",data);
      }
      printf("\n");
      return(data);
    }
    break;

  case 0x1c154: // console serial command register
    {
      uint8_t status = 0x81; // -DSR, TXRDY
      if(sdu_conn_fd < 0){
	status ^= 0x80; // Clear -DSR
      }
      if(sducons_tx_top == 0){
	status |= 0x04; // TX EMPTY
      }
      if(sdu_rx_ptr > 0){
	status |= 0x02; // RX RDY
      }
      // printf("RXSTA 0x"); writeH8(status); printf("\n");
      return(status);
    }
    break;
    
  case 0x1c15c: // "other" serial command register
    {
      uint8_t status = 0x81; // -DSR, TXRDY
      status ^= 0x80; // Clear -DSR      
      status |= 0x04; // TX EMPTY
      // printf("OTHER-RXSTA 0x"); writeH8(status); printf("\n");
      return(status);
    }
    break;
    
  case 0x1c180: // nubus timeout registeer
    printf("i8088: NUBUS TIMEOUT REG READ\n");
    return(nubus_timeout_reg);
    break;

  case 0x1c1c0: // PIC #0 Cmd
    return(pic_read(0,0));
    break;
  case 0x1c1c4: // PIC #0 Data port?
    return(pic_read(0,1));
    break;
  case 0x1c1c8: // PIC #1 Cmd
    return(pic_read(1,0));
    break;
  case 0x1c1cc: // PIC #1 Data port?    
    return(pic_read(1,1));
    break;
  case 0x1c1d0: // PIC #2 Cmd
    return(pic_read(2,0));
    break;
  case 0x1c1d4: // PIC #2 Data port?
    return(pic_read(2,1));
    break;

  case 0x1c1e0: // Multibus Interrupt 0
    {
      // Reads interrupt status!
      uint8_t data = PIC[2].IRQ;
      printf("i8088: READ MB INT STATUS\n");
      return(data);
    }
    break;

  case 0x18000 ... 0x18FFF: // multibus -> nubus map
    {
      int MAP_Addr = ((addr.raw-0x18000)>>2);
      int byte = addr.raw&0x03;
      return(MNA_MAP[MAP_Addr].byte[byte]);
    }
    break;

  case 0x01E000 ... 0x01FFFF: // CMOS RAM
    {
      uint32_t CMOS_Addr = ((addr.raw-0x01E000)>>2);
      /*
      printf("i8088: CMOS READ: 0x");
      writeH32(CMOSAddr);
      printf(" = 0x");
      writeH8(CMOS_RAM[CMOSAddr]);
      printf("\n");      
      */
      return(CMOS_RAM[CMOS_Addr]);      
    }
    break;

  case 0x030000 ... 0x031FFF: // 3com Ethernet
    {
      uint16_t enet_addr = (addr.raw&0xFFFF);
      uint8_t data = enet_read(enet_addr);
      printf("SDU: 3COM READ: 0x%X = 0x%X\n",enet_addr,data);
      return(data);
    }
    break;

  case 0x040000 ... 0x0EFFFF: // DYNAMICALLY ALLOCATED MAPPED AREA
    // This should have been caught above. The map must not have been set up.
    // Throw a multibus timeout.
  case 0x2fe00: // Don't know what this is, newboot tries to read it
  case 0x2ff00: // Don't know what this is, newboot tries to read it
    printf("multibus_read: timeout for addr 0x%X\n",addr.raw);
    PIC[0].IRQ |= 0x01;
    return(0xFF);
    break;

  case 0x0F0000 ... 0x0FFFFF: // SDU ROM
    {
      uint32_t ROMAddr = addr.raw-0x0F0000;
      return(SDU_ROM[ROMAddr]);
    }
    break;    

  default:
    printf("multibus_read: Unknown addr 0x%X\n",addr.raw);
    ld_die_rq = 1;
    return(0xFF);
  }
}

// Peripherals use this. The 8088's 8-bit BIU cannot generate 16-bit bus cycles.
void multibus_word_write(mbAddr addr,uint16_t data){
  // HANDLE NUBUS MAP
  if(MNA_MAP[addr.Page].Enable != 0){
    nuAddr MNB_Addr;
    uint32_t Word = data;
    MNB_Addr.Page = MNA_MAP[addr.Page].NUbus_Page;
    MNB_Addr.Offset = addr.Offset;
    if(MNB_Addr.Card != 0xFA && MNB_Addr.Card != 0xF9 &&
       MNB_Addr.Card != 0xF8 && MNB_Addr.Card != 0xFC){
      printf("MULTIBUS Word Addr 0x%X => NUBUS Addr 0x%X\n",addr.raw,MNB_Addr.raw);
    }
    // Obtain bus if we don't already have it
    while(NUbus_Busy != 0 && NUbus_master != 0xFF){
      nubus_cycle(1);
    }
    // Place request on bus
    Word <<= (8*MNB_Addr.Byte); // This won't work if the address isn't on a halfword boundary
    nubus_io_request(VM_WRITE,0xFF,MNB_Addr.raw|1,Word);
    // Await completion or error
    while(NUbus_Busy != 0 && NUbus_error == 0 && NUbus_acknowledge == 0){
      nubus_cycle(1);
    }
    // What did we get?
    if(NUbus_error != 0){
      // Light bus error reg
      nubus_timeout_reg = 0xFF;
      // Fire interrupt
      PIC[0].IRQ |= 0x02;
    }else{
      nubus_timeout_reg = 0;
    }
    return;
  }

  switch(addr.raw){
  case 0x000000 ... 0x00FFFF: // SDU RAM
    *(uint16_t *)(SDU_RAM+addr.raw) = data;
    break;

  default:
    printf("multibus_word_write: Unknown addr 0x%X\n",addr.raw);
    ld_die_rq = 1;
  }
}

void multibus_write(mbAddr addr,uint8_t data){
  // HANDLE NUBUS MAP
  if(MNA_MAP[addr.Page].Enable != 0){
    nuAddr MNB_Addr;
    uint32_t Word = data;
    extern volatile int NUbus_trace;
    MNB_Addr.Page = MNA_MAP[addr.Page].NUbus_Page;
    MNB_Addr.Offset = addr.Offset;
    if(MNB_Addr.Card != 0xFA && MNB_Addr.Card != 0xF9 &&
       MNB_Addr.Card != 0xF8 && MNB_Addr.Card != 0xFC){
      printf("MULTIBUS Addr 0x%X => NUBUS Addr 0x%X\n",addr.raw,MNB_Addr.raw);
    }
    // Obtain bus if we don't already have it
    while(NUbus_Busy != 0 && NUbus_master != 0xFF){
      if(NUbus_trace == 1){
	printf("SDU: Awaiting NUBUS: %d\n",NUbus_Busy);
      }
      nubus_cycle(1);
    }
    // Place request on bus
    Word <<= (8*MNB_Addr.Byte);
    nubus_io_request(VM_BYTE_WRITE,0xFF,MNB_Addr.raw,Word);
    // Await completion or error
    while(NUbus_Busy != 0 && NUbus_error == 0 && NUbus_acknowledge == 0){
      if(NUbus_trace == 1){
	printf("SDU: Driving NUBUS cycle: %d\n",NUbus_Busy);
      }
      nubus_cycle(1);
    }
    if(NUbus_trace == 1){
      printf("NUBUS: SDU Cycle Complete: Request %o Addr 0x%X (0%o) w/ data 0x%X (0%o) Ack %o\n",
	     NUbus_Request,NUbus_Address.raw,NUbus_Address.raw,NUbus_Data.word,NUbus_Data.word,NUbus_acknowledge);
    }
    // What did we get?
    if(NUbus_error != 0){
      // Light bus error reg
      nubus_timeout_reg = 0xFF;
      // Fire interrupt
      PIC[0].IRQ |= 0x02;
    }else{
      nubus_timeout_reg = 0;
    }
    return;
  }

  switch(addr.raw){
  case 0x000000 ... 0x00FFFF: // SDU RAM
    SDU_RAM[addr.raw] = data;
    break;

  case 0x1c080: // Front Panel Lights
    // LEDs are RUN, SET UP, and ATTN.
    // 0 = on, 1 = off
    // Bit 0 = RUN, 1 = SET UP, 2 = ATTN
    printf("SDU: LIGHTS = 0x%X\n",data);
    break;

  case 0x1c088: // SDU CSR0
    printf("SDU: WRITE CSR0 = 0x%X",data);
    if(data&0x08){
      // QIC tape reset
      printf(" QIC-RESET");
    }
    if(data&0x04){
      // MULTIBUS RESET
      printf(" MB-RESET");
    }
    if(data&0x02){
      // NUBUS RESET
      printf(" NB-RESET");
    }
    if(data&0x01){
      // "X" RESET
      printf(" X-RESET");
    }
    if(data&0xF0){ ld_die_rq = 1; } // QIC status/control bits
    printf("\n");
    break;

  case 0x1c08c: // SDU CSR1
    if(data&0x01){ sdu_multibus_enable = 1; }else{ sdu_multibus_enable = 0; } // MULTIBUS ENABLE
    if(data&0x02){ } // MULTIBUS->NUBUS CONVERTER ENABLE
    if(data&0x04){ sdu_nubus_enable = 1; }else{ sdu_nubus_enable = 0; } // NUBUS ENABLE
    if(data&0x08){ } // CLOCK SLOW 
    if(data&0x10){ } // CLOCK FAST
    // CLOCK SLOW & CLOCK FAST CLEAR = CLOCK NORMAL
    if(data&0x20){ } // VOLT HI
    if(data&0x40){ } // VOLT LOW
    // VOLT HI + VOLT LO = VOLT NORM
    if(data&0x80){ ld_die_rq = 1; }
    printf("SDU: WRITE CSR1 = 0x%X\n",data);
    break;

  case 0x1c120: // RTC Data Register
    {
      // Process bits
      switch(rtc_addr){
      case 0x00 ... 0x09:
	if(RTC_REGA.Update_In_Progress == 0){
	  RTC_Counter[rtc_addr] = data;
	}
	break;
      case 0x0A:
	// UIP bit is read only
	RTC_REGA.byte &= 0x80;
	RTC_REGA.byte |= (data&0x7F);
	break;
      case 0x0B:
	RTC_REGB.byte = data;
	break;
      case 0x0C:
	// Read Only
	break;
      case 0x0D:
	// Read Only
	break;
      case 0x0E ... 0x3F:
	RTC_RAM[rtc_addr-0x0E] = data;
	break;
      }
      printf("RTC: Data Write, data 0x%X\n",data);
    }
    break;

  case 0x1c124: // RTC Address Register
    rtc_addr = data;
    break;

  case 0x1c150: // console serial data register
    {
      sducons_tx_buf[sducons_tx_top] = data;
      sducons_tx_top++;
      printf("SDUCONS: OUT ");
      if(data >= 0x20 && data <= 0x7E){
	printf("%c",data);
      }else{
	printf("0x%X",data);
      }
      printf("\n");
    }
    break;

  case 0x1c154: // console serial command register
    printf("SDU: Console Serial Command Reg = 0x%X\n",data);
    break;

  case 0x1c160: // PIT #1 Counter 0 register (Console Baud Rate Generator)
    pit_write(1,0,data);
    break;    
  case 0x1c164: // PIT #1 Counter 1 register (Aux Baud Rate Generator)
    pit_write(1,1,data);
    break;    
  case 0x1c168: // PIT #1 Counter 2 register 
    pit_write(1,2,data);
    break;    
  case 0x1c16c: // PIT #1 mode register
    pit_write(1,3,data);
    break;    

  case 0x1c170: // PIT #0 counter #0
    pit_write(0,0,data);
    break;    
  case 0x1c17c: // PIT #0 mode register
    pit_write(0,3,data);
    break;    

  case 0x1c180: // nubus timeout registeer
    printf("SDU: NUBus Timeout Register = 0x%X\n",data);
    nubus_timeout_reg = data;
    break;    

  case 0x1c1c0: // PIC #0 (AKA PIC "M") OCW/ICW0
    pic_write(0,0,data);
    break;
  case 0x1c1c4: // PIC #0 OCW/ICW1
    pic_write(0,1,data);
    break;
  case 0x1c1c8: // PIC #1 OCW/ICW0
    pic_write(1,0,data);
    break;
  case 0x1c1cc: // PIC #1 OCW/ICW1
    pic_write(1,1,data);
    break;
  case 0x1c1d0: // PIC #2 OCW/ICW0
    pic_write(2,0,data);
    break;
  case 0x1c1d4: // PIC #2 OCW/ICW1
    pic_write(2,1,data);
    break;

  case 0x1c1e0: // initiate multibus interrupt #0
    if(data == 0){      
      clear_multibus_interrupt(0); // Drop IRQ
    }else{      
      multibus_interrupt(0); // Set IRQ
    }
    break;

  case 0x1c1e4: // initiate multibus interrupt #1
    if(data == 0){      
      clear_multibus_interrupt(1);
    }else{      
      multibus_interrupt(1);
    }
    break;

  case 0x1c1e8: // initiate multibus interrupt #2
    if(data == 0){      
      clear_multibus_interrupt(2);
    }else{      
      multibus_interrupt(2);
    }
    break;

  case 0x1c1ec: // initiate multibus interrupt #3
    if(data == 0){      
      clear_multibus_interrupt(3);
    }else{      
      multibus_interrupt(3);
    }
    break;

  case 0x1c1f0: // initiate multibus interrupt #4
    if(data == 0){      
      clear_multibus_interrupt(4);
    }else{      
      multibus_interrupt(4);
    }
    break;

  case 0x1c1f4: // initiate multibus interrupt #5
    if(data == 0){      
      clear_multibus_interrupt(5);
    }else{      
      multibus_interrupt(5);
    }
    break;

  case 0x1c1f8: // initiate multibus interrupt #6
    if(data == 0){      
      clear_multibus_interrupt(6);
    }else{      
      multibus_interrupt(6);
    }
    break;

  case 0x1c1fc: // initiate multibus interrupt #7
    if(data == 0){      
      clear_multibus_interrupt(7);
    }else{      
      multibus_interrupt(7);
    }
    break;

  case 0x18000 ... 0x18FFF: // multibus -> nubus map
    {
      int MAP_Addr = ((addr.raw-0x18000)>>2);
      int byte = addr.raw&0x03;
      MNA_MAP[MAP_Addr].byte[byte] = data;
    }
    break;

  case 0x01E000 ... 0x01FFFF: // CMOS RAM
    {
      int CMOS_Addr = ((addr.raw-0x01E000)>>2);
      CMOS_RAM[CMOS_Addr] = data;      
    }
    break;

  case 0x030000 ... 0x031FFF: // 3com Ethernet
    {
      uint16_t enet_addr = (addr.raw&0xFFFF);
      printf("SDU: 3COM WRITE: 0x%X = 0x%X\n",enet_addr,data);
      enet_write(enet_addr,data);
    }
    break;

  case 0x040000 ... 0x0EFFFF: // DYNAMICALLY ALLOCATED MAPPED AREA
    // This should have been caught above. The map must not have been set up.
    // Throw a multibus timeout.
    printf("multibus_write: timeout for addr 0x%X\n",addr.raw);
    PIC[0].IRQ |= 0x01;
    break;

  default:
    printf("multibus_write: Unknown addr 0x%X\n",addr.raw);
    ld_die_rq = 1;
  }
}

void multibus_interrupt(int irq){
  uint8_t irbit = 0x01;
  irbit <<= irq;
  /*
  if(PIC[2].IRQ & irbit){
    printf("SDU: FIRE MULTIBUS INTERRUPT %d (ALREADY SET!)\n",irq);
  }
  */
  PIC[2].IRQ |= irbit;
  // PIC[2].Last_IRQ &= ~irbit; // Ensure edge detector fires
  /*
  if(PIC[2].IMR & irbit){
    printf("SDU: FIRE MULTIBUS INTERRUPT %d (MASKED!)\n",irq);
  }
  */
}

void clear_multibus_interrupt(int irq){
  uint8_t irbit = 0x01;
  irbit <<= irq;
  if(PIC[2].IRQ & irbit){
    /*
    printf("SDU: CLEAR MULTIBUS INTERRUPT ");
    writeDec(irq);
    printf("\n");
    */
    PIC[2].IRQ ^= irbit;    
  } /* else{
    printf("SDU: CLEAR MULTIBUS INTERRUPT %d (ALREADY CLEAR!)\n",irq);
  } */
  // What about this?
  if(PIC[2].Last_IRQ&irbit){
    PIC[2].Last_IRQ ^= irbit;
  }
}

uint8_t i8088_port_read(uint32_t addr){
  uint8_t data;
  switch(addr){

  case 0x40: // INTERPHASE STATUS
    data = smd_read(0);
    // printf("SMD: status read 0x%X\n",data);
    return(data);
    break;

  case 0x80 ... 0x87: // MTI SERIAL
    PIC[0].IRQ |= 0x01; // MULTIBUS TIMEOUT
    return(0xFF);    
    break;

  case 0xA0 ... 0xA7: // EXCELAN
    PIC[0].IRQ |= 0x01; // MULTIBUS TIMEOUT
    return(0xFF);
    break;

  case 0x2000 ... 0x3F00: // TITN
    PIC[0].IRQ |= 0x01; // MULTIBUS TIMEOUT
    return(0xFF);    
    break;    
    
  default:
    printf("i8088_port_read: Unknown addr 0x%X\n",addr);
    ld_die_rq = 1;
    PIC[0].IRQ |= 0x01; // MULTIBUS TIMEOUT
    return(0xFF);    
  }
}

void i8088_port_write(uint32_t addr,uint8_t data){
  switch(addr){
  case 0x40: // SMD COMMAND
    smd_write(0,data);
    break;
  case 0x41: // SMD IOPB ADDR HI
    smd_write(1,data);
    break;
  case 0x42: // SMD IOPB ADDR MID
    smd_write(2,data);
    break;
  case 0x43: // SMD IOPB ADDR LO
    smd_write(3,data);
    break;
  case 0x60: // TAPEMASTER CHANNEL ATTN
    tapemaster_attn();
    break;
  case 0x61: // TAPEMASTER RESET
    tapemaster_reset();
    break;

  case 0x80 ... 0x87: // MTI SERIAL
    PIC[0].IRQ |= 0x01; // MULTIBUS TIMEOUT
    break;

  case 0xA0 ... 0xA7: // EXCELAN
    PIC[0].IRQ |= 0x01; // MULTIBUS TIMEOUT
    break;

  case 0x2000 ... 0x3F00: // TITN
    PIC[0].IRQ |= 0x01; // MULTIBUS TIMEOUT
    break;    
    
  default:
    printf("i8088_port_write: Unknown addr 0x%X w/ data 0x%X\n",addr,data);
    PIC[0].IRQ |= 0x01; // MULTIBUS TIMEOUT
    ld_die_rq = 1;
  }
}

void sdu_clock_pulse(){
  // Step 8088 PITs
  // The SDU PIT clock is 1.2288 MHz
  pit_clockpulse();
  // Drive console (HACK HACK)
  // PIT doesn't work properly yet so this fakes the approximate rate.
  pit_cycle_counter++;
  if(pit_cycle_counter == 520){
    if(sducons_tx_top > 0){
      sducons_write(sducons_tx_buf[sducons_tx_bot]);
      sducons_tx_bot++;
      if(sducons_tx_bot == sducons_tx_top){
	sducons_tx_bot = sducons_tx_top = 0;
	PIC[1].IRQ |= 0x02; // SERIO OUT INT
      }
    }
    pit_cycle_counter = 0;
  }
  // RTC updation
  if(RTC_REGB.Set == 0){
    // We are enabled
    rtc_cycle_count++;
    // Lambda has a 5MHz clock.
    // At 32 KHz, it takes 1984 microseconds to update the clock.
    if(rtc_cycle_count >= 5000000){
      rtc_cycle_count = 0;
      RTC_REGA.Update_In_Progress = 1;      
      RTC_Counter[RTC_SECONDS]++;
      if(RTC_Counter[RTC_SECONDS] > 59){
	RTC_Counter[RTC_SECONDS] = 0;
	RTC_Counter[RTC_MINUTES]++;
	if(RTC_Counter[RTC_MINUTES] > 59){
	  uint8_t Inc_Day=0;
	  RTC_Counter[RTC_MINUTES] = 0;
	  RTC_Counter[RTC_HOURS]++;
	  if(RTC_REGB.Format == 0){
	    if(RTC_Counter[RTC_HOURS] > 13 && RTC_Counter[RTC_HOURS] < 0x81){ 
	      // Convert to PM
	      RTC_Counter[RTC_HOURS] = 0x81;
	    } 
	    if(RTC_Counter[RTC_HOURS] > 0x8C){
	      RTC_Counter[RTC_HOURS] = 1;
	      Inc_Day = 1;
	    }
	  }else{
	    if(RTC_Counter[RTC_HOURS] > 23){
	      RTC_Counter[RTC_HOURS] = 0;
	      Inc_Day = 1;
	    }
	  }
	  if(Inc_Day == 1){
	    uint8_t Last_Day=0;;
	    RTC_Counter[RTC_DAY_OF_WEEK]++;
	    RTC_Counter[RTC_DATE_OF_MONTH]++;
	    if(RTC_Counter[RTC_DAY_OF_WEEK] > 7){ RTC_Counter[RTC_DAY_OF_WEEK] = 1; }
	    switch(RTC_Counter[RTC_MONTH]){
	    case 1: 
	    case 3:
	    case 5:
	    case 7:
	    case 8:
	    case 10:
	    case 12:
	      Last_Day = 31; break;
	    case 2:
	      if(RTC_Counter[RTC_MONTH]%400 == 0){
		Last_Day = 29; break;
	      }else{
		if(RTC_Counter[RTC_MONTH]%100 == 0){
		  Last_Day = 28; break;
		}else{
		  if(RTC_Counter[RTC_MONTH]%4 == 0){
		    Last_Day = 29; break;
		  }else{
		    Last_Day = 28; break;
		  }
		}
	      }
	      break;
	    case 4:
	    case 6:
	    case 9:
	    case 11:
	      Last_Day = 30; break;
	    }
	    if(RTC_Counter[RTC_DATE_OF_MONTH] > Last_Day){
	      RTC_Counter[RTC_DATE_OF_MONTH] = 1;
	      RTC_Counter[RTC_MONTH]++;
	      if(RTC_Counter[RTC_MONTH] > 12){
		RTC_Counter[RTC_MONTH] = 1;
		RTC_Counter[RTC_YEAR]++;
		if(RTC_Counter[RTC_YEAR] > 99){
		  // TODO: CAUSE APOCALYPSE
		  RTC_Counter[RTC_YEAR] = 0;
		}
	      }
	    }
	  }
	}
      }
      // TODO: ALARM INTERRUPT CHECK (if lisp uses it)
    }
    if(RTC_REGA.Update_In_Progress != 0 && rtc_cycle_count > 9920){
      RTC_REGA.Update_In_Progress = 0;      
    }
  }

  // SDU nubus interface
  // If the bus is busy and not acknowledged...
  if(NUbus_Busy == 2 && NUbus_acknowledge == 0){
    // Is it us?
    if(NUbus_Address.Card == 0xFF){
      // Yes, answer.

      // THIS IS THE SDU ADDRESS SPACE LAYOUT (AS SEEN FROM THE NUBUS)
      // 0x000000 - 0x00FFFF = SDU RAM
      // 0x018000 - 0x018FFF = Multibus -> NUbus map control registers
      // 0x01C000 - 0x01C3FF = SDU register page (serial ports, etc)
      // 0x01E000 - 0x01FFFF = CMOS RAM
      // 0x02F800 - 0x02FBFF = Second burr-brown
      // 0x02FC00 - 0x02FFFF = First burr-brown
      // 0x030000 - 0x031FFF = 3Com ethernet
      // 0x040000 - 0x0EFFFF = DYNAMICALLY ALLOCATED MAPPED AREA (MAPPED TO NUBUS!)
      // 0x0F0000 - 0x0FFFFF = SDU ROM

      switch(NUbus_Address.Addr){
      case 0x000000 ... 0x00FFFF: // SDU RAM
	{
	  uint32_t MEM_Addr = NUbus_Address.Addr;
	  if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	    if(NUbus_Request == VM_READ){ // Read four bytes
	      switch(NUbus_Address.Byte){
	      case 1: // Read Low Half
		NUbus_Data.byte[1] = SDU_RAM[MEM_Addr];
		NUbus_Data.byte[0] = SDU_RAM[MEM_Addr-1];
		break;
		
	      case 2: // Block Transfer
		printf("SDU: BLOCK READ REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Read High Half
		NUbus_Data.byte[3] = SDU_RAM[MEM_Addr];
		NUbus_Data.byte[2] = SDU_RAM[MEM_Addr-1];
		break;
		
	      case 0:
		// Full word read
		NUbus_Data.byte[3] = SDU_RAM[MEM_Addr+3];
		NUbus_Data.byte[2] = SDU_RAM[MEM_Addr+2];
		NUbus_Data.byte[1] = SDU_RAM[MEM_Addr+1];
		NUbus_Data.byte[0] = SDU_RAM[MEM_Addr];
		break;
	      }
	    }else{
	      // BYTE READ
	      NUbus_Data.byte[NUbus_Address.Byte] = SDU_RAM[MEM_Addr];
	    }
	    if(SDU_RAM_trace){
	      printf("SDU: RAM Read: Request %o Addr 0x%X (0x%X",NUbus_Request,NUbus_Address.raw,MEM_Addr);
	      if(MEM_Addr >= sysconf_base && MEM_Addr <= (sysconf_base+sizeof(system_configuration_qs))){
		uint32_t Offset = (MEM_Addr-sysconf_base)/4;
		printf(", sysconf %s",sysconf_q_names[Offset]);
	      }
	      if(MEM_Addr >= proc0_conf_base && MEM_Addr <= (proc0_conf_base+sizeof(processor_configuration_qs))){
		uint32_t Offset = (MEM_Addr-proc0_conf_base)/4;
		printf(", proc0_conf %s",proc_conf_q_names[Offset]);
	      }
	      if(MEM_Addr >= proc1_conf_base && MEM_Addr <= (proc1_conf_base+sizeof(processor_configuration_qs))){
		uint32_t Offset = (MEM_Addr-proc1_conf_base)/4;
		printf(", proc1_conf %s",proc_conf_q_names[Offset]);
	      }
	      printf(")");
	      if(NUbus_Data.word == 0){
		printf(" returned zeroes");
	      }else{
		printf(" returned 0x%X",NUbus_Data.word);
	      }
	      printf("\n");	  
	    }
	    NUbus_acknowledge=1;
	    return;
	  }
	  // Write
	  if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	    if(NUbus_Request == VM_BYTE_WRITE){
	      SDU_RAM[MEM_Addr] = NUbus_Data.byte[NUbus_Address.Byte];               // Store data
	    }else{
	      // WORD WRITE
	      switch(NUbus_Address.Byte){
	      case 1: // Write low half
		SDU_RAM[MEM_Addr-1] = NUbus_Data.byte[0];
		SDU_RAM[MEM_Addr] = NUbus_Data.byte[1];
		break;
		
	      case 2: // BLOCK TRANSFER
		printf("SDU: BLOCK TRANSFER REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Write high half
		SDU_RAM[MEM_Addr-1] = NUbus_Data.byte[2];
		SDU_RAM[MEM_Addr] = NUbus_Data.byte[3];
		break;
		
	      case 0: // Full Word
		SDU_RAM[MEM_Addr] = NUbus_Data.byte[0];
		SDU_RAM[MEM_Addr+1] = NUbus_Data.byte[1];
		SDU_RAM[MEM_Addr+2] = NUbus_Data.byte[2];
		SDU_RAM[MEM_Addr+3] = NUbus_Data.byte[3];
		break;
	      }
	    }
	    if(SDU_RAM_trace){
	      printf("SDU: RAM Write: Request %o Addr 0x%X (0x%X",NUbus_Request,NUbus_Address.raw,MEM_Addr);
	      if(MEM_Addr >= sysconf_base && MEM_Addr <= (sysconf_base+sizeof(system_configuration_qs))){
		uint32_t Offset = (MEM_Addr-sysconf_base)/4;
		printf(", sysconf %s",sysconf_q_names[Offset]);
	      }
	      if(MEM_Addr >= proc0_conf_base && MEM_Addr <= (proc0_conf_base+sizeof(processor_configuration_qs))){
		uint32_t Offset = (MEM_Addr-proc0_conf_base)/4;
		printf(", proc0_conf %s",proc_conf_q_names[Offset]);
	      }
	      if(MEM_Addr >= proc1_conf_base && MEM_Addr <= (proc1_conf_base+sizeof(processor_configuration_qs))){
		uint32_t Offset = (MEM_Addr-proc1_conf_base)/4;
		printf(", proc1_conf %s",proc_conf_q_names[Offset]);
	      }
	      printf(") data 0x%X\n",NUbus_Data.word);
	    }
	    NUbus_acknowledge=1;
	    return;
	  }
	}
	break;

      case 0x0F0000 ... 0x0FFFFF: // SDU ROM
	{
	  uint32_t MEM_Addr = NUbus_Address.Addr-0x0F0000;
	  if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	    if(NUbus_Request == VM_READ){ // Read four bytes
	      switch(NUbus_Address.Byte){
	      case 1: // Read Low Half
		NUbus_Data.byte[1] = SDU_ROM[MEM_Addr];
		NUbus_Data.byte[0] = SDU_ROM[MEM_Addr-1];
		break;
		
	      case 2: // Block Transfer
		printf("SDU: BLOCK READ REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Read High Half
		NUbus_Data.byte[3] = SDU_ROM[MEM_Addr];
		NUbus_Data.byte[2] = SDU_ROM[MEM_Addr-1];
		break;
		
	      case 0:
		// Full word read
		NUbus_Data.byte[3] = SDU_ROM[MEM_Addr+3];
		NUbus_Data.byte[2] = SDU_ROM[MEM_Addr+2];
		NUbus_Data.byte[1] = SDU_ROM[MEM_Addr+1];
		NUbus_Data.byte[0] = SDU_ROM[MEM_Addr];
		break;
	      }
	    }else{
	      // BYTE READ
	      NUbus_Data.byte[NUbus_Address.Byte] = SDU_ROM[MEM_Addr];
	    }
	    NUbus_acknowledge=1;
	    return;
	  }
	  printf("SDU: Write to ROM?\n");
	}
	break;

      case 0x1E000 ... 0x1FFFF: // CMOS
	{
	  uint32_t CMOS_Addr = ((NUbus_Address.Addr-0x1E000)>>2);
	  // Handle it
	  if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	    if(NUbus_Request == VM_READ){ // Read four bytes
	      switch(NUbus_Address.Byte){
	      case 1: // Read Low Half
		NUbus_Data.byte[1] = 0;
		NUbus_Data.byte[0] = CMOS_RAM[CMOS_Addr];
		break;
		
	      case 2: // Block Transfer
		printf("SDU: BLOCK READ REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Read High Half
		NUbus_Data.byte[3] = 0;
		NUbus_Data.byte[2] = CMOS_RAM[CMOS_Addr];
		break;
		
	      case 0:
		// Full word read
		NUbus_Data.byte[3] = NUbus_Data.byte[2] = NUbus_Data.byte[1] = 0;
		NUbus_Data.byte[0] = CMOS_RAM[CMOS_Addr];
		break;
	      }
	    }else{
	      NUbus_Data.byte[NUbus_Address.Byte] = CMOS_RAM[CMOS_Addr];
	    }
	    NUbus_acknowledge=1;
	    return;
	  }
	  if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	    ld_die_rq = 1;
	  }
	}
	break;

      case 0x18000 ... 0x18FFF: // multibus -> nubus map
	{
	  int MAP_Addr = ((NUbus_Address.Addr-0x18000)>>2);
	  /*      
		  1024 registers
		  each 32 bits, of which 24 are present.
		  1 bit enable, 1 bit unused, 22 bits hi NUBUS adr.
		  divide 20 bit multibus into 2 10 bit pieces:
		  each register maps a 1K byte segment to a 1K nubus page.	  
	  */
	  /* THEY ONLY WORK IF REFERENCED ON SINGLE BYTE TRANSFERS. (???) */
	  if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	    if(NUbus_Request == VM_READ){ // Read four bytes
	      switch(NUbus_Address.Byte){
	      case 1: // Read Low Half
		NUbus_Data.byte[1] = MNA_MAP[MAP_Addr].byte[1];
		NUbus_Data.byte[0] = MNA_MAP[MAP_Addr].byte[0];
		break;
		
	      case 2: // Block Transfer
		printf("SDU: BLOCK READ REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Read High Half
		NUbus_Data.byte[3] = MNA_MAP[MAP_Addr].byte[3];
		NUbus_Data.byte[2] = MNA_MAP[MAP_Addr].byte[2];
		break;
		
	      case 0:
		// Full word read, supposedly does not work
		NUbus_Data.word = MNA_MAP[MAP_Addr].word;
		break;
	      }
	    }else{
	      // BYTE READ
	      NUbus_Data.byte[NUbus_Address.Byte] = MNA_MAP[MAP_Addr].byte[NUbus_Address.Byte]; 
	    }
	    NUbus_acknowledge=1;
	    return;
	  }
	  // Write
	  if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	    if(NUbus_Request == VM_BYTE_WRITE){
	      MNA_MAP[MAP_Addr].byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
	    }else{
	      // WORD WRITE
	      switch(NUbus_Address.Byte){
	      case 1: // Write low half
		MNA_MAP[MAP_Addr].byte[0] = NUbus_Data.byte[0];
		MNA_MAP[MAP_Addr].byte[1] = NUbus_Data.byte[1];
		break;
		
	      case 2: // BLOCK TRANSFER
		printf("SDU: BLOCK TRANSFER REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Write high half
		MNA_MAP[MAP_Addr].byte[2] = NUbus_Data.byte[2];
		MNA_MAP[MAP_Addr].byte[3] = NUbus_Data.byte[3];
		break;
		
	      case 0: // Full Word, supposedly does not work
		MNA_MAP[MAP_Addr].word = NUbus_Data.word;
		break;
	      }
	    }
	    NUbus_acknowledge=1;
	    // Debug log
	    if(NUbus_trace || SDU_RAM_trace){
	      printf("SDU: MNA MAP ent 0x%X wrote: Enable %X Spare %X NB-Page 0x%X (0x%X)\n",
		     MAP_Addr,MNA_MAP[MAP_Addr].Enable,MNA_MAP[MAP_Addr].Spare,
		     MNA_MAP[MAP_Addr].NUbus_Page,(MNA_MAP[MAP_Addr].NUbus_Page<<10));
	    }
	    return;
	  }
	}
	break;	

      // Multibus IO space.
      /*
      case 0x19000 ... 0x80000: 
	// What are these?
	if(NUbus_Request == VM_READ){
	  // Let's see where this goes!
	  NUbus_Data.word = 0x0; // DEADBEEF;
	  NUbus_acknowledge=1;	  
	}else{
	  printf("SDU: Unimplemented address ");
	  writeH32(NUbus_Address.Addr);
	  printf(" (");
	  writeH32(NUbus_Address.raw);
	  printf(")\n");
	  ld_die_rq=1;	  
	}
	return;
        break;
      */

      // 0x1c000 - 0x1c3FF: SDU register space

      case 0x1c120: // RTC Data Register
	{
	  nuData RTC_Data;
	  RTC_Data.word = 0;
	  switch(rtc_addr){
	  case 0x00 ... 0x09:
	    if(RTC_REGA.Update_In_Progress == 0){
	      RTC_Data.word = RTC_Counter[rtc_addr];
	    }else{
	      RTC_Data.word = 0;
	    }
	    break;
	  case 0x0A:
	    RTC_Data.word = RTC_REGA.byte;
	    break;
	  case 0x0B:
	    RTC_Data.word = RTC_REGB.byte;
	    break;
	  case 0x0C:
	    RTC_Data.word = RTC_REGC.byte;
	    break;
	  case 0x0D:
	    RTC_Data.word = RTC_REGD.byte;
	    RTC_REGD.Valid_RAM = 1; // Reading Reg D sets this bit
	    break;
	  case 0x0E ... 0x3F:
	    RTC_Data.word = RTC_RAM[rtc_addr-0x0E];
	    break;
	  default:
	    RTC_Data.word = 0;	    
	  }
	  if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	    if(NUbus_Request == VM_READ){
	      switch(NUbus_Address.Byte){
	      case 1: // Read Low Half
		NUbus_Data.byte[1] = RTC_Data.byte[1];
		NUbus_Data.byte[0] = RTC_Data.byte[0];
		break;
		
	      case 2: // Block Transfer
		printf("SDU: BLOCK READ REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Read High Half
		NUbus_Data.byte[3] = RTC_Data.byte[3];
		NUbus_Data.byte[2] = RTC_Data.byte[2];
		break;
		
	      case 0:
		// Full word read
		NUbus_Data.word = RTC_Data.word;
		break;
	      }
	    }else{
	      // BYTE READ
	      NUbus_Data.byte[NUbus_Address.Byte] = RTC_Data.byte[NUbus_Address.Byte];	    
	    }
	    printf("RTC: Data Read, returned 0x%X for addr 0x%X and request %o\n",
		   NUbus_Data.word,NUbus_Address.raw,NUbus_Request);
	    NUbus_acknowledge=1;
	    return;
	  }
	  if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	    if(NUbus_Request == VM_WRITE){
	      switch(NUbus_Address.Byte){
	      case 1: // Write Low Half
		RTC_Data.byte[1] = NUbus_Data.byte[1];	      
		RTC_Data.byte[0] = NUbus_Data.byte[0];	      
		break;
	      
	      case 2: // Block Transfer
		printf("SDU: BLOCK WRITE REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Write High Half
		RTC_Data.byte[3] = NUbus_Data.byte[3];	      
		RTC_Data.byte[2] = NUbus_Data.byte[2];	      
		break;
		
	      case 0:
		// Full word write
		RTC_Data.word = NUbus_Data.word;
		break;
	      }
	    }else{
	      // BYTE WRITE
	      RTC_Data.byte[NUbus_Address.Byte] = NUbus_Data.byte[NUbus_Address.Byte];
	    }
	    // Process bits
	    switch(rtc_addr){
	    case 0x00 ... 0x09:
	      if(RTC_REGA.Update_In_Progress == 0){
		RTC_Counter[rtc_addr] = RTC_Data.byte[0];
	      }
	      break;
	    case 0x0A:
	      // UIP bit is read only
	      RTC_REGA.byte &= 0x80;
	      RTC_REGA.byte |= (RTC_Data.byte[0]&0x7F);
	      break;
	    case 0x0B:
	      RTC_REGB.byte = RTC_Data.byte[0];
	      break;
	    case 0x0C:
	      // Read Only
	      break;
	    case 0x0D:
	      // Read Only
	      break;
	    case 0x0E ... 0x3F:
	      RTC_RAM[rtc_addr-0x0E] = RTC_Data.byte[0];
	      break;
	    }
	    printf("RTC: Data Write, data 0x%X\n",NUbus_Data.word);
	    NUbus_acknowledge=1;
	    return;
	  }
	}
	break;
      case 0x1c124: // RTC Address Register
	// Lisp will write an address here
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  if(NUbus_Request == VM_READ){
	    switch(NUbus_Address.Byte){
	    case 1: // Read Low Half
	      NUbus_Data.byte[1] = 0;
	      NUbus_Data.byte[0] = rtc_addr;
	      break;
	      
	    case 2: // Block Transfer
	      printf("SDU: BLOCK READ REQUESTED\n");
	      ld_die_rq=1;
	      break;
		
	    case 3: // Read High Half
	      NUbus_Data.byte[3] = 0;
	      NUbus_Data.byte[2] = 0;
	      break;
	      
	    case 0:
	      // Full word read
	      NUbus_Data.word = rtc_addr;
	      break;
	    }
	  }else{
	    // BYTE READ
	    if(NUbus_Address.Byte == 0){
	      NUbus_Data.byte[NUbus_Address.Byte] = rtc_addr;
	    }else{
	      NUbus_Data.byte[NUbus_Address.Byte] = 0;
	    }
	  }
	  printf("RTC: Address Read, returned 0x%X for addr 0x%X and request %o\n",
		 NUbus_Data.word,NUbus_Address.raw,NUbus_Request);
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  if(NUbus_Request == VM_WRITE){
	    switch(NUbus_Address.Byte){
	    case 1: // Write Low Half
	      rtc_addr = NUbus_Data.byte[0];	      
	      break;

	    case 2: // Block Transfer
	      printf("SDU: BLOCK WRITE REQUESTED\n");
	      ld_die_rq=1;
	      break;

	    case 3: // Write High Half
	      break;

	    case 0:
	      // Full word write
	      rtc_addr = NUbus_Data.word;
	      break;
	    }
	  }else{
	    // BYTE WRITE
	    if(NUbus_Address.Byte == 0){ rtc_addr = NUbus_Data.byte[NUbus_Address.Byte]; }
	  }
	  // Process bits
	  printf("RTC: Address Write, data 0x%X\n",NUbus_Data.word);
	  NUbus_acknowledge=1;
	  return;
	}
	break;

      case 0x01c1fc: // Multibus Interrupt 7
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  // extern int SMD_Controller_State;
	  extern int SDU_disk_trace;
	  if(NUbus_Data.byte[0] != 0){
	    if(SDU_disk_trace){
	      printf("SDU: Set Interrupt 7\n");
	    }
	    multibus_interrupt(7);
	    /*
	      if(SMD_Controller_State == 0){
	        // Start operations
		SMD_Controller_State = 93;
	      }
	    */
	  }else{
	    if(SDU_disk_trace){
	      printf("SDU: Clear Interrupt 7\n");
	    }
	    clear_multibus_interrupt(7);
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	break;

#ifdef BURR_BROWN
	// BURR-BROWN DEBUGGING MASTER
	/*
      case 0x02ff02: // CSR (when read), 
      case 0x02ff04: // LO DATA (0x000000FF)
      case 0x02ff05: // HI DATA (0x0000FF00)
	*/
      case 0x02ff02: // CSR
	// Writing 7 = DRIVE DATA LINES
	// Writing 4 = DON'T DRIVE DATA LINES
        if(NUbus_Request == VM_BYTE_WRITE && NUbus_Data.byte[2] == 07){
	  BB_Drive_Data_Lines = 1;
          NUbus_acknowledge=1;
	  break;
        }
        if(NUbus_Request == VM_BYTE_WRITE && NUbus_Data.byte[2] == 04){
	  BB_Drive_Data_Lines = 0;
          NUbus_acknowledge=1;
	  break;
        }
	printf("BURR-BROWN: Unimplemented Request %o Addr 0x%X Data 0x%X\n",
	       NUbus_Request,NUbus_Address.raw,NUbus_Data.word);
	ld_die_rq = 1;
	break;

      case 0x02ff04: // DATA / LO DATA
	if(NUbus_Request == VM_WRITE){
	  BB_Data.hword[0] = NUbus_Data.hword[0];
	  NUbus_acknowledge=1;
	  break;
	}
	if(NUbus_Request == VM_BYTE_WRITE){
	  BB_Data.byte[0] = NUbus_Data.byte[0];
	  NUbus_acknowledge=1;
	  break;
	}
	if(NUbus_Request == VM_READ){
	  NUbus_Data.hword[0] = BB_Data.hword[0];
	  NUbus_Data.hword[1] = 0;
	  NUbus_acknowledge=1;
	  break;
	}
        printf("BURR-BROWN: Unimplemented Request %o Addr 0x%X Data 0x%X\n",
               NUbus_Request,NUbus_Address.raw,NUbus_Data.word);
	ld_die_rq = 1;
	break;

      case 0x02ff05: // HI DATA
	if(NUbus_Request == VM_BYTE_WRITE){
	  BB_Data.byte[1] = NUbus_Data.byte[1];
	  NUbus_acknowledge=1;
	  break;
	}
        printf("BURR-BROWN: Unimplemented Request %o Addr 0x%X Data 0x%X\n",
               NUbus_Request,NUbus_Address.raw,NUbus_Data.word);
	ld_die_rq = 1;
	break;

      case 0x02ff06: // Control
	if(NUbus_Request == VM_BYTE_WRITE){	  
	  BB_Reg = (~NUbus_Data.byte[2])&0x03;
	    
	  // 0x08 = REQ.L (Strobe)
	  // 0x04 = READ DATA
	  // 0x03 = REG (inverted)
	  if(NUbus_Data.byte[2]&0x08){
	    printf("BURR-BROWN: ");
	    if(NUbus_Data.byte[2]&0x04){
	      printf("READ ");
	    }else{
	      printf("WRITE ");
	    }
	    printf("REG %X",BB_Reg);
	    if(!(NUbus_Data.byte[2]&0x04)){
	      printf(" DATA 0x%X",BB_Data.word);
	      // WRITE EXECUTION
	      // WRITE REG 0 DATA 0x2
	      switch(BB_Reg){
	      case 0: // MODE REG
		BB_Mode_Reg = BB_Data.byte[0];
		if(BB_Mode_Reg&0x02){
		  // RESET
		  extern uint8_t debug_master_mode;
		  printf(" (RESET)");
		  // Does what?
		  debug_master_mode = 1;
		  debug_connect();
		}
		break;
	      case 1:
		if(BB_Mode_Reg&0x01){
		  // START WRITE
		  printf(" (START WRITE: ADDR 0x%X DATA 0x%X)",BB_Remote_Addr.raw,BB_Remote_Data.word);
		  if(BB_Drive_Data_Lines != 1){ printf(" !!DDL!!"); }
		  if(!(BB_Mode_Reg&0x04)){
		    debug_tx_rq(VM_BYTE_WRITE,BB_Remote_Addr.raw,BB_Remote_Data.word);
		  }else{
		    debug_tx_rq(VM_WRITE,BB_Remote_Addr.raw,BB_Remote_Data.word);
		  }
		}else{
		  // Low Data Load
		  BB_Remote_Data.hword[0] = BB_Data.hword[0];
		}
		break;
	      case 2:
		if(BB_Mode_Reg&0x01){
		  // Low Addr Load
		  BB_Remote_Addr.hword[0] = BB_Data.hword[0];
		}else{
		  // Hi Data Load
		  BB_Remote_Data.hword[1] = BB_Data.hword[0];
		}
		break;
	      case 3:
		if(BB_Mode_Reg&0x01){
		  // Hi Addr Load
		  BB_Remote_Addr.hword[1] = BB_Data.hword[0];
		}else{
		  // NC
		}
		break;
	      }
	    }else{
	      // READ EXECUTION
	      switch(BB_Reg){
	      case 0: // MODE REG
		// 0x300 = RESPONSE BITS
		// Let's try this
		while(BB_Remote_Result == 1){
		  debug_clockpulse(); // Hang for IO
		}
		if(BB_Remote_Result == 2){
		  // Success
		  BB_Data.word = 0x0300|BB_Mode_Reg;
		}
		if(BB_Remote_Result == 3){
		  // Error
		  BB_Data.word = 0x0100|BB_Mode_Reg;
		}
		break;
	      case 1:
		if(BB_Mode_Reg&0x01){
		  // START READ
		  printf(" (START READ: ADDR 0x%X)",BB_Remote_Addr.raw);
		  if(!(BB_Mode_Reg&0x04)){
		    debug_tx_rq(VM_BYTE_READ,BB_Remote_Addr.raw,BB_Remote_Data.word);
		  }else{
		    debug_tx_rq(VM_READ,BB_Remote_Addr.raw,BB_Remote_Data.word);
		  }
		}else{
		  // NC
		}
		break;
	      case 2:
		if(BB_Mode_Reg&0x01){
		  // Lo Data Read
		  BB_Data.hword[0] = BB_Remote_Data.hword[0];
		}else{
		  // NC
		}
		break;
	      case 3:
		if(BB_Mode_Reg&0x01){
		  // Hi Data Read
		  BB_Data.hword[0] = BB_Remote_Data.hword[1];		  
		}else{
		  // NC
		}
		break;

	      default:
		printf(" (BADREG)");
		ld_die_rq = 1;
	      }
	    }
	    printf("\n");
	  }
	  NUbus_acknowledge=1;
	  break;
	}
	printf("BURR-BROWN: Unimplemented Request %o Addr 0x%X Data 0x%X\n",
	       NUbus_Request,NUbus_Address.raw,NUbus_Data.word);
	ld_die_rq = 1;
	break;
#endif

      case 0x030000 ... 0x31FFF: // 3com Ethernet
	{
	  uint16_t enet_addr = (NUbus_Address.raw&0xFFFF);
	  if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	    if(NUbus_Request == VM_READ){
	      switch(NUbus_Address.Byte){
	      case 1: // Read Low Half
		NUbus_Data.byte[0] = enet_read(enet_addr-1);
		NUbus_Data.byte[1] = enet_read(enet_addr);
		break;
		
	      case 2: // Block Transfer
		printf("SDU: BLOCK READ REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Read High Half
		NUbus_Data.byte[2] = enet_read(enet_addr-1);
		NUbus_Data.byte[3] = enet_read(enet_addr);
		break;
		
	      case 0:
		// Full word read
		NUbus_Data.byte[0] = enet_read(enet_addr);
		NUbus_Data.byte[1] = enet_read(enet_addr+1);
		NUbus_Data.byte[2] = enet_read(enet_addr+2);
		NUbus_Data.byte[3] = enet_read(enet_addr+3);
		break;
	      }
	    }else{
	      // BYTE READ
	      NUbus_Data.byte[NUbus_Address.Byte] = enet_read(enet_addr);
	    }
	    if(NUbus_trace == 1){
	      printf("3COM: CSR Read, returned 0x%X for addr 0x%X and request %o\n",
		     NUbus_Data.word,NUbus_Address.raw,NUbus_Request);
	    }
	    NUbus_acknowledge=1;
	    return;
	  }	
	  if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
            if(NUbus_Request == VM_WRITE){
              switch(NUbus_Address.Byte){
              case 1: // Write Low Half
		enet_write(enet_addr-1,NUbus_Data.byte[0]);
		enet_write(enet_addr,NUbus_Data.byte[1]);
                break;

              case 2: // Block Transfer
                printf("SDU: BLOCK WRITE REQUESTED\n");
                ld_die_rq=1;
                break;

              case 3: // Write High Half
		enet_write(enet_addr-1,NUbus_Data.byte[2]);
		enet_write(enet_addr,NUbus_Data.byte[3]);
                break;

              case 0:
                // Full word write
		enet_write(enet_addr,NUbus_Data.byte[0]);
		enet_write(enet_addr+1,NUbus_Data.byte[1]);
		enet_write(enet_addr+2,NUbus_Data.byte[2]);
		enet_write(enet_addr+3,NUbus_Data.byte[3]);
                break;
              }
            }else{
              // BYTE WRITE
	      enet_write(enet_addr,NUbus_Data.byte[NUbus_Address.Byte]);
            }
	    NUbus_acknowledge=1;
	    return;
	  }
	}
	break;

      case 0x100100: // SMD controller status/command registers
	{
	  if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	    // On read, it's the status register
	    if(NUbus_Request == VM_READ){ 
	      switch(NUbus_Address.Byte){
	      case 1: // Read Low Half
		NUbus_Data.byte[1] = 0;
		NUbus_Data.byte[0] = smd_read(0);
		break;

	      case 2: // Block Transfer
		printf("SDU: BLOCK READ REQUESTED\n");
		ld_die_rq=1;
		break;

	      case 3: // Read High Half
		NUbus_Data.byte[3] = 0;
		NUbus_Data.byte[2] = smd_read(0);
		break;

	      case 0:
		// Full word read
		NUbus_Data.word = smd_read(0);
		break;
	      }
	    }else{
	      // BYTE READ
	      NUbus_Data.byte[NUbus_Address.Byte] = smd_read(0);
	    }
	    NUbus_acknowledge=1;
	    return;
	  }	  
	  if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	    // On write, it's the command register
	    // SMD_RCmd
	    if(NUbus_Request == VM_WRITE){
	      switch(NUbus_Address.Byte){
	      case 1: // Write Low Half
		smd_write(0,NUbus_Data.byte[0]);
		break;
		
	      case 2: // Block Transfer
		printf("SDU: BLOCK WRITE REQUESTED\n");
		ld_die_rq=1;
		break;
		
	      case 3: // Write High Half
		smd_write(0,NUbus_Data.byte[2]);
		break;
		
	      case 0:
		// Full word write
		smd_write(0,NUbus_Data.word);
		break;
	      }
	    }else{
	      // BYTE WRITE
	      smd_write(0,NUbus_Data.byte[NUbus_Address.Byte]);
	    }
	    NUbus_acknowledge=1;
	    return;
	  }	    
	}
	break;
      case 0x100104: // SMD controller IOPB base (hi)
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  ld_die_rq = 1;
	}
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  if(NUbus_Request == VM_WRITE){
	    switch(NUbus_Address.Byte){
	    case 1: // Write Low Half
	      smd_write(1,NUbus_Data.byte[0]);
	      break;

	    case 2: // Block Transfer
	      printf("SDU: BLOCK WRITE REQUESTED\n");
	      ld_die_rq=1;
	      break;

	    case 3: // Write High Half
	      smd_write(1,NUbus_Data.byte[2]);
	      break;

	    case 0:
	      // Full word write
	      smd_write(1,NUbus_Data.word);
	      break;
	    }
	  }else{
	    // BYTE WRITE
	    smd_write(1,NUbus_Data.byte[NUbus_Address.Byte]);
	  }
	  NUbus_acknowledge=1;
	  return;
	}	
	break;
      case 0x100108: // SMD controller IOPB base (middle)
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  ld_die_rq = 1;
	}
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  if(NUbus_Request == VM_WRITE){
	    switch(NUbus_Address.Byte){
	    case 1: // Write Low Half
	      smd_write(2,NUbus_Data.byte[0]);
	      break;

	    case 2: // Block Transfer
	      printf("SDU: BLOCK WRITE REQUESTED\n");
	      ld_die_rq=1;
	      break;

	    case 3: // Write High Half
	      smd_write(2,NUbus_Data.byte[2]);
	      break;

	    case 0:
	      // Full word write
	      smd_write(2,NUbus_Data.word);
	      break;
	    }
	  }else{
	    // BYTE WRITE
	    smd_write(2,NUbus_Data.byte[NUbus_Address.Byte]);
	  }
	  NUbus_acknowledge=1;
	  return;
	}	
	break;
      case 0x10010c: // SMD controller IOPB base (low)
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  ld_die_rq = 1;
	}
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  if(NUbus_Request == VM_WRITE){
	    switch(NUbus_Address.Byte){
	    case 1: // Write Low Half
	      smd_write(3,NUbus_Data.byte[0]);
	      break;

	    case 2: // Block Transfer
	      printf("SDU: BLOCK WRITE REQUESTED\n");
	      ld_die_rq=1;
	      break;

	    case 3: // Write High Half
	      smd_write(3,NUbus_Data.byte[2]);
	      break;

	    case 0:
	      // Full word write
	      smd_write(3,NUbus_Data.word);
	      break;
	    }
	  }else{
	    // BYTE WRITE
	    smd_write(3,NUbus_Data.byte[NUbus_Address.Byte]);
	  }
	  NUbus_acknowledge=1;
	  return;
	}	
	break;
      case 0x100110: // SMD controller IOPB base (lower)
	// Lisp does this deliberately. Why?
	if(NUbus_Request == VM_WRITE){
	  switch(NUbus_Address.Byte){
	  case 1: // Write Low Half
	    smd_write(4,NUbus_Data.byte[0]);
	    break;
	    
	  case 2: // Block Transfer
	    printf("SDU: BLOCK WRITE REQUESTED\n");
	    ld_die_rq=1;
	    break;
	    
	  case 3: // Write High Half
	    smd_write(4,NUbus_Data.byte[2]);
	    break;
	    
	  case 0:
	    // Full word write
	    smd_write(4,NUbus_Data.word);
	    break;
	  }
	  NUbus_acknowledge=1;
	  break;
	}
	// Otherwise...
	printf("SDU: Unimplemented address 0x%X (0x%X)\n",
	       NUbus_Address.Addr,NUbus_Address.raw);
	lambda_dump(DUMP_ALL);
	ld_die_rq=1;
	break;
	
	// Tapemaster controller
      case 0x100180:
	// Channel Attention
	if(NUbus_Request == VM_WRITE){
	  tapemaster_attn();
	  NUbus_acknowledge=1;
	  return;
	}
	// Falls into...
      case 0x100184:
	// Reset CPU
	if(NUbus_Request == VM_WRITE){
	  tapemaster_reset();
          NUbus_acknowledge=1;
          return;
        }
	// Die if we ended up here
        printf("SDU: Unimplemented address 0x%X (0x%X)\n",
               NUbus_Address.Addr,NUbus_Address.raw);
	lambda_dump(DUMP_ALL);
	ld_die_rq=1;
	break;

	// Excelan (we don't have one)
      case 0x100280 ... 0x1002E0:
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  if(NUbus_Request == VM_READ){ 	    
	    switch(NUbus_Address.Byte){
	    case 1: // Read Low Half
	      NUbus_Data.byte[0] = 0xFF;
	      NUbus_Data.byte[1] = 0;
	      break;
	      
	    case 2: // Block Transfer
	      printf("SDU: BLOCK READ REQUESTED\n");
	      ld_die_rq=1;
	      break;
	      
	    case 3: // Read High Half
	      NUbus_Data.byte[2] = 0xFF;
	      NUbus_Data.byte[3] = 0;
	      break;
	      
	    case 0:
	      // Full word read
	      NUbus_Data.byte[0] = 0xFF;
	      NUbus_Data.byte[1] = 0;
	      NUbus_Data.byte[2] = 0;
	      NUbus_Data.byte[3] = 0;
	      break;
	    }
	  }else{
	    // BYTE READ
	    NUbus_Data.byte[NUbus_Address.Byte] = 0xFF;
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	if(NUbus_Request == VM_WRITE || NUbus_Request == VM_BYTE_WRITE){
	  if(NUbus_Request == VM_WRITE){
	    switch(NUbus_Address.Byte){
	    case 1: // Write Low Half
	      break;

	    case 2: // Block Transfer
	      printf("SDU: BLOCK WRITE REQUESTED\n");
	      ld_die_rq=1;
	      break;

	    case 3: // Write High Half
	      break;

	    case 0:
	      // Full word write
	      break;
	    }
	  }else{
	    // BYTE WRITE
	  }
	  NUbus_acknowledge=1;
	  return;
	}
	break;
	  
	// SDU does not have a config prom
      case 0xFFF800 ... 0xFFF8FF: 
      case 0xFFFF64 ... 0xFFFFFF:
	if(NUbus_Request == VM_READ || NUbus_Request == VM_BYTE_READ){
	  NUbus_Data.word = 0;
	  NUbus_acknowledge=1;
	}
	return;

      default:
        printf("SDU: Unimplemented address 0x%X (0x%X)\n",
               NUbus_Address.Addr,NUbus_Address.raw);
	lambda_dump(DUMP_ALL);
	ld_die_rq=1;
      }
    }
    // Request finished or not ours
  }
}

// TEMPORARY: Lisp is about to start, dump its configuration
void dump_lisp_start_state(int I){
  uint32_t proc_conf_base = 0;
  system_configuration_qs *sys_conf = 0;
  processor_configuration_qs *proc_conf = 0;
  int x;
  extern unsigned char MEM_RAM[2][0x800000];
  // Obtain proc conf base from Q
  proc_conf_base = pS[I].Qregister;
  printf("LISP: PROC %d CONF BASE = 0x%X\n",I,proc_conf_base);
  // This is in nubus RAM, not SDU RAM!
  if((proc_conf_base&0xFF000000) == 0xF9000000){
    proc_conf = (processor_configuration_qs *)&MEM_RAM[0][proc_conf_base&0x7FFFFF];
  }
  if((proc_conf_base&0xFF000000) == 0xFA000000){
    proc_conf = (processor_configuration_qs *)&MEM_RAM[1][proc_conf_base&0x7FFFFF];
  }
  if((proc_conf_base&0xFF000000) == 0xFF000000){
    proc_conf = (processor_configuration_qs *)&SDU_RAM[proc_conf_base&0xFFFF];
  }
  if(proc_conf != 0){
    printf("LISP: SYS CONF BASE = 0x%X\n",proc_conf->sys_conf_ptr);
    if((proc_conf->sys_conf_ptr&0xFF000000) == 0xF9000000){
      sys_conf = (system_configuration_qs *)&MEM_RAM[0][proc_conf->sys_conf_ptr&0x7FFFFF];
    }
    if((proc_conf->sys_conf_ptr&0xFF000000) == 0xFA000000){
      sys_conf = (system_configuration_qs *)&MEM_RAM[1][proc_conf->sys_conf_ptr&0x7FFFFF];
    }
    if((proc_conf->sys_conf_ptr&0xFF000000) == 0xFF000000){
      sys_conf = (system_configuration_qs *)&SDU_RAM[proc_conf->sys_conf_ptr&0xFFFF];
    }
    // Print memory configuration
    x = 0;
    while(x < 10){
      if(proc_conf->memory_base[x] != 0){
	printf("LISP: PROC %d: MEMORY MAP %d: 0x%X - 0x%X (0x%X bytes)\n",
	       I,x,proc_conf->memory_base[x],((proc_conf->memory_base[x]+proc_conf->memory_bytes[x])-1),proc_conf->memory_bytes[x]);
      }
      x++;
    }
    // Print chaos share stuff
    x = 0;
    while(x < 5){
      if(proc_conf->chaos_share[x] != 0){
	printf("LISP: PROC %d: CHAOS SHARE %d @ 0x%X\n",
	       I,x,proc_conf->chaos_share[x]);
      }
      x++;
    }
    // Print other stuff
    if(sys_conf != 0){
      printf("LISP: SYSCONF: SHARE STRUCT POINTER 0x%X\n",sys_conf->share_struct_pointer);
      printf("LISP: SYSCONF: GLOBAL SHARED AREA 0x%X - 0x%X (0x%X bytes)\n",
	     sys_conf->global_shared_base,((sys_conf->global_shared_base+sys_conf->global_shared_size)-1),sys_conf->global_shared_size);
      printf("LISP: SYSCONF: SDU NUBUS 0x%X - 0x%X (0x%X bytes)\n",
	     sys_conf->sdu_nubus_base,((sys_conf->sdu_nubus_base+sys_conf->sdu_nubus_size)-1),sys_conf->sdu_nubus_size);
      printf("LISP: SYSCONF: CHAOS SHAREDEV BUFFER SIZE 0x%X\n",sys_conf->chaos_sharedev_buffer_size_in_bytes);      
    }
  }
}
