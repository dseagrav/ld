/* Copyright 2016-2017
   Daniel Seagraves <dseagrav@lunar-tokyo.net>

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

// Register definitions

/*
  THE DP MODE REGISTER IS MOSTLY UNUSED, BUT SOMEDAY MAY CONTAIN A GREAT DEAL MORE
  DEBUGGING INFORMATION.  FOR NOW, IT ONLY HAS TWO SIGNALS, BUT IT CAN ALSO BE USED
  TO JUMPER INTERESTING SIGNALS TO SO THAT THEY CAN BE EXAMINED BY THE PROCESSOR WHILE
  IT'S RUNNING.  NOTE THAT ONLY THE BOTTOM 6 BITS ARE READ/WRITE - THE TOP TWO BITS
  ARE READ ONLY.

  bit 7 : the divisor sign bit ... 0 = positive , 1 = negitive
  bits 6-1 are unused
  bit 0 : the PDL.address.high bit ... 0 = M memory (only low 64 locations normally used)
                                       1 = PDL memory

*/
// With the PDL.address.high bit bootstrap says there is 2KW of PDL memory.
typedef union rDP_Mode_Reg {
  uint8_t raw;
  struct {
    uint8_t PDL_Addr_Hi:1;
    uint8_t Unused:6;
    uint8_t Divisor_Sign:1;
  } __attribute__((packed));
} DP_Mode_Reg;

/*
  THERE IS ALSO A 16 BIT RG MODE REGISTER WHICH HAS LOTS OF INTERESTING THINGS.
  IT CONTAINS MOST OF THE FUNCTIONALITY OF THE OLD CADR INTERRRUPT CONTROL REGISTER
  NOTE THAT ONLY THE TOP 12 BITS ARE READ/WRITE -- THE LOW 4 BITS ARE WRITE ONLY.
  ALSO NOTE THAT THESE 16 BITS ARE THE HIGH 16 BITS OF THE WORD {MFO.<31-16>}.

  AS OF VERSION 3 OF THE RG BOARD, THE RG MODE REGISTER IS NOW A FULL 32 BITS. IT
  RETAINS THE OLD DEFINITIONS AS BEFORE (EXCEPT THE AUX-STAT-COUNT-CONTROL BIT, WHICH
  IS NOW UNUSED), AND NOW HAS 4 MORE READ ONLY BITS (THE RG NUBUS SLOT ID) AND 12
  MORE READ/WRITE BITS.  THE WHOLE RG MODE REGISTER NOW LOOKS LIKE 12 READ/WRITE BITS,
  8 READ-ONLY BITS, AND THEN 12 MORE READ/WRITE BITS.
*/
typedef union rRG_Mode_Reg {
  uint32_t raw;
  struct {
    /*
      bits 2-0 determine what the aux stat-counter increments on.  There are 8 possibilities.
            7 = hi.c -- a permanently high signal, so that you can increment every
                        time ... this allows ratios between the two stat counters
            6 = microsecond.clock -- a microsecond clock for timing purposes
            5 = t.statistics.bit -- a bit in the timing ram
            4 = t.hold -- to determine how long we are sitting in "hang" states
                          (note that you probably want to clock on sm.ticks)
            3 = increment.LC -- goes high during fetches of the next macro-instruction
            2 = csm.statistics.bit -- a bit in the cache state machine
                                      (usually signifies cache hits)
            1 = memory.start.next.cycle -- goes high during memory fetches
            0 = valid.statistics.bit -- a bit in the current microinstruction
      bit 2. : aux.stat.count.control 2 -- see above
      bit 1. : aux.stat.count.control 1 -- see above
      bit 0. : aux.stat.count.control 0 -- see above
    */
    uint8_t Aux_Stat_Count_Control:3;
    // Bit 3 is unused?
    uint8_t Bit3:1;
    /*
      bit 4 selects clock for aux stat counter  (this was mod 31 to rev 2 ww boards.
      the mod may not have been made to most rev 2 boards).
    */
    uint8_t Aux_Stat_Count_Clock:1;
    //   bits 11-5 are currently unused
    uint16_t Unused:7;
    /*
      bits 15-12 : rg.nubus.id.<3-0> -- the nubus slot id that the RG board is currently in.
      bit 15 : rg.nubus.id.3 -- bit 3 of the rg nubus slot id
      bit 14 : rg.nubus.id.2 -- bit 2 of the rg nubus slot id
      bit 13 : rg.nubus.id.1 -- bit 1 of the rg nubus slot id
      bit 12 : rg.nubus.id.0 -- bit 0 of the rg nubus slot id
    */
    uint8_t NUbus_ID:4;
    /*
      bit 16 : need.macro.inst.fetch L -- asserted when you need another pair of
      macro-instructions (i.e., you don't need to
      memory fetch each one, you first check to see if
      you still haven't used the other one; if so, assert
      this bit and you will fetch from memory two more.
    */
    uint8_t Need_Macro_Inst_Fetch:1;
    // bits 17-19 are unused read-only bits
    uint8_t Unused_RO:3;
    /*
      bits 22-20 determine what the main stat-counter increments on.  There are 8 possibilities.
            7 = hi.c -- a permanently high signal, so that you can increment every
                        time ... this allows ratios between the two stat counters
            6 = unused -- can be jumpered to anything needed to examine
            5 = t.statistics.bit -- a bit in the timing ram
            4 = t.hold -- to determine how long we are sitting in "hang" states
                          (note that you probably want to clock on sm.ticks)
            3 = increment.LC -- goes high during fetches of the next macro-instruction
            2 = csm.statistics.bit -- a bit in the cache state machine
                                      (usually signifies cache hits)
            1 = memory.start.next.cycle -- goes high during memory fetches
            0 = valid.statistics.bit -- a bit in the current microinstruction
      bit 22. : main.stat.count.control 2 -- see above
      bit 21. : main.stat.count.control 1 -- see above
      bit 20. : main.stat.count.control 0 -- see above
    */
    uint8_t Main_Stat_Count_Control:3;
    /*
      bit 23 : aux.stat.count.control -- determines whether we increment the aux stat-counter
      based on the statistic bit in the current microinstruction, or the signal
      memory.start.next.cycle, which comes on during memory fetches....
                                            0 = valid.statistics.bit causes increment
                                            1 = memory.start.next.cycle causes increment
      Also see AUX-STAT-CLOCK-CONTROL.
    */
    uint8_t Aux_Stat_Count_Control_2:1;
    /*
      bit 24 : main.stat.clock.control -- determines whether we clock the main stat counter on
      uinst.clocks (standard) or sm.clocks; 0 = sm.clocking, 1 = uinst.clocking
    */
    uint8_t Main_Stat_Clock_Control:1;
    /*
      bit 25 : enable.misc.MID
      when HI let the macro instruction decode stuff dispatch on the whole MISC field
      of the instruction, if it is a MISC instruction to destination ignore (0).
      MID memory 6000 - 7777 is used to hold dispatch addresses for MISC (6000-6777)
      and MISC1 (7000 - 7777).
      On source cycles, if the 40 bit in M.source is set, the above areas are ref'ed on
      a misc instruction regardless of its destination.
    */
    uint8_t Enable_Misc_MID:1;
    /*
      bit 26 : sequence.break L -- this bit is checked by the microcode for the instruction
      jump-on-sequence-break.  0 = set the sequence break, 1 = disable sequence breaks
    */
    uint8_t Sequence_Break:1;
    /*
        bit 27 : interrupt.enable -- enables program interrupts; 0 = no interrupts,
                                                                 1 = normal program interrupts
    */
    uint8_t Interrupt_Enable:1;
    /*
      bits 28-29: mid.hi.adr.0 and mid.hi.adr.1
      high two bits of MID Ram address when others are comming from the high 10 bits of
      the macro.ir.   These allow convenient addressing of the portion of the macro.ir
      used for MISC decodeing, etc.  Should be 0 for normal operation.
    */
    uint8_t MID_Hi_Adr:2;
    // bit 30 : 25.bit.virtual.address.mode L; 0 = 25 bit addresses, 1 = 24 bit addresses
    uint8_t VA_Mode:1;
    // bit 31 : single.step.macro.inst.mode; 0 = normal execution, 1 = macrocode single stepping
    uint8_t Single_Step_Macro_Inst:1;
  } __attribute__((packed));
} RG_Mode_Reg;

/* Configuration Register */
typedef union rConfiguration_Reg {
  uint32_t word;
  uint8_t byte[4];
  uint8_t writable_bits:4;
  struct {
    uint8_t Init:1;
    uint8_t Enable_NU_Master:1;
    uint8_t LED:1;
    uint8_t Enable_SM_Clock:1;
    // The next 4 bits are read-only, and are inverted from the signal on the board.
    uint8_t nop:1;
    uint8_t nop_next:1;
    uint8_t t_hold_l:1;
    uint8_t uinst_clock_l:1;
    // The remaining 24 bits are read-only, but are not inverted.
    uint8_t stat_halt_l:1;
    uint8_t halt_request:1;
    uint8_t any_parity_error:1;
    uint8_t hbus_present_l:1;
    uint8_t debug_clock_mode_synced:1;
    // This is actually what stops SM.clock.
    // When a parity error is detected, one more sm.clock happens before machine really stops completely.
    // (t.hold is asserted during this clock, however)
    // If the parity error goes away (during?) this last sm.clock, the machine will stop
    // with the any-parity-error-synced bit set, but any-parity-error no longer set.
    uint8_t any_parity_error_synced_l:1;
    // Remaining bits unused
  } __attribute__((packed));
} Configuration_Reg;


/* This is the Processor Mode Register, accessed by the SPY interface */
typedef union rProcessor_Mode_Reg {
  uint32_t raw;
  uint8_t byte[4];
  struct {
    uint8_t Stat_Halt_Enable:1;
    uint8_t Single_Step_Mode:1; // NOT USED IN V3.0 AND LATER
    uint8_t Fast_Clock_Enable:1; // Inverted
    uint8_t Reset_Interrupt_Counter:1;
    uint8_t Debug_Clock:1;      // Drives SM clock when CONREG Enable_SM_Clock is clear.
    uint8_t Advance_UInst_Request:1;
    uint8_t Debug_Cache_Permit:1;
    uint8_t Parity_Stop_Enable:1;
    uint8_t Enable_MI_Parity:1;
    uint8_t Enable_CM_Parity:1;
    uint8_t Enable_DP_Parity:1;
    uint8_t Enable_MID_Parity:1;
    uint8_t Enable_TREG_Parity:1;
    uint8_t Spare0:1;
    uint8_t Clear_NOP:1;
    uint8_t Spare1:2;
    uint8_t Spy_Address_TRAM_L:1;
    uint8_t Force_T_Hold:1;
    uint8_t Force_MI_Reset_L:1;
    uint8_t Force_CSM_Use_Spy_Addr_L:1;
    uint8_t Allow_UInst_Clocks:1;
    uint8_t Spare2:2;
  } __attribute__((packed));
} Processor_Mode_Reg;

// This is a TRAM word
typedef union rTRAM_Word {
  uint32_t word;
  uint8_t byte[4];
  struct {
    uint8_t state;
    uint8_t next_select:2;
    uint8_t hold_control:2; // Unused
    uint8_t M_address_control:2;
    uint8_t M_CS:1;
    uint8_t M_WE_L:1;
    uint8_t A_address_control:2;
    uint8_t A_WE_L:1;
    uint8_t A_clock_next:1;
    uint8_t L_to_A_L:1;
    uint8_t L_to_M_L:1;
    uint8_t source_cycle:1; // Also first.source.cycle.next and first.source.cycle.next-l
    uint8_t data_paths_to_MFO:1;
    uint8_t M_clock_next:1;
    uint8_t slow_dest_write:1;
    uint8_t Spare:1;
    uint8_t new_uinst:1;
    uint8_t next_cycle_number:2; // Shared with parity
    uint8_t parity:2;
  } __attribute__((packed));
} TRAM_Word;

// CSM.ADR register
typedef union rCSMAdr_Word {
  uint32_t raw;
  struct {
    uint16_t Addr:12;
    uint32_t Cached_Phy_Addr:18;
    uint8_t Memory_Cycle_Active:1;
    uint8_t Memory_Cycle_Pending:1;
  } __attribute__((packed));
} CSMAdr_Word;

// Words in CSMRAM
typedef union rCSM_Word {
  uint32_t raw;
  uint8_t byte[4];
  struct {
    uint16_t State:11;
    uint8_t Condition:3;
    uint8_t Cache_Mode:2; // 0 = nothing, 1 = read, 2 = write if NU.xfer, 3 = write if cache hit
    uint8_t MD_to_NUDATA_bus:1;
    uint8_t RD_DATA_to_NUDATA_bus:1;
    uint8_t Cache_Adr_from_NUADR:1;
    uint8_t Cache_Hit_Enable:1;
    uint8_t Memory_Cycle_Ending_L:1; // Defaulted HI if not spec'd
    uint8_t Count_NUADR:1;
    uint8_t Virtual_Page_Group:1;
    uint8_t NU_Idle:1;
    uint8_t Statistics_Bit:1;
    uint8_t Lambda_STREQ:1;
    uint8_t Lambda_BREQ:1;
    uint8_t Request_Noted_L:1; // Defaulted HI if not spec'd
    uint8_t Parity:4;
  } __attribute__((packed));
} CSM_Word;

/* Shadow Memory Page Map Entry */
typedef union ShadowMemoryPageEnt {
  uint8_t raw;
  struct {
    uint8_t Resident:1; // Resident in physical memory
    uint8_t Written:1;  // Written to (other than paging in)
    uint8_t Paged:1;    // Written to by being paged out
    // Resident == 0 with Written == 0 and Paged == 0 means unused page
  } __attribute__((packed));
} ShadowMemoryPageEnt;

/* Processor State Structure */
struct lambdaState {
  /* Buses */
  uint32_t Abus; // A-bus
  uint32_t Mbus; // M-bus
  uint32_t Obus; // O-bus
  uint32_t MFObus; // MFO-bus
  uint32_t Rbus; // R-bus
  /* Halt control */
  int cpu_die_rq;
  // Memories
  UInst WCS[64*1024];          // Writable Control Store (16KW on Raven)
  uint32_t Amemory[1024*4];    // A-memory (1KW on Raven)
  uint32_t Mmemory[1024*4];    // M-memory and PDLmemory (see DP_Mode)
  uint32_t MIDmemory[1024*4];  // Macro-Instruction-Dispatch-memory
  uint32_t Cache_Sector_Addr[256]; // Base address of cache sector
  uint8_t  Cache_Sector_Age[256];  // Order of cache sector allocation
  volatile nuData   Cache_Data[256][4];    // Cache data
  volatile uint8_t  Cache_Status[256][4];  // Status bits for cache entry

  MicroAddr loc_ctr_reg;        // (Micro)Location Counter Register -- Address of NEXT instruction
  uint32_t loc_ctr_cnt;         // (Micro)Location Counter Register -- Address of CURRENT instruction
  int32_t  loc_ctr_nxt;         // (Micro)Location Counter Register -- Address of AFTER-NEXT instruction
  int32_t  popj_after_nxt;      // POPJ-After-Next trigger
  int32_t macro_dispatch_inst;  // Macro instruction dispatch trigger
  int last_loc_ctr;             // Lpc
  lv1_ent  vm_lv1_map[4096];    // VM Map, level 1
  lv2_ctl_ent vm_lv2_ctl[4096]; // VM Map, level 2, Control Storage
  lv2_adr_ent vm_lv2_adr[4096]; // VM Map, level 2, Address Storage
  lv2_idx  vm_lv2_index;        // VM Map, level 1 to level 2 index
  int      vm_byte_mode;        // Gets ORed with access type to implement byte mode
  int      cached_gcv;          // Cached GC Volatility
  PhysAddr vm_phys_addr;        // VM Map, Generated Physical Address
  int Page_Fault;               // VM Map Failure Flag
  uint16_t CRAM_map[4096];      // CRAM map.
  MIDAddress MIDAddr;           // Macro Instruction Dispatch Address
  TRAM_Word TRAM[0x1000];       // Timing RAM
  TRAM_Word TREG;               // Timing RAM output register
  uint32_t CSM_Output;          // Cache State Machine output register
  CSM_Word CSMRAM[0x1000];      // Cache State Machine RAM

  uint8_t InterruptPending;     // Interrupt Pending Counter
  uint8_t InterruptStatus[0x100]; // Interrupt states from the RG board
  uint8_t InterruptVector;        // Which one is active?

  /* Mode registers */
  DP_Mode_Reg DP_Mode;
  RG_Mode_Reg RG_Mode;
  Processor_Mode_Reg PMR;
  Configuration_Reg ConReg;

  /* MF-bus sources */
  uint32_t pdl_ptr_reg;        // PDL Pointer Register
  uint32_t pdl_index_reg;      // PDL Index Register
  uint32_t uPCS_ptr_reg;       // Microcode counter stack pointer
  uint32_t disp_constant_reg;  // Dispatch constant register
  uint32_t stat_counter_main;  // Main stat counter
  uint32_t stat_counter_aux;   // Aux stat counter

  /* SPY items */
  CSMAdr_Word CSM_Adr;          // For hacking (CSMRAM?), also carries memory info on read
  uint16_t TRAM_Adr;            // Timing RAM address register, 12 bits
  uint32_t SPY_Register;        // Bond, James Bond
  uint32_t Parity_Vector;       // Parity Vector
  uint16_t History_Pointer;     // HPTR, 12 bits (docs say 10, diags test for 12)
  uint16_t History_RAM[0x1000]; // HRAM, 16 bits

  /* Local Bus Interface */
  int      NUbus_Master;        // Are we the nubus master?
  volatile int LCbus_error;
  volatile int LCbus_Busy;
  volatile int LCbus_acknowledge;
  volatile nuAddr LCbus_Address;
  volatile nuData LCbus_Data;
  volatile nuData LCbus_Block[4];     // Block Transfer Hack
  volatile int LCbus_Request;

  /* Others */
  Q        MIregister;                // Macroinstruction Register
  uint8_t  mirInvalid;                // MIR validity
  Q        MDregister;                // Memory Data register
  Q        VMAregister;               // Virtual Memory Address
  Q        LCregister;                // Macro Location Counter register
  UInst    Iregister;                 // 56b Instruction register
  uint32_t Qregister;                 // Q register
  uint32_t uPCS_stack[255];           // Microcode counter stack memory
  int64_t  ALU_Result;                // Result of ALU operation
  bool     ALU_Carry_Out;             // Carry-Out
  bool     ALU_Fixnum_Oflow;          // FIXNUM Overflow
  bool     imod_en;                   // Enable IMOD
  bool     spy_wrote_ireg;            // Spy wrote the IR, so don't fetch over it.
  DispatchWord disp_word;             // Dispatch operation word
  uint32_t imod_lo,imod_hi;           // IMOD storage
  uint32_t Multiplier_Input;          // Multiplier input
  uint32_t Multiplier_Output;         // Multiplier output
  uint32_t Multiplier_FT;             // Multiplier flow-through
  uint16_t TRAM_PC;                   // Timing RAM "PC" register, 12 bits
  uint64_t Obus_Input;                // Obus Multiplexer Input

  bool NOP_Next;                    // Next microinstruction gets NOPed out
  bool cram_write_cyc;              // CRAM write cycle (or map)

  int SM_Clock;                     // State Machine Clock

  // Software flags (Nonexistent on real hardware)
  uint8_t NUbus_ID;                 // Bus slot for this processor (RG board slot?)
  bool test_true;                   // Condition Test True
  bool slow_dest;                   // Slow destination flag
  bool long_inst;                   // long instruction flag
  bool exec_hold;                   // Execution hold flag, waiting for bus to free
  bool uI_Clock_Pulse;              // Microinstruction clock pulsed
  bool wrote_uPC;                   // Did we write the uPC?
  bool microtrace;                  // Microcode tracing enable
  bool macrotrace;                  // Macrocode tracing enable
  uint16_t Next_History_Pointer;    // Next HPTR, 12 bits
  uint8_t Cache_Permit;             // Allow cache hit
  uint8_t Packetize_Writes;         // Write-through cache in block transfers
  uint8_t Packet_Code;              // Size of block transfer used
  int Cache_Resolved;               // Did we look this up in cache?
  int Cache_Sector_Hit;             // Did we find it in a sector?
  int Cache_Sector;                 // What sector?
  int Cache_Oldest_Sector;          // Which sector is oldest? (For eviction purposes)
  pthread_mutex_t cache_wc_mutex;   // Used to prevent nubus write check from updating evicted sectors
  volatile int SM_Clock_Pulse;      // Used to generate SM clock pulses
  volatile Processor_Mode_Reg *SM_Old_PMR; // Old PMR value used when making SM clock pulses

  // Performance monitoring
  volatile unsigned long cycle_count;
  volatile unsigned long stall_count;
  volatile int64_t delta_time;
};

/* Cache write-check mutexes */
extern pthread_mutex_t cache_wc_mutex[];

/* Functions */
void lambda_initialize(int I,int ID);
void lambda_nubus_pulse(int I);
void lambda_clockpulse(int I);
void *lam_thread(void *arg);
void cache_write_check(int access, int I, uint32_t address, uint32_t data);
void shadow_write(uint32_t addr,Q data);
Q shadow_read(uint32_t addr);
