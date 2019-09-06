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

void sdu_init();
void sdu_clock_pulse();
void *sdu_thread(void *arg);

extern pthread_mutex_t multibus_master_mutex;

#ifdef HAVE_YAML_H
int yaml_sdu_mapping_loop(yaml_parser_t *parser);
#endif
void boot_lambda(int cp,int step);
void dump_lisp_start_state(int I);

// 8086 processor address
typedef union ri8086_Addr {
  uint32_t raw;
  uint16_t hw[2];
  uint8_t byte[4];
  struct {
    uint16_t Offset;
    uint16_t Segment;
  } __attribute__((packed));
} i8086_Addr;

// Multibus address
typedef union rmbAddr {
  uint32_t raw;
  uint8_t byte[4];
  struct {
    uint32_t Offset:10;
    uint32_t Page:10;
  } __attribute__((packed));
} mbAddr;

// Multibus to Nubus map entry
typedef union rMNAMap_Ent {
  uint32_t word;
  uint8_t byte[4];
  struct {
    uint32_t NUbus_Page:22;
    uint8_t Spare:1;
    uint8_t Enable:1;
  } __attribute__((packed));
} MNAMap_Ent;

// interfaces to things
uint8_t multibus_read(mbAddr addr);
uint16_t multibus_word_read(mbAddr addr);
void multibus_write(mbAddr addr,uint8_t data);
void multibus_word_write(mbAddr addr,uint16_t data);
void multibus_interrupt(int irq);
void clear_multibus_interrupt(int irq);
uint8_t i8088_port_read(uint32_t addr);
void i8088_port_write(uint32_t addr,uint8_t data);
uint16_t pic_chk();
void sducons_rx_int();

typedef struct tagsystem_configuration_qs {
  uint32_t version_number;
  uint32_t size;
  uint32_t number_of_processors;
  uint32_t processor_block_size;
  uint32_t share_struct_pointer;
  uint32_t debug_level;
  uint32_t lock;
  uint32_t ethernet_owner;
  uint32_t tapemaster_owner;
  uint32_t mti_8_line_owner;
  uint32_t mti_16_line_owner;
  uint32_t quarter_inch_tape_owner;
  uint32_t sdu_serial_a_owner;
  uint32_t sdu_serial_b_owner;
  uint32_t share_tty[3];
  uint32_t grey_owner;
  uint32_t grey_slot;
  uint32_t number_of_share_ttys;
  uint32_t number_of_share_tty_pages;
  uint32_t global_shared_base;
  uint32_t global_shared_size;
  uint32_t excelan_owner;
  uint32_t excelan_2_owner;
  uint32_t shared_excelan_pointer;
  uint32_t excelan_2_initted;
  uint32_t sdu_interrupt_map;
  uint32_t tapemaster_base_multibus_map;
  uint32_t tapemaster_multibus_map_size;
  uint32_t titn_owner;
  uint32_t system_console;
  uint32_t sdu_nubus_base;
  uint32_t sdu_nubus_size;
  uint32_t multibus_tapemaster_parameter_block;
  uint32_t excelan_base_multibus_map_block;
  uint32_t excelan_multibus_map_size;
  uint32_t cmos_clock_chip_owner;
  uint32_t user_base_multibus_map;
  uint32_t user_multibus_map_size;
  uint32_t second_grey_owner;
  uint32_t second_grey_slot;
  uint32_t default_grey_owner;
  uint32_t default_second_grey_owner;
  uint32_t flavors_bus_link_owner;
  uint32_t flavors_bus_link_slot;
  uint32_t second_flavors_bus_link_owner;
  uint32_t second_flavors_bus_link_slot;
  uint32_t newboot_version_number;
  uint32_t sdu_rom_version_number;
  uint32_t burr_brown_owner;
  uint32_t second_burr_brown_owner;
  uint32_t interphase_2181_owner;
  uint32_t disk_unit_0_initialized;
  uint32_t disk_unit_1_initialized;
  uint32_t disk_unit_2_initialized;
  uint32_t disk_unit_3_initialized;
  uint32_t disk_unit_4_initialized;
  uint32_t disk_unit_5_initialized;
  uint32_t disk_unit_6_initialized;
  uint32_t disk_unit_7_initialized;
  uint32_t nubus_disk_owner;
  uint32_t newboot_idle_count;
  uint32_t chaos_sharedev_buffer_size_in_bytes;
  uint32_t lmi_debug_board_owner;
  uint32_t lmi_debug_board_slot;
  uint32_t second_lmi_debug_board_owner;
  uint32_t second_lmi_debug_board_slot;
} __attribute__((packed)) system_configuration_qs;

typedef struct tagprocessor_configuration_qs {
  uint32_t sys_conf_ptr;
  uint32_t slot_number;
  uint32_t major_version;
  uint32_t minor_version;
  uint32_t starting_processor_switches;
  uint32_t share_runme;
  uint32_t share_slot;
  uint32_t share_type;
  uint32_t share_iopb;
  uint32_t share_interrupt_addr;
  uint32_t share_spare_1;
  uint32_t share_spare_2;
  uint32_t share_spare_3;
  uint32_t share_spare_4;
  uint32_t chaos_address;
  uint32_t send_chaos_share_dev;
  uint32_t rcv_chaos_share_dev;
  uint32_t memory_base[10];
  uint32_t memory_bytes[10];
  uint32_t vcmem_slot;
  uint32_t processor_type;
  uint32_t micro_band;
  uint32_t load_band;
  uint32_t paging_band;
  uint32_t file_band;
  uint32_t base_multibus_mapping_register;
  uint32_t boot_status;
  uint32_t chaos_share[5];
  uint32_t parity_enables;
  uint32_t vcmem_words_per_line;
  uint32_t number_of_multibus_maps;
  uint32_t boot_command;
  uint32_t boot_mode;
  uint32_t console;
  uint32_t console_baud_rate;
  uint32_t watchdog;
  uint32_t intmap_multibus_map;
  uint32_t n_aux_devs;
  uint32_t aux_dev[2];
  uint32_t excelan_multibus_map_base;
  uint32_t excelan_multibus_map_size;
} __attribute__((packed)) processor_configuration_qs;

#define MAX_SHARE_IOPB 8

// share structure
typedef struct tagshare_struct_qs {
  uint32_t lock;
  uint32_t max_iopbs;
  uint32_t current_iopb;
  // IOPB table here
  uint32_t valid_siopb[MAX_SHARE_IOPB];
  uint32_t siopb_ptr[MAX_SHARE_IOPB];
} __attribute__((packed)) share_struct_qs;
