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

#include "config.h"

#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

// tun/tap ethernet interface
#include <errno.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#ifdef HAVE_LINUX_IF_H
#include <linux/if.h>
#endif
#ifdef HAVE_LINUX_IF_TUN_H
#include <linux/if_tun.h>
#endif

// bpf ethernet interface
#if !defined (HAVE_LINUX_IF_H) && defined (HAVE_NET_BPF_H)
#include <net/if.h>
#include <net/bpf.h>
#endif

// YAML configuration
#ifdef HAVE_YAML_H
#include <yaml.h>
#endif

#include "ld.h"
#include "nubus.h"
#include "sdu.h"

/* 3COM 3C400 Multibus Ethernet */
/* Note that Lambda requires the byte-ordering switch to be ON!
   This means everything in the manuals will be byte-swapped! */

typedef union rETH_MECSR_MEBACK_Reg {
  uint32_t word;
  uint8_t byte[4];
  struct {
    uint8_t Reset:1;
    uint8_t Spare:1;
    uint8_t RBBA:1;
    uint8_t AMSW:1;
    uint8_t JAM:1;
    uint8_t TBSW:1;
    uint8_t ABSW:1;
    uint8_t BBSW:1;
    uint8_t PA:4;
    uint8_t JINTEN:1; // Jam Int Enable
    uint8_t TINTEN:1; // TX Int Enable
    uint8_t AINTEN:1; // RX A Int Enable
    uint8_t BINTEN:1; // RX B Int Enable
    uint16_t Spare2;
  } __attribute__((packed));
} ETH_MECSR_MEBACK_Reg;

typedef union rETH_HW_ADDR_Reg {
  uint32_t word[2];
  uint8_t byte[8];
} ETH_HW_ADDR_Reg;

ETH_MECSR_MEBACK_Reg ETH_MECSR_MEBACK;
ETH_HW_ADDR_Reg ETH_HW_ADDR;

uint8_t ETH_Addr_RAM[8];
uint8_t ETH_TX_Buffer[0x800];
uint8_t ETH_RX_Buffer[2][0x800];
uint32_t eth_cycle_count;
int enet_trace=0;
extern int ld_die_rq;

// Linux tuntap interface
char ether_iface[30] = "ldtap";
unsigned char ether_addr[6] = {0x00,0x02,0x9C,0x55,0x89,0xC6};
int pkt_count = 0;
int ether_fd = -1;
uint8_t ether_rx_buf[0x800];

#if defined (HAVE_LINUX_IF_H) && defined (HAVE_LINUX_IF_TUN_H)
#define USES_ETHER_CODE "Linux tuntap"

int ether_init(){
  struct ifreq ifr;
  int fd, err, flags;
  char *tundev = "/dev/net/tun";

  // Open device
  fd = open(tundev, O_RDWR);
  if(fd < 0){
    return(fd);
  }

  // Clobber ifr
  memset(&ifr, 0, sizeof(ifr));

  // Set flags for TAP device
  ifr.ifr_flags = IFF_TAP;
  strncpy(ifr.ifr_name, ether_iface, IFNAMSIZ);

  // Create queue
  err = ioctl(fd, TUNSETIFF,(void *)&ifr);
  if(err < 0){
    close(fd);
    return(err);
  }

  // Become nonblocking
  flags = fcntl(fd,F_GETFL,0);
  if(flags < 0){ flags = 0; }
  fcntl(fd,F_SETFL,flags|O_NONBLOCK);

  // All done!
  return(fd);
}

void ether_tx_pkt(uint8_t *data,uint32_t len){
  ssize_t res = 0;
  if(ether_fd < 0){ return; }
  // printf("Ether: Sending %d bytes\n",len+4);
  res = write(ether_fd,data-4,len+4);
  if(res < 0){
    perror("ether:write()");
  }
}

uint32_t ether_rx_pkt(){
  ssize_t res = 0;
  if(ether_fd < 0){ return(0); }
  res = read(ether_fd,ether_rx_buf,(0x800-2));
  if(res < 0){
    if(errno != EAGAIN && errno != EWOULDBLOCK){
      perror("ether:read()");
    }
    return(0);
  }
  // printf("Done! Got %d bytes\n",(int)res);
  return(res);
}
#endif /* Linux ethertap code */

// Berkeley Packet Filter code
#if !defined (USES_ETHER_CODE) && defined (HAVE_NET_BPF_H)
#define USES_ETHER_CODE "BPF"

char ether_bpfn[64];

int ether_init(){
  struct ifreq ifr;
  int fd, err, flags;
  int x=0;
  uint32_t y;

  while(x < 16){
    sprintf(ether_bpfn,"/dev/bpf%d",x);
    // Open device
    fd = open(ether_bpfn, O_RDWR);
    if(fd < 0){
      if(errno == EBUSY){ x++; continue; }
      if(errno == ENOENT){
	printf("BPF: Unable to find available BPF device (tried %d)\n",x);
	return(0);
      }
      // Some other error happened, bail
      return(-1);
    }else{
      // Done!
      break;
    }
    x++;
  }

  // Set max packet length
  y = 0x800;
  err = ioctl(fd, BIOCSBLEN,(void *)&y);
  if(err < 0){
    close(fd);
    return(err);
  }

  // Clobber ifr
  memset(&ifr, 0, sizeof(ifr));

  // Set header-complete mode to one, since we will be providing complete packets
  y = 1;
  err = ioctl(fd, BIOCSHDRCMPLT,(void *)&y);
  if(err < 0){
    close(fd);
    return(err);
  }

  // Don't echo my own packets back to me
  y = 0;
  err = ioctl(fd, BIOCSSEESENT,(void *)&y);
  if(err < 0){
    close(fd);
    return(err);
  }

  // Operate in Immediate Mode
  y = 1;
  err = ioctl(fd, BIOCIMMEDIATE,(void *)&y);
  if(err < 0){
    close(fd);
    return(err);
  }

  // Attach to interface
  strncpy(ifr.ifr_name,ether_iface,IFNAMSIZ);
  err = ioctl(fd, BIOCSETIF,(void *)&ifr);
  if(err < 0){
    close(fd);
    return(err);
  }

  // Operate in Promiscuous Mode
  y = 1;
  err = ioctl(fd, BIOCPROMISC,(void *)&y);
  if(err < 0){
    close(fd);
    return(err);
  }

  // Become nonblocking
  flags = fcntl(fd,F_GETFL,0);
  if(flags < 0){ flags = 0; }
  fcntl(fd,F_SETFL,flags|O_NONBLOCK);

  printf("BPF: Operating\n");
  return(fd);
}

void ether_tx_pkt(uint8_t *data,uint32_t len){
  ssize_t res = 0;
  if(ether_fd < 0){ return; }
  // printf("Ether: Sending %d bytes\n",len);
  res = write(ether_fd,data,len);
  if(res < 0){
    perror("ether:write()");
  }  
  return;
}

unsigned int bpf_buf_offset = 0;
unsigned int bpf_buf_length = 0;
uint8_t ether_bpf_buf[0x800];

uint32_t ether_rx_pkt(){
  ssize_t res = 0;
  struct bpf_hdr *bpf_header;

  if(ether_fd < 0){ return(0); }
  if(bpf_buf_offset == 0){
    // Get more packets
    res = read(ether_fd,ether_bpf_buf,0x800);
    if(res < 0){
      if(errno != EAGAIN && errno != EWOULDBLOCK){
	perror("ether:read()");
      }
      return(0);
    }
    // printf("Ether: read got %d bytes\n",(int)res);
    bpf_buf_length = res;
  }
  // There is a BPF header that must be dealt with
  // printf("BPF: Header @ %d\n",bpf_buf_offset);
  bpf_header = (struct bpf_hdr *)(ether_bpf_buf+bpf_buf_offset);
  // Extract packet into ether_rx_buf+4 (to fake the 4-byte header that linux prepends)
  // printf("BPF: Actual packet %d bytes @ offset %d\n",bpf_header->bh_caplen,bpf_header->bh_hdrlen);
  memcpy(ether_rx_buf+4,(uint8_t *)(ether_bpf_buf+bpf_buf_offset+bpf_header->bh_hdrlen),bpf_header->bh_caplen);
  /* printf("PKT: DST %.2X:%.2X:%.2X:%.2X:%.2X:%.2X SRC %.2X:%.2X:%.2X:%.2X:%.2X:%.2X PTYPE %.2X %.2X\n",
	 ether_rx_buf[4],ether_rx_buf[5],ether_rx_buf[6],ether_rx_buf[7],ether_rx_buf[8],ether_rx_buf[9],
	 ether_rx_buf[10],ether_rx_buf[11],ether_rx_buf[12],ether_rx_buf[13],ether_rx_buf[14],ether_rx_buf[15],
	 ether_rx_buf[16],ether_rx_buf[17]); */
  // Do we have more packets?
  if((bpf_buf_offset+bpf_header->bh_hdrlen+bpf_header->bh_caplen) < bpf_buf_length){
    /* printf("BPF: At %d, want %d, continuing buffer processing\n",
       (bpf_buf_offset+bpf_header->bh_hdrlen+bpf_header->bh_caplen),bpf_buf_length); */
    bpf_buf_offset += BPF_WORDALIGN(bpf_header->bh_hdrlen+bpf_header->bh_caplen);
  }else{
    // printf("BPF: All done!\n");
    bpf_buf_offset = 0;
  }
  if(bpf_header->bh_caplen != bpf_header->bh_datalen){
    /* printf("BPF: LENGTH MISMATCH: Captured %d of %d\n",
       bpf_header->bh_caplen,bpf_header->bh_datalen); */
    return(0); // Throw away packet
  }
  return(bpf_header->bh_caplen+4);
}
#endif /* BPF code */

// If we did not include any other ethernet code...
#ifndef USES_ETHER_CODE
#define USES_ETHER_CODE "null"

int ether_init(){
  return(0);
}

void ether_tx_pkt(uint8_t *data __attribute__ ((unused)),uint32_t len __attribute__ ((unused))){
  return;
}

uint32_t ether_rx_pkt(){
  return(0);
}
#endif /* Stub code */

// Device
void enet_reset(){
  if(ether_fd < 0){
    // tuntap initialization
    ether_fd = ether_init();
    if(ether_fd < 1){
      if(ether_fd < 0){
	perror("ether_init()");
      }
      ether_fd = -1;
    }
  }
  ETH_HW_ADDR.byte[0] = ether_addr[0];
  ETH_HW_ADDR.byte[1] = ether_addr[1];
  ETH_HW_ADDR.byte[2] = ether_addr[2];
  ETH_HW_ADDR.byte[3] = ether_addr[3];
  ETH_HW_ADDR.byte[4] = ether_addr[4];
  ETH_HW_ADDR.byte[5] = ether_addr[5];
  ETH_HW_ADDR.byte[6] = 0x00;
  ETH_HW_ADDR.byte[7] = 0x00;
  // Clobber state
  eth_cycle_count = 0;
  ETH_MECSR_MEBACK.word = 0; // Clobber all
  ETH_MECSR_MEBACK.TBSW = 0; // TB belongs to host
  ETH_MECSR_MEBACK.JAM = 0; // No collision
  ETH_MECSR_MEBACK.AMSW = 0; // Address Memory belongs to host
  ETH_MECSR_MEBACK.ABSW = 0; // 1; // A buffer is mine
  ETH_MECSR_MEBACK.BBSW = 0; // 1; // B buffer is mine
  ETH_MECSR_MEBACK.RBBA = 0; // A buffer has first packet
}

uint8_t enet_read(uint16_t addr){
  uint16_t subaddr = 0;
  if(enet_trace){
    printf("3COM: enet_read addr 0x%X\n",addr);
  }
  switch(addr){
  case 0x0000 ... 0x03FF: // MEBACK
    return(ETH_MECSR_MEBACK.byte[addr&0x03]);
    break;
  case 0x0400 ... 0x05FF: // Address ROM
    subaddr = ((addr-0x0400)&0x07);
    return(ETH_HW_ADDR.byte[subaddr]);
    break;
  case 0x0600 ... 0x07FF: // Station Address RAM
    subaddr = ((addr-0x0600)&0x07);
    return(ETH_Addr_RAM[subaddr]);
    break;
  case 0x0800 ... 0x0FFF: // TX Packet Buffer
    subaddr = addr-0x0800;
    return(ETH_TX_Buffer[subaddr]);
    break;
  case 0x1000 ... 0x17FF: // RX Buffer A
    subaddr = addr-0x1000;
    return(ETH_RX_Buffer[0][subaddr]);
    break;
  case 0x1800 ... 0x1FFF: // RX Buffer B 
    subaddr = addr-0x1800;
    return(ETH_RX_Buffer[1][subaddr]);
    break;
  default:
    printf("3COM: UNKNOWN READ ADDR 0x%X\n",addr);
    ld_die_rq = 1;
  }
  return(0xFF);
}

void enet_write(uint16_t addr,uint8_t data){
  uint16_t subaddr = 0;
  switch(addr){
  case 0x0000 ... 0x03FF: // MEBACK
    {
      ETH_MECSR_MEBACK_Reg ETH_MECSR_MEBACK_Wt;
      ETH_MECSR_MEBACK_Wt.word = ETH_MECSR_MEBACK.word;
      ETH_MECSR_MEBACK_Wt.byte[addr&0x03] = data;
      // Process bits
      if(ETH_MECSR_MEBACK_Wt.Reset == 1){
	printf("3COM: RESET\n");
	enet_reset();
	return;
      }
      if(ETH_MECSR_MEBACK_Wt.AMSW == 1 && ETH_MECSR_MEBACK.AMSW == 0){
	printf("3COM: AMSW given to interface: Our address is %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
	       ETH_Addr_RAM[0],ETH_Addr_RAM[1],ETH_Addr_RAM[2],ETH_Addr_RAM[3],ETH_Addr_RAM[4],ETH_Addr_RAM[5]);
	ETH_MECSR_MEBACK.AMSW = 1;
      }
      if(ETH_MECSR_MEBACK_Wt.TBSW == 1 && ETH_MECSR_MEBACK.TBSW == 0){
	if(enet_trace){
	  printf("3COM: TBSW given to interface: Packet offset ");
	}
	uint32_t pktoff = (ETH_TX_Buffer[0x00]&0x07);
	uint32_t pktlen = 0x800;
	pktoff <<= 8;
	pktoff |= ETH_TX_Buffer[0x01];
	pktlen -= pktoff;
	if(enet_trace){
	  printf("%d, length %d\n",pktoff,pktlen);
	}
	// Must include our address
	ETH_TX_Buffer[pktoff+6] = ETH_Addr_RAM[0];
	ETH_TX_Buffer[pktoff+7] = ETH_Addr_RAM[1];
	ETH_TX_Buffer[pktoff+8] = ETH_Addr_RAM[2];
	ETH_TX_Buffer[pktoff+9] = ETH_Addr_RAM[3];
	ETH_TX_Buffer[pktoff+10] = ETH_Addr_RAM[4];
	ETH_TX_Buffer[pktoff+11] = ETH_Addr_RAM[5];
	// FCS will be appended by the host (one way or the other)
	// All ready! Send it
	ether_tx_pkt(ETH_TX_Buffer+pktoff,pktlen);
	ETH_MECSR_MEBACK.TBSW = 0;
	if(ETH_MECSR_MEBACK.TINTEN != 0){
	  multibus_interrupt(0);
	}
      }
      ETH_MECSR_MEBACK.PA = ETH_MECSR_MEBACK_Wt.PA;
      // Clobber state
      ETH_MECSR_MEBACK.TBSW = 0; // TB belongs to host
      ETH_MECSR_MEBACK.JAM = 0; // No collision
      ETH_MECSR_MEBACK.ABSW = 1; // A buffer is mine
      ETH_MECSR_MEBACK.BBSW = 1; // B buffer is mine
      ETH_MECSR_MEBACK.RBBA = 0; // A buffer has first packet
    }
    break;
  case 0x0400 ... 0x0403: // Address ROM
    printf("3COM: WRITING ADDRESS ROM?\n");
    break;
  case 0x0600 ... 0x0607: // Station Address RAM
    if(ETH_MECSR_MEBACK.AMSW != 0){
      // We own this, disregard
      return;
    }
    subaddr = addr-0x0600;
    ETH_Addr_RAM[subaddr] = data;
    break;
  case 0x0800 ... 0x0FFF: // TX Packet Buffer
    if(ETH_MECSR_MEBACK.TBSW != 0){
      // We own this, disregard
      return;
    }
    subaddr = addr-0x0800;
    ETH_TX_Buffer[subaddr] = data;
    break;
  case 0x1000 ... 0x17FF: // RX Buffer A
    if(ETH_MECSR_MEBACK.ABSW != 0){
      // We own this, disregard
      return;
    }
    subaddr = addr-0x1000;
    ETH_RX_Buffer[0][subaddr] = data;
    break;
  case 0x1800 ... 0x1FFF: // RX Buffer B
    if(ETH_MECSR_MEBACK.BBSW != 0){
      // We own this, disregard
      return;
    }
    subaddr = addr-0x1800;
    ETH_RX_Buffer[1][subaddr] = data;
    break;
  default:
    printf("3COM: UNKNOWN WRITE ADDR 0x%X\n",addr);
    ld_die_rq = 1;
  }
}

void enet_clock_pulse(){
  // Ethernet controller maintenance
  eth_cycle_count++;
  if(eth_cycle_count > (5000000/60)){
    int32_t pktlen = 0;
    int32_t drop = 0;
    eth_cycle_count = 0;
    // Ethernet ready to take a packet?
    if(ETH_MECSR_MEBACK.AMSW == 1 && (ETH_MECSR_MEBACK.ABSW == 1 || ETH_MECSR_MEBACK.BBSW == 1)){ 
      // Yes
      pktlen = ether_rx_pkt();      
      if(pktlen > 0){
        // We can has packet!
        pktlen -= 4;
        // uint16_t hdr = ((pktlen+2)<<1);
        uint16_t hdr = (pktlen+2);
        if(hdr&0x01){ hdr++; }
        if(!(ether_rx_buf[4] == ETH_Addr_RAM[0] &&
             ether_rx_buf[5] == ETH_Addr_RAM[1] &&
             ether_rx_buf[6] == ETH_Addr_RAM[2] &&
             ether_rx_buf[7] == ETH_Addr_RAM[3] &&
             ether_rx_buf[8] == ETH_Addr_RAM[4] &&
             ether_rx_buf[9] == ETH_Addr_RAM[5])){
          // It's not ours
          hdr |= 0x1000;
        }
        // Is this multicast/broadcast?
        if(ether_rx_buf[4]&0x01){
          // Yes. Is it broadcast?        
          if(ether_rx_buf[4] == 0xFF &&
             ether_rx_buf[5] == 0xFF &&
             ether_rx_buf[6] == 0xFF &&
             ether_rx_buf[7] == 0xFF &&
             ether_rx_buf[8] == 0xFF &&
             ether_rx_buf[9] == 0xFF){
            // It's a broadcast packet
            hdr |= 0x4000;
          }else{
            // It's not a broadcast packet. Are we in broadcast mode?
            if(ETH_MECSR_MEBACK.PA < 6){
              // Yes, so discard this
              drop = 1;
              /* printf("3COM: Not Broadcast, DST %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
		 ether_rx_buf[4],ether_rx_buf[5],ether_rx_buf[6],ether_rx_buf[7],ether_rx_buf[8],ether_rx_buf[9]); */
            }
          }
        }else{
          // Not multicast/broadcast.
          if(hdr&0x1000 && ETH_MECSR_MEBACK.PA > 2){
            // And not mine, and we are not in promisc.
            /* printf("3COM: Not mine or multicast, DST %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
	       ether_rx_buf[4],ether_rx_buf[5],ether_rx_buf[6],ether_rx_buf[7],ether_rx_buf[8],ether_rx_buf[9]); */
            drop = 1;
          }
        }
        // Can we put it in A?
        if(ETH_MECSR_MEBACK.ABSW == 1 && drop == 0){
          // yes!
          // Obtain packet
          memcpy(ETH_RX_Buffer[0]+2,ether_rx_buf+4,pktlen);
          // Obtain header
          ETH_RX_Buffer[0][0] = ((hdr&0xFF00)>>8);
          ETH_RX_Buffer[0][1] = (hdr&0xFF);       
          ETH_MECSR_MEBACK.ABSW = 0; // Now belongs to host
          if(enet_trace){
            printf("3COM: PACKET STORED IN A\n");
          }
          if(ETH_MECSR_MEBACK.BBSW != 1){
            ETH_MECSR_MEBACK.RBBA = 0; // Packet B is older than this one.
          }
	  if(ETH_MECSR_MEBACK.AINTEN != 0){
	    multibus_interrupt(0);
	  }
        }else{
          // No, can we put it in B?
          if(ETH_MECSR_MEBACK.BBSW == 1 && drop == 0){
            // Yes!
            // Obtain packet
            memcpy(ETH_RX_Buffer[1]+2,ether_rx_buf+4,pktlen);
            // Obtain header
            ETH_RX_Buffer[1][0] = ((hdr&0xFF00)>>8);
            ETH_RX_Buffer[1][1] = (hdr&0xFF);
            if(enet_trace){
              printf("3COM: PACKET STORED IN B\n");
            }
            ETH_MECSR_MEBACK.BBSW = 0; // Now belongs to host
            ETH_MECSR_MEBACK.RBBA = 0; // Packet A is older than this one.
	    if(ETH_MECSR_MEBACK.BINTEN != 0){
	      multibus_interrupt(0);
	    }
          }else{
            // Can't do anything with it! Drop it!
            /* printf("3COM: PA exclusion, packet dropped: PA mode 0x%X and header word 0x%X\n",
	       ETH_MECSR_MEBACK.PA,hdr); */
          }
        }       
      }
    }
  }  
}

#ifdef HAVE_YAML_H
int yaml_network_mapping_loop(yaml_parser_t *parser){
  char key[128];
  char value[128];
  yaml_event_t event;
  int mapping_done = 0;
  key[0] = 0;
  value[0] = 0;
  while(mapping_done == 0){
    if(!yaml_parser_parse(parser, &event)){
      if(parser->context != NULL){
	printf("YAML: Parser error %d: %s %s\n", parser->error,parser->problem,parser->context);
      }else{
	printf("YAML: Parser error %d: %s\n", parser->error,parser->problem);
      }
      return(-1);
    }
    switch(event.type){
    case YAML_NO_EVENT:
      printf("No event?\n");
      break;
    case YAML_STREAM_START_EVENT:
    case YAML_DOCUMENT_START_EVENT:
      // printf("STREAM START\n");
      printf("Unexpected stream/document start\n");
      break;
    case YAML_STREAM_END_EVENT:
    case YAML_DOCUMENT_END_EVENT:
      // printf("[End Document]\n");
      printf("Unexpected stream/document end\n");
      break;
    case YAML_SEQUENCE_START_EVENT:
    case YAML_MAPPING_START_EVENT:
      printf("Unexpected sequence/mapping start\n");
      return(-1);
      break;
    case YAML_SEQUENCE_END_EVENT:
      printf("Unexpected sequence end\n");
      return(-1);
      break;
    case YAML_MAPPING_END_EVENT:
      mapping_done = 1;
      break;
    case YAML_ALIAS_EVENT:
      printf("Unexpected alias (anchor %s)\n", event.data.alias.anchor);
      return(-1);
      break;
    case YAML_SCALAR_EVENT:
      if(key[0] == 0){
	strncpy(key,(const char *)event.data.scalar.value,128);
      }else{
	strncpy(value,(const char *)event.data.scalar.value,128);
        if(strcmp(key,"interface") == 0){
	  strncpy(ether_iface,value,30);
	  printf("Using 3Com Ethernet interface %s\n",ether_iface);	  
	  goto value_done;
	}
	if(strcmp(key,"address") == 0){
	  int x = 0;
	  char *tok;
	  char *str = value;
	  while(x < 6){
	    long int val = 0;
	    tok = strtok(str," :\t\r\n");
	    if(str != NULL){ str = NULL; } // Clobber
	    if(tok != NULL){
	      val = strtol(tok,NULL,16);
	    }
	    ether_addr[x] = val;
	    x++;
	  }
	  printf("Using 3Com Ethernet address %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
		 ether_addr[0],ether_addr[1],ether_addr[2],ether_addr[3],ether_addr[4],ether_addr[5]);
	  
	  goto value_done;
	}	
	printf("network: Unknown key %s (value %s)\n",key,value);
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
