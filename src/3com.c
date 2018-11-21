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
#include <stdlib.h>
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
#ifdef USE_UTUN
#include <netinet/in.h>
#include <net/if_dl.h>
#include <net/if_arp.h>
#include <net/if_types.h>
#include <net/ethernet.h>
#endif
#include <net/bpf.h>
#ifdef USE_UTUN
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <sys/sys_domain.h>
#include <sys/kern_control.h>
#include <net/if_utun.h>
#endif
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
  logmsgf(LT_3COM,10,"3COM: Sending %d bytes\n",len+4);
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
  logmsgf(LT_3COM,10,"3COM: RX got %d bytes\n",(int)res);
  return(res);
}
#endif /* Linux ethertap code */

// Berkeley Packet Filter code
#if !defined (USES_ETHER_CODE) && defined (HAVE_NET_BPF_H)
#define USES_ETHER_CODE "BPF"

char ether_bpfn[64];
#ifdef USE_UTUN
// For tunnel frame diverter hack
char guest_ip_addr[32] = "";
ETH_HW_ADDR_Reg HOST_HW_ADDR;
struct in_addr HOST_IN_ADDR;
struct in_addr GUEST_IN_ADDR;
int utun_fd = -1;
int host_iface_state = -1;
int gen_arp_response = -1; // ARP response forgery timer
#endif

#ifdef USE_UTUN
void activate_utun(){
  struct ctl_info ctlInfo;
  struct sockaddr_ctl sc;
  
  strlcpy(ctlInfo.ctl_name, UTUN_CONTROL_NAME, sizeof(ctlInfo.ctl_name));
  utun_fd = socket(PF_SYSTEM, SOCK_DGRAM, SYSPROTO_CONTROL);
  if (utun_fd < 0) {
    perror("utun: socket()");	    
  }else{
    if(ioctl(utun_fd, CTLIOCGINFO, &ctlInfo) == -1){
      close(utun_fd);
      perror("utun: ioctl()");
    }else{
      logmsgf(LT_3COM,10,"ctl_info: {ctl_id: %ud, ctl_name: %s}\n",ctlInfo.ctl_id, ctlInfo.ctl_name);
      
      sc.sc_id = ctlInfo.ctl_id;
      sc.sc_len = sizeof(sc);
      sc.sc_family = AF_SYSTEM;
      sc.ss_sysaddr = AF_SYS_CONTROL;
      sc.sc_unit = 10;
      
      if(connect(utun_fd, (struct sockaddr *)&sc, sizeof(sc)) < 0){
	perror("utun: connect()");
		  close(utun_fd);
      }else{
	char syscmd[512];
	int rv=0;
	// Become nonblocking
	fcntl(utun_fd, F_SETFL, O_NONBLOCK);
	
	// Set IP addresses!		  
	// ifconfig utun9 inet 10.0.0.165 10.0.0.50 netmask 255.255.255.255
	sprintf(syscmd,"ifconfig utun9 inet %s",inet_ntoa(HOST_IN_ADDR));
	sprintf(syscmd,"%s %s netmask 255.255.255.255",syscmd,inet_ntoa(GUEST_IN_ADDR));
	rv = system(syscmd);
	if(rv < 0){
	  logmsgf(LT_3COM,0,"UTUN: Interface IP configuration failed, do it yourself\n");
	}
	// Done
	logmsgf(LT_3COM,1,"UTUN: Initialization completed!\n");
	host_iface_state++;
      }
    }
  }
}
#endif

int ether_init(){
  struct ifreq ifr;
  int fd, err, flags;
  int x=0;
  uint32_t y;
#ifdef USE_UTUN
  struct ifaddrs *addrlist = NULL;
  
  // Determine host's ethernet and IP addresses.
  x  = getifaddrs(&addrlist);
  if(x == 0){
    struct ifaddrs *addr = addrlist;
    while(addr != NULL){
      if(strcmp(addr->ifa_name,ether_iface) == 0){
	// This is the one we want!
	if(addr->ifa_addr->sa_family == AF_LINK){
	  // link-level address
	  struct sockaddr_dl *linkaddr = (struct sockaddr_dl *)addr->ifa_addr;
	  if(linkaddr->sdl_type == IFT_ETHER && linkaddr->sdl_alen == 6){
	    // Ethernet MAC	    
	    uint8_t *eaddr = (uint8_t *)LLADDR(linkaddr);
	    HOST_HW_ADDR.byte[0] = eaddr[0];
	    HOST_HW_ADDR.byte[1] = eaddr[1];
	    HOST_HW_ADDR.byte[2] = eaddr[2];
	    HOST_HW_ADDR.byte[3] = eaddr[3];
	    HOST_HW_ADDR.byte[4] = eaddr[4];
	    HOST_HW_ADDR.byte[5] = eaddr[5];
	    HOST_HW_ADDR.byte[6] = 0;
	    HOST_HW_ADDR.byte[7] = 0;	    
	    logmsgf(LT_3COM,10,"BPF: Host MAC address %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
		   HOST_HW_ADDR.byte[0],HOST_HW_ADDR.byte[1],HOST_HW_ADDR.byte[2],
		   HOST_HW_ADDR.byte[3],HOST_HW_ADDR.byte[4],HOST_HW_ADDR.byte[5]);
	  }else{
	    logmsgf(LT_3COM,10,"BPF: Non-ethernet link address, len %d data ",addr->ifa_addr->sa_len);
	    int y = 0;
	    while(y < addr->ifa_addr->sa_len){
	      if(y > 0){ logmsgf(LT_3COM,10,":"); }
	      logmsgf(LT_3COM,10,"%.2hhX",addr->ifa_addr->sa_data[y]);
	      y++;
	    }
	    logmsgf(LT_3COM,10,"\n");
	  }
	}
	if(addr->ifa_addr->sa_family == AF_INET){
	  // IPv4
	  struct sockaddr_in *ip4addr = (struct sockaddr_in *)addr->ifa_addr;
	  HOST_IN_ADDR.s_addr = ip4addr->sin_addr.s_addr;
	  logmsgf(LT_3COM,10,"BPF: Host IPv4 address %s\n",inet_ntoa(HOST_IN_ADDR));
	}
	// logmsgf(LT_3COM,10,"BPF: Addr family %d\n",addr->ifa_addr->sa_family);
      }
      addr = addr->ifa_next;
    }    
    freeifaddrs(addrlist);
  }else{
    perror("BPF:getifaddrs()");
    return(0);
  }

  if(HOST_HW_ADDR.byte[0] != 0 && HOST_IN_ADDR.s_addr != 0){
    host_iface_state = 0; // Pre-init done
    if(strlen(guest_ip_addr) > 0){
      if(inet_aton(guest_ip_addr,&GUEST_IN_ADDR) != 0){
	logmsgf(LT_3COM,10,"BPF: Guest IP address is %s\n",inet_ntoa(GUEST_IN_ADDR));
	// Initialize tunnel
	activate_utun();
      }else{
	logmsgf(LT_3COM,0,"UTUN: Unable to parse configured guest IP address, fallback to ARP\n");
      }
    }
  }
#endif

  x = 0;
  while(x < 16){
    sprintf(ether_bpfn,"/dev/bpf%d",x);
    // Open device
    fd = open(ether_bpfn, O_RDWR);
    if(fd < 0){
      if(errno == EBUSY){ x++; continue; }
      if(errno == ENOENT){
	logmsgf(LT_3COM,0,"BPF: Unable to find available BPF device (tried %d)\n",x);
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

  logmsgf(LT_3COM,1,"BPF: Operating\n");
  return(fd);
}

void ether_tx_pkt(uint8_t *data,uint32_t len){
  ssize_t res = 0;
  if(ether_fd < 0){ return; }

#ifdef USE_UTUN
  // HOST PACKET DIVERSION HACK
  // BPF on OS X cannot send packets to the local host, so we want to divert packets intended for the local host
  // to a tunnel interface instead.
  // Until this is fixed, this means we can only use IP to speak with the local host.

  // Is this a broadcast packet?  
  struct ether_header *header = (struct ether_header *)data;
  if(header->ether_dhost[0] == 0xFF && header->ether_dhost[1] == 0xFF && header->ether_dhost[2] == 0xFF &&
     header->ether_dhost[3] == 0xFF && header->ether_dhost[4] == 0xFF && header->ether_dhost[5] == 0xFF){
    // Yes, is it an ARP packet?
    if(ntohs(header->ether_type) == ETHERTYPE_ARP){
      // Yes.
      struct arphdr *arp_header = (struct arphdr *)(data+ETHER_HDR_LEN);
      // Ethernet request for IP address?
      if(ntohs(arp_header->ar_hrd) == ARPHRD_ETHER &&
	 ntohs(arp_header->ar_pro) == ETHERTYPE_IP &&
	 ntohs(arp_header->ar_op) == ARPOP_REQUEST){
	// Is it for the host?
	if((ntohl(*(uint32_t *)(data+(ETHER_HDR_LEN+24)))) == ntohl(HOST_IN_ADDR.s_addr)){
	  // Yes! Do we know our IP address?
	  if(host_iface_state == 0){
	    // No - Take it
	    GUEST_IN_ADDR = *(struct in_addr *)(data+(ETHER_HDR_LEN+14));
	    logmsgf(LT_3COM,10,"BPF: Guest IP address is %s\n",inet_ntoa(GUEST_IN_ADDR));
	    // Initialize tunnel
	    activate_utun();
	  }
	  if(host_iface_state > 0 && gen_arp_response < 0){
	    // Generate ARP response packet
	    gen_arp_response = 1; // Slight delay enough?
	  }
	}else{
	  logmsgf(LT_3COM,10,"PKT: DST IP 0x%.8X vs 0x%.8X\n",(ntohl(*(uint32_t *)(data+(ETHER_HDR_LEN+24)))),ntohl(HOST_IN_ADDR.s_addr));
	}
	/*
	logmsgf(LT_3COM,10,"ARP REQUEST: Find %s",inet_ntoa(*(struct in_addr *)(data+(ETHER_HDR_LEN+24))));
	logmsgf(LT_3COM,10," for %s",inet_ntoa(*(struct in_addr *)(data+(ETHER_HDR_LEN+14))));
	logmsgf(LT_3COM,10," (stored %s)\n",inet_ntoa(GUEST_IN_ADDR));
	*/
      }
    }
  }else{
    // Is this a unicast packet for our host?
    if(host_iface_state == 1){
      if(header->ether_dhost[0] == HOST_HW_ADDR.byte[0] &&
	 header->ether_dhost[1] == HOST_HW_ADDR.byte[1] &&
	 header->ether_dhost[2] == HOST_HW_ADDR.byte[2] &&
	 header->ether_dhost[3] == HOST_HW_ADDR.byte[3] &&
	 header->ether_dhost[4] == HOST_HW_ADDR.byte[4] &&
	 header->ether_dhost[5] == HOST_HW_ADDR.byte[5]){
	// Yes!
	// logmsgf(LT_3COM,10,"3COM: UNICAST PACKET FOR HOST\n");
	// IPv4?
	if(ntohs(header->ether_type) == ETHERTYPE_IP){
	  // Yes, relay it
	  uint8_t tunbuf[0x800];
	  memcpy(tunbuf+4,data+14,len-14); // Get data
	  *((uint32_t *)&tunbuf) = htonl(AF_INET);
	  res = write(utun_fd,tunbuf,len-10);
	  if(res < 0){
	    perror("utun:write()");
	  }  	  
	}
      }
    }
  }
#endif
    
  // logmsgf(LT_3COM,10,"Ether: Sending %d bytes\n",len);
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
#ifdef USE_UTUN
    // ARP forgery timer active?
    if(gen_arp_response >= 0){      
      gen_arp_response--;      
      if(gen_arp_response == -1){
	// Generate response
	// Packet goes at ether_rx_buf+4 to account for Linux ethernet header
	memset(ether_rx_buf, 0, 0x800); // Clobber
	// Ethernet header
	ether_rx_buf[4] = ETH_HW_ADDR.byte[0];   // Destination address
	ether_rx_buf[5] = ETH_HW_ADDR.byte[1];
	ether_rx_buf[6] = ETH_HW_ADDR.byte[2];
	ether_rx_buf[7] = ETH_HW_ADDR.byte[3];
	ether_rx_buf[8] = ETH_HW_ADDR.byte[4];
	ether_rx_buf[9] = ETH_HW_ADDR.byte[5];
	ether_rx_buf[10] = HOST_HW_ADDR.byte[0]; // Source Address
	ether_rx_buf[11] = HOST_HW_ADDR.byte[1];
	ether_rx_buf[12] = HOST_HW_ADDR.byte[2];
	ether_rx_buf[13] = HOST_HW_ADDR.byte[3];
	ether_rx_buf[14] = HOST_HW_ADDR.byte[4];
	ether_rx_buf[15] = HOST_HW_ADDR.byte[5];
	ether_rx_buf[16] = 0x08;                 // ARP (0x0806)
	ether_rx_buf[17] = 0x06; 
	// ARP packet
	ether_rx_buf[18] = 0x00;                 // HTYPE (1 = ethernet)
	ether_rx_buf[19] = 0x01;
	ether_rx_buf[20] = 0x08;                 // PTYPE (0x800 = IPv4)
	ether_rx_buf[21] = 0x00;
	ether_rx_buf[22] = 0x06;                 // HLEN (Ethernet = 6)
	ether_rx_buf[23] = 0x04;                 // PLEN (IPv4 = 4)
	ether_rx_buf[24] = 0x00;                 // OPER (0x0002 = reply)
	ether_rx_buf[25] = 0x02;
	ether_rx_buf[26] = HOST_HW_ADDR.byte[0]; // Sender Hardware Address
	ether_rx_buf[27] = HOST_HW_ADDR.byte[1];
	ether_rx_buf[28] = HOST_HW_ADDR.byte[2];
	ether_rx_buf[29] = HOST_HW_ADDR.byte[3];
	ether_rx_buf[30] = HOST_HW_ADDR.byte[4];
	ether_rx_buf[31] = HOST_HW_ADDR.byte[5];
	ether_rx_buf[32] = (HOST_IN_ADDR.s_addr&0x000000FF); // Sender Protocol Address
	ether_rx_buf[33] = ((HOST_IN_ADDR.s_addr&0x0000FF00)>>8);
	ether_rx_buf[34] = ((HOST_IN_ADDR.s_addr&0x00FF0000)>>16);
	ether_rx_buf[35] = ((HOST_IN_ADDR.s_addr&0xFF000000)>>24);
	ether_rx_buf[36] = ETH_HW_ADDR.byte[0]; // Target Hardware Address
	ether_rx_buf[37] = ETH_HW_ADDR.byte[1];
	ether_rx_buf[38] = ETH_HW_ADDR.byte[2];
	ether_rx_buf[39] = ETH_HW_ADDR.byte[3];
	ether_rx_buf[40] = ETH_HW_ADDR.byte[4];
	ether_rx_buf[41] = ETH_HW_ADDR.byte[5];
	ether_rx_buf[42] = (GUEST_IN_ADDR.s_addr&0x000000FF); // Target Protocol Address
	ether_rx_buf[43] = ((GUEST_IN_ADDR.s_addr&0x0000FF00)>>8);
	ether_rx_buf[44] = ((GUEST_IN_ADDR.s_addr&0x00FF0000)>>16);
	ether_rx_buf[45] = ((GUEST_IN_ADDR.s_addr&0xFF000000)>>24);
	// FCS	
	// The Lambda doesn't seem to actually check it, can we just leave it zeroes?
	// Yes!
	logmsgf(LT_3COM,10,"ENET: ARP response generated!\n");
	return(4+14+28+4); // Return this length (plus 4 byte header)
      }
    }
    // What about the tunnel?
    if(host_iface_state == 1){
      uint8_t tunbuf[0x800];
      uint32_t addrfam;
      res = read(utun_fd,tunbuf,0x800);
      if(res < 0){
	if(errno != EAGAIN && errno != EWOULDBLOCK){
	  perror("utun:read()");
	}
      }else{
	// We got something!
	// logmsgf(LT_3COM,10,"UTUN: Read got %ld bytes\n",res);
	// This will most likely be IPv4
	addrfam = ntohl(*((uint32_t *)&tunbuf));
	if(addrfam == AF_INET){
	  // Construct packet
	  // Packet goes at ether_rx_buf+4 to account for Linux ethernet header
	  memset(ether_rx_buf, 0, 0x800); // Clobber
	  // Ethernet header
	  ether_rx_buf[4] = ETH_HW_ADDR.byte[0];   // Destination address
	  ether_rx_buf[5] = ETH_HW_ADDR.byte[1];
	  ether_rx_buf[6] = ETH_HW_ADDR.byte[2];
	  ether_rx_buf[7] = ETH_HW_ADDR.byte[3];
	  ether_rx_buf[8] = ETH_HW_ADDR.byte[4];
	  ether_rx_buf[9] = ETH_HW_ADDR.byte[5];
	  ether_rx_buf[10] = HOST_HW_ADDR.byte[0]; // Source Address
	  ether_rx_buf[11] = HOST_HW_ADDR.byte[1];
	  ether_rx_buf[12] = HOST_HW_ADDR.byte[2];
	  ether_rx_buf[13] = HOST_HW_ADDR.byte[3];
	  ether_rx_buf[14] = HOST_HW_ADDR.byte[4];
	  ether_rx_buf[15] = HOST_HW_ADDR.byte[5];
	  ether_rx_buf[16] = 0x08;                 // IPv4 (0x0800)
	  ether_rx_buf[17] = 0x00; 
	  // Target packet
	  memcpy(ether_rx_buf+18,tunbuf+4,res-4);
	  // Tell the host about it
	  // logmsgf(LT_3COM,10,"UTUN: Forwarded\n");
	  return(4+14+(res-4)+4);
	}else{
	  logmsgf(LT_3COM,10,"UTUN: Got %ld bytes Not IPv4 (AF %d), disregarded\n",res,addrfam);
	}
      }
    }
#endif
    // Get more packets from BPF
    res = read(ether_fd,ether_bpf_buf,0x800);
    if(res < 0){
      if(errno != EAGAIN && errno != EWOULDBLOCK){
	perror("ether:read()");
      }
      return(0);
    }
    // logmsgf(LT_3COM,10,"Ether: read got %d bytes\n",(int)res);
    bpf_buf_length = res;
  }
  // There is a BPF header that must be dealt with
  // logmsgf(LT_3COM,10,"BPF: Header @ %d\n",bpf_buf_offset);
  bpf_header = (struct bpf_hdr *)(ether_bpf_buf+bpf_buf_offset);
  // Extract packet into ether_rx_buf+4 (to fake the 4-byte header that linux prepends)
  // logmsgf(LT_3COM,10,"BPF: Actual packet %d bytes @ offset %d\n",bpf_header->bh_caplen,bpf_header->bh_hdrlen);
  memcpy(ether_rx_buf+4,(uint8_t *)(ether_bpf_buf+bpf_buf_offset+bpf_header->bh_hdrlen),bpf_header->bh_caplen);
  /* logmsgf(LT_3COM,10,"PKT: DST %.2X:%.2X:%.2X:%.2X:%.2X:%.2X SRC %.2X:%.2X:%.2X:%.2X:%.2X:%.2X PTYPE %.2X %.2X\n",
	 ether_rx_buf[4],ether_rx_buf[5],ether_rx_buf[6],ether_rx_buf[7],ether_rx_buf[8],ether_rx_buf[9],
	 ether_rx_buf[10],ether_rx_buf[11],ether_rx_buf[12],ether_rx_buf[13],ether_rx_buf[14],ether_rx_buf[15],
	 ether_rx_buf[16],ether_rx_buf[17]); */
  // Do we have more packets?
  if((bpf_buf_offset+bpf_header->bh_hdrlen+bpf_header->bh_caplen) < bpf_buf_length){
    /* logmsgf(LT_3COM,10,"BPF: At %d, want %d, continuing buffer processing\n",
       (bpf_buf_offset+bpf_header->bh_hdrlen+bpf_header->bh_caplen),bpf_buf_length); */
    bpf_buf_offset += BPF_WORDALIGN(bpf_header->bh_hdrlen+bpf_header->bh_caplen);
  }else{
    // logmsgf(LT_3COM,10,"BPF: All done!\n");
    bpf_buf_offset = 0;
  }
  if(bpf_header->bh_caplen != bpf_header->bh_datalen){
    /* logmsgf(LT_3COM,10,"BPF: LENGTH MISMATCH: Captured %d of %d\n",
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
  // logmsgf(LT_3COM,10,"3COM: enet_read addr 0x%X\n",addr);
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
    logmsgf(LT_3COM,0,"3COM: UNKNOWN READ ADDR 0x%X\n",addr);
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
	logmsgf(LT_3COM,10,"3COM: RESET\n");
	enet_reset();
	return;
      }
      if(ETH_MECSR_MEBACK_Wt.AMSW == 1 && ETH_MECSR_MEBACK.AMSW == 0){
	logmsgf(LT_3COM,10,"3COM: AMSW given to interface: Our address is %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
	       ETH_Addr_RAM[0],ETH_Addr_RAM[1],ETH_Addr_RAM[2],ETH_Addr_RAM[3],ETH_Addr_RAM[4],ETH_Addr_RAM[5]);
	ETH_MECSR_MEBACK.AMSW = 1;
      }
      if(ETH_MECSR_MEBACK_Wt.TBSW == 1 && ETH_MECSR_MEBACK.TBSW == 0){
	logmsgf(LT_3COM,10,"3COM: TBSW given to interface: Packet offset ");
	uint32_t pktoff = (ETH_TX_Buffer[0x00]&0x07);
	uint32_t pktlen = 0x800;
	pktoff <<= 8;
	pktoff |= ETH_TX_Buffer[0x01];
	pktlen -= pktoff;
	logmsgf(LT_3COM,10,"%d, length %d\n",pktoff,pktlen);
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
	  logmsgf(LT_3COM,10,"3COM: TINTEN SET, INTERRUPTING\n");
	  multibus_interrupt(0);
	}
      }
      ETH_MECSR_MEBACK.PA = ETH_MECSR_MEBACK_Wt.PA;
      ETH_MECSR_MEBACK.BINTEN = ETH_MECSR_MEBACK_Wt.BINTEN;
      ETH_MECSR_MEBACK.AINTEN = ETH_MECSR_MEBACK_Wt.AINTEN;
      ETH_MECSR_MEBACK.TINTEN = ETH_MECSR_MEBACK_Wt.TINTEN;
      ETH_MECSR_MEBACK.JINTEN = ETH_MECSR_MEBACK_Wt.JINTEN;
      // Clobber state
      ETH_MECSR_MEBACK.TBSW = 0; // TB belongs to host
      ETH_MECSR_MEBACK.JAM = 0; // No collision
      ETH_MECSR_MEBACK.ABSW = 1; // A buffer is mine
      ETH_MECSR_MEBACK.BBSW = 1; // B buffer is mine
      ETH_MECSR_MEBACK.RBBA = 0; // A buffer has first packet
    }
    break;
  case 0x0400 ... 0x0403: // Address ROM
    logmsgf(LT_3COM,10,"3COM: WRITING ADDRESS ROM?\n");
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
    logmsgf(LT_3COM,0,"3COM: UNKNOWN WRITE ADDR 0x%X\n",addr);
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
	      logmsgf(LT_3COM,10,"3COM: DROP PACKET: Not Broadcast, DST %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
		 ether_rx_buf[4],ether_rx_buf[5],ether_rx_buf[6],ether_rx_buf[7],ether_rx_buf[8],ether_rx_buf[9]);
            }
          }
        }else{
          // Not multicast/broadcast.
          if(hdr&0x1000 && ETH_MECSR_MEBACK.PA > 2){
            // And not mine, and we are not in promisc.
            logmsgf(LT_3COM,10,"3COM: DROP PACKET: Not mine or multicast, DST %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
	       ether_rx_buf[4],ether_rx_buf[5],ether_rx_buf[6],ether_rx_buf[7],ether_rx_buf[8],ether_rx_buf[9]);
            drop = 1;
          }
        }
	// 3COM STORES PACKET B FIRST!
        // Can we put it in B?
        if(ETH_MECSR_MEBACK.BBSW == 1 && drop == 0){
          // yes!
          // Obtain packet
          memcpy(ETH_RX_Buffer[1]+2,ether_rx_buf+4,pktlen);
          // Obtain header
          ETH_RX_Buffer[1][0] = ((hdr&0xFF00)>>8);
          ETH_RX_Buffer[1][1] = (hdr&0xFF);       
	  logmsgf(LT_3COM,10,"3COM: PACKET STORED IN B\n");
          ETH_MECSR_MEBACK.BBSW = 0; // Now belongs to host
          if(ETH_MECSR_MEBACK.ABSW == 0){
            ETH_MECSR_MEBACK.RBBA = 0; // Packet A is older than packet B.
          }
	  if(ETH_MECSR_MEBACK.BINTEN != 0){
	    logmsgf(LT_3COM,10,"3COM: BINTEN SET, INTERRUPTING\n");
	    multibus_interrupt(0);
	  }
        }else{
          // No, can we put it in A?
          if(ETH_MECSR_MEBACK.ABSW == 1 && drop == 0){
            // Yes!
            // Obtain packet
            memcpy(ETH_RX_Buffer[0]+2,ether_rx_buf+4,pktlen);
            // Obtain header
            ETH_RX_Buffer[0][0] = ((hdr&0xFF00)>>8);
            ETH_RX_Buffer[0][1] = (hdr&0xFF);
	    logmsgf(LT_3COM,10,"3COM: PACKET STORED IN A\n");
            ETH_MECSR_MEBACK.ABSW = 0; // Now belongs to host
            ETH_MECSR_MEBACK.RBBA = 1; // Packet B is older than packet A.
	    if(ETH_MECSR_MEBACK.AINTEN != 0){
	      logmsgf(LT_3COM,10,"3COM: AINTEN SET, INTERRUPTING\n");
	      multibus_interrupt(0);
	    }
          }else{
            // Can't do anything with it! Drop it!
            logmsgf(LT_3COM,10,"3COM: PA exclusion, packet dropped: PA mode 0x%X and header word 0x%X\n",
	       ETH_MECSR_MEBACK.PA,hdr);
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
	logmsgf(LT_3COM,0,"YAML: Parser error %d: %s %s\n", parser->error,parser->problem,parser->context);
      }else{
	logmsgf(LT_3COM,0,"YAML: Parser error %d: %s\n", parser->error,parser->problem);
      }
      return(-1);
    }
    switch(event.type){
    case YAML_NO_EVENT:
      logmsgf(LT_3COM,0,"No event?\n");
      break;
    case YAML_STREAM_START_EVENT:
    case YAML_DOCUMENT_START_EVENT:
      // logmsgf(LT_3COM,10,"STREAM START\n");
      logmsgf(LT_3COM,0,"Unexpected stream/document start\n");
      break;
    case YAML_STREAM_END_EVENT:
    case YAML_DOCUMENT_END_EVENT:
      // logmsgf(LT_3COM,10,"[End Document]\n");
      logmsgf(LT_3COM,0,"Unexpected stream/document end\n");
      break;
    case YAML_SEQUENCE_START_EVENT:
    case YAML_MAPPING_START_EVENT:
      logmsgf(LT_3COM,0,"Unexpected sequence/mapping start\n");
      return(-1);
      break;
    case YAML_SEQUENCE_END_EVENT:
      logmsgf(LT_3COM,0,"Unexpected sequence end\n");
      return(-1);
      break;
    case YAML_MAPPING_END_EVENT:
      mapping_done = 1;
      break;
    case YAML_ALIAS_EVENT:
      logmsgf(LT_3COM,0,"Unexpected alias (anchor %s)\n", event.data.alias.anchor);
      return(-1);
      break;
    case YAML_SCALAR_EVENT:
      if(key[0] == 0){
	strncpy(key,(const char *)event.data.scalar.value,128);
      }else{
	strncpy(value,(const char *)event.data.scalar.value,128);
        if(strcmp(key,"interface") == 0){
	  strncpy(ether_iface,value,30);
	  logmsgf(LT_3COM,0,"Using 3Com Ethernet interface %s\n",ether_iface);	  
	  goto value_done;
	}
#ifdef USE_UTUN
        if(strcmp(key,"guest-ip") == 0){
	  strncpy(guest_ip_addr,value,31);
	  logmsgf(LT_3COM,0,"Using guest IP address %s\n",guest_ip_addr);	  
	  goto value_done;
	}
#endif
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
	  logmsgf(LT_3COM,0,"Using 3Com Ethernet address %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n",
		 ether_addr[0],ether_addr[1],ether_addr[2],ether_addr[3],ether_addr[4],ether_addr[5]);
	  
	  goto value_done;
	}	
	logmsgf(LT_3COM,0,"network: Unknown key %s (value %s)\n",key,value);
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
