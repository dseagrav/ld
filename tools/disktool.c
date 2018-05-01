/* Lambda disk tool

   Copyright 2016-2018
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
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int disk_fd = -1;            // FD for disk image
int file_fd = -1;            // FD for source/target file
char *disk_fname = NULL;     // Disk image filename
uint8_t DISK_BLOCK[1024];    // One disk block
uint32_t label_block = 0;    // Address of disk label (used for block offsets)

// TRUE MINI LABEL (physical block 10)
typedef struct rTrue_Mini_Label {
  uint32_t magic;       // "MINI"
  uint32_t length;      // in bytes
  uint32_t label_block;
  uint32_t backup_label_block;
  uint32_t bad_track_list;
  uint32_t spare1;
  uint32_t usable_tracks;
  uint32_t disk_type;
  uint32_t heads;
  uint32_t sectors;
  uint32_t cyls;
  uint32_t gap1;
  uint32_t gap2;
  uint32_t interleave;
  uint32_t skew;
  uint32_t sector_size;
  uint32_t bad_track_list_2;
  uint32_t backup_label_track;  
} __attribute__((packed)) True_Mini_Label;

// PARTITION TABLE ENTRY (in MAIN LABEL)
typedef struct rPartition {
  uint8_t name[4];
  uint32_t start;
  uint32_t size;
  uint8_t comment[16];
} __attribute__((packed)) Partition;

// MAIN LABEL
typedef struct rDisk_Label {
  uint32_t magic;
  uint32_t version;
  uint32_t cyls;
  uint32_t heads;
  uint32_t sectors;
  uint32_t sectors_per_cyl;
  uint8_t  microload[4];
  uint8_t  load[4];
  uint8_t  type[32];
  uint8_t  pack[32];
  uint8_t  comment[96];
  uint8_t  padding[320]; // Unused space
  uint32_t partitions;   // Part count
  uint32_t partsize;     // Words per partition entry
  Partition partent[29]; // Partitions
} __attribute__((packed)) Disk_Label;

int disk_block_read(int adr){
  ssize_t io_res; // Result of read/write operations  
  // Reposition the file pointer.
  off_t seek_res = lseek(disk_fd,(adr*0x400),SEEK_SET);
  if(seek_res < 0){
    // Seek error!
    perror("disktool: disk lseek()");
    return(-1);
  }
  // Read in a sector.
  io_res = read(disk_fd,DISK_BLOCK,1024);
  if(io_res < 0){
    perror("disktool: disk read()");
    return(-1);    
  }
  return(io_res);
}

int disk_block_write(int adr){
  ssize_t io_res; // Result of read/write operations  
  // Reposition the file pointer.
  off_t seek_res = lseek(disk_fd,(adr*0x400),SEEK_SET);
  if(seek_res < 0){
    // Seek error!
    perror("disktool: disk lseek()");
    return(-1);
  }
  // Write out a sector.
  io_res = write(disk_fd,DISK_BLOCK,1024);
  if(io_res < 0){
    perror("disktool: disk write()");
    return(-1);    
  }
  return(io_res);
}

int read_label_info(){
  True_Mini_Label *TML = NULL;
  Disk_Label *Label = NULL;
  // Obtain TRUE MINI LABEL
  int rv = 0;
  rv = disk_block_read(10);
  if(rv < 0){ return(rv); }
  // Is this it?
  TML = (True_Mini_Label *)DISK_BLOCK;
  if(TML->magic != 0x494E494D){
    printf("disktool: True Mini Label magic number mismatch: Expected 0x494E494D, got 0x%.8X\n",TML->magic);
    return(-1);
  }
  // This is it! Obtain actual label
  label_block = TML->label_block;
  rv = disk_block_read(label_block);
  if(rv < 0){ return(rv); }  
  TML = NULL;
  Label = (Disk_Label *)DISK_BLOCK;
  if(Label->magic != 0x4C42414C){
    printf("disktool: Label magic number mismatch: Expected 0x4C42414C, got 0x%.8X\n",Label->magic);
    return(-1);
  }
  if(Label->partsize != 7){
    printf("disktool: Unexpected partition entry size: Expected 7, got %d\n",Label->partsize);
    return(-1);    
  }
  return(0);
}

int disk_print(){
  int x;
  Disk_Label *Label = NULL;
  // Obtain label
  x = read_label_info();
  if(x < 0){ return(x); }
  Label = (Disk_Label *)DISK_BLOCK;
  // Print it
  printf("Disk label at block %.8X\n",label_block);
  printf("Disk from machine(s): %s\n",Label->pack);
  printf("%d partitions\n",Label->partitions);
  x = 0;
  if(Label->partitions > 0){
    printf("NAME START    SIZE     COMMENT\n");
    printf("---- -------- -------- ----------------\n");    
    while(x < Label->partitions){
      char comment[17];
      strncpy(comment,(char *)Label->partent[x].comment,16);
      comment[16] = 0; // Ensure null termination
      printf("%c%c%c%c %.8X %.8X %s\n",
	     Label->partent[x].name[0],Label->partent[x].name[1],Label->partent[x].name[2],Label->partent[x].name[3],
	     Label->partent[x].start,Label->partent[x].size,comment);
      x++;
    }
  }
  close(disk_fd);  
  return(0);
}

int read_partition(char *pname,char *filename){
  Disk_Label *Label = NULL;  
  int pslot = -1;
  uint32_t block;
  uint32_t psize;
  int x;
  // Find source partition
  x = read_label_info();
  if(x < 0){ return(x); }
  Label = (Disk_Label *)DISK_BLOCK;
  x = 0;
  while(x < Label->partitions){
    if(strncmp((char *)Label->partent[x].name,pname,4)==0){
      pslot = x;
      block = Label->partent[x].start+label_block;
      psize = Label->partent[x].size;
      break;
    }
    x++;
  }
  if(pslot < 0){
    printf("disktool: read: Unable to find source partition\n");
    return(-1);
  }
  // Open target
  file_fd = open(filename,O_RDWR|O_CREAT,0660);
  if(file_fd < 0){
    perror("disktool: file open()");
    return(-1);
  }
  x = 0;
  printf("Copying %s to %s (%.8X blocks)...\n",pname,filename,psize);
  while(x < psize){
    int rv = 0;
    printf("\rBlock %.8X ",x);
    rv = disk_block_read(block+x);
    if(rv < 0){
      printf("\n");
      return(rv);
    }
    // Write out
    rv = write(file_fd,DISK_BLOCK,1024);
    if(rv < 0){
      perror("disktool: file write()");
      return(-1);
    }
    // Next!
    x++;
  }
  printf("\nDone\n");
  close(file_fd);
  close(disk_fd);
  return(0);
}

int write_partition(char *pname,char *filename,char * comment){
  Disk_Label *Label = NULL;
  int pslot = -1;
  uint32_t block;
  uint32_t psize;
  uint32_t fsize;
  int x;
  struct stat file_stat;
  
  // Open source
  file_fd = open(filename,O_RDONLY);
  if(file_fd < 0){
    perror("disktool: file open()");
    return(-1);
  }
  // Fill stat
  x = fstat(file_fd,&file_stat);
  if(x < 0){
    perror("disktool: file stat()");
    return(-1);
  }
  // Find target partition
  x = read_label_info();
  if(x < 0){ return(x); }
  Label = (Disk_Label *)DISK_BLOCK;
  x = 0;
  while(x < Label->partitions){
    if(strncmp((char *)Label->partent[x].name,pname,4)==0){
      pslot = x;
      block = Label->partent[x].start+label_block;
      psize = Label->partent[x].size;
      // Will the file fit?
      fsize = file_stat.st_size/1024;
      if(fsize > psize){
	printf("disktool: write: File of size %.8X blocks won't fit into partition of %.8X blocks\n",fsize,psize);
	return(-1);	
      }
      // All good! Are we writing a comment?
      if(comment != NULL){
	int rv;
	// Yes, do that
	strncpy((char *)Label->partent[x].comment,comment,16);
	// Write back label
	rv = disk_block_write(label_block);
	if(rv < 0){ return(rv); }	
      }
      break;
    }
    x++;
  }
  if(pslot < 0){
    printf("disktool: write: Unable to find target partition\n");
    return(-1);
  }
  x = 0;
  printf("Copying %s to %s (%.8X blocks)...\n",filename,pname,psize);
  while(x < psize){
    int rv = 0;
    printf("\rBlock %.8X ",x);
    if(x < fsize){
      // Read in
      rv = read(file_fd,DISK_BLOCK,1024);
      if(rv < 0){
	perror("disktool: file read()");
	return(-1);
      }
    }else{
      if(x == fsize){
	// Zero buffer
	bzero(DISK_BLOCK,1024);
      }
    }
    // Write
    rv = disk_block_write(block+x);
    if(rv < 0){
      printf("\n");
      return(rv);
    }
    // Next!
    x++;
  }
  printf("\nDone\n");
  close(file_fd);
  close(disk_fd);  
  return(0);
}

int main(int argc, char *argv[]){
  // Handle command-line options
  if(argc < 2 || strncmp(argv[1],"help",4) == 0 || strncmp(argv[1],"-?",2) == 0){
    printf("Lambda Disktool v0.1\n");
    printf("Usage: disktool (command) (disk image file name) [parameters]...\n");
    printf(" Commands:\n");
    printf("  help       Prints this information\n");
    printf("  print      Prints the disk label(s) and partitition table, if present\n");
    printf("  read       Reads the given partition into the given file\n");
    printf("             Parameters: (partition name) (partition image file name)\n");
    printf("  write      Writes the given file into the given partition, optionally setting the comment\n");
    printf("             The remainder of the partition will be zeroed\n");
    printf("             Parameters: (partition image file name) (partition name) [partition comment]\n");
    return(0);
  }
  if(argc < 3){
    printf("disktool: Disk image file name is required; See \"disktool help\" for usage information.\n");
    return(-1);
  }
  // We have a disk filename, so open it.
  disk_fname = argv[2];
  disk_fd = open(disk_fname,O_RDWR);
  if(disk_fd < 0){
    perror("disktool: disk open()");
    return(-1);
  }  
  // Select option (or bail)
  if(argc >= 3){
    if(strncmp(argv[1],"print",5) == 0){
      return(disk_print());
    }
    if(strncmp(argv[1],"read",4) == 0){      
      if(argc < 4){
	printf("disktool: read: partition name is required\n");
      }      
      if(argc < 5){
	printf("disktool: read: partition image name is required\n");
      }
      return(read_partition(argv[3],argv[4]));
    }
    if(strncmp(argv[1],"write",5) == 0){
      char * comment = NULL;
      if(argc < 4){
	printf("disktool: write: partition image name is required\n");
      }      
      if(argc < 5){
	printf("disktool: write: partition name is required\n");
      }
      if(argc == 6){
	comment = argv[5];
      }
      return(write_partition(argv[4],argv[3],comment));
    }
    printf("disktool: Unknown parameters; See \"disktool help\" for usage information.\n");
    return(-1);
  }  
}
