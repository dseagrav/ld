/* Lambda FUSE LMFS implementation

   Copyright 2016-2019
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

#define FUSE_USE_VERSION 26

// Lispm epoch is 1/1/1900
#define EPOCH_OFFSET 2208988800

#include <fuse.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stddef.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// static const char *lmfs_path = "/lmfs";
// Default band and file name
static const char *lmfs_dnam = "disk.img";
static const char *lmfs_dbnd = "LMFS";
// Globals
int disk_fd = -1;            // FD for disk image
uint8_t DISK_BLOCK[1024];    // One disk block
uint32_t label_block = 0;    // Address of disk label (used for block offsets)
uint32_t band_block = 0;     // Address of LMFS band
uint32_t band_size = 0;      // Size of LMFS band
uint32_t put_base = 0;       // Page Usage Table
uint32_t put_size = 0;
uint32_t root_nblks = 0;     // Number of blocks in root directory map
uint32_t root_blk[32];       // Map
uint32_t root_blk_size[32];  // Map
uint32_t root_base = 0;      // Root Directory
uint32_t root_size = 0;
// Buffers
// uint8_t * ROOT_DIRECTORY = NULL; // Root directory

// Actual disk filename and band name
struct lmfs_config {
  char *disk_fname;
  char *band_name;
};

// Configuration alterable by command-line options
struct lmfs_config conf;

// Structures
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

// DIRECTORY HEADER
typedef struct rDirectory_Header {
  uint8_t header_id[28]; // "_THIS_IS_A_DIRECTORY_HEADER_"
  uint32_t version;
  uint8_t id_string[40];
  uint32_t cdate;
  uint32_t self;
  uint32_t flag;
  // Submaps follow
} __attribute__((packed)) Directory_Header;

// Attributes bits
#define ATTR_DONT_DELETE  0x0001
#define ATTR_CLOSED       0x0002
#define ATTR_DELETED      0x0004
#define ATTR_DUMPED       0x0008
#define ATTR_DONT_REAP    0x0010
#define ATTR_CHARACTERS   0x0020
#define ATTR_DIRECTORY    0x0040
#define ATTR_HEADER_BLOCK 0x4000

// A directory entry
typedef struct rDirectory_Entry {
  char fname[256];
  char ftype[256];
  int version;
  uint8_t bytesize;
  char fauth[256];
  uint32_t cdate;
  uint16_t map_size;     // Size of the map, in block entries
  uint32_t * map_block;      // Array of map blocks
  uint32_t * map_block_size; // Array of map block sizes
  uint32_t total_size;
  uint16_t attributes;
  uint8_t  proplist_len;
} Directory_Entry;

// Code
uint32_t get24(int offset){
  uint32_t ret = 0;
  ret  = (DISK_BLOCK[offset+0]<<16);
  ret |= (DISK_BLOCK[offset+1]<<8);
  ret |=  DISK_BLOCK[offset+2];
  return(ret);
};

uint16_t get16(int offset){
  uint16_t ret = 0;
  ret |= (DISK_BLOCK[offset+0]<<8);
  ret |=  DISK_BLOCK[offset+1];
  return(ret);
};

int disk_block_read(int adr){
  ssize_t io_res; // Result of read/write operations
  // Reposition the file pointer.
  off_t seek_res = lseek(disk_fd,(adr*0x400),SEEK_SET);
  if(seek_res < 0){
    // Seek error!
    perror("lmfuse: lseek()");
    return(-1);
  }
  // Read in a sector.
  io_res = read(disk_fd,DISK_BLOCK,1024);
  if(io_res < 0){
    perror("lmfuse: read()");
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
    printf("lmfuse: True Mini Label magic number mismatch: Expected 0x494E494D, got 0x%.8X\n",TML->magic);
    return(-1);
  }
  // This is it! Obtain actual label
  label_block = TML->label_block;
  rv = disk_block_read(label_block);
  if(rv < 0){ return(rv); }
  TML = NULL;
  Label = (Disk_Label *)DISK_BLOCK;
  if(Label->magic != 0x4C42414C){
    printf("lmfuse: Label magic number mismatch: Expected 0x4C42414C, got 0x%.8X\n",Label->magic);
    return(-1);
  }
  if(Label->partsize != 7){
    printf("lmfuse: Unexpected partition entry size: Expected 7, got %d\n",Label->partsize);
    return(-1);
  }
  return(0);
}

int read_directory_header(int blocks,uint32_t *block,uint32_t *block_size,uint8_t **DIRHEADER){
  // Result
  int res = 0;
  // Disk IO
  off_t seek_res; // Offset for seek
  ssize_t io_res; // Result of read/write operations
  int size = 0;   // Final size of structure
  int x = 0;
  // Header structure
  Directory_Header *Dir_Header = NULL;
  // Check parameters
  if(block == NULL || block_size == NULL){
    return -EINVAL;
  }
  // Reset header buffer pointer
  *DIRHEADER = NULL;
  // Walk block list
  size = 0;
  while(x < blocks){
    int block_byte_size = block_size[x]/8;
    // Bounds check
    if(block[x] > band_size){
      fprintf(stderr,"lmfuse: read_directory_header(): Attempt to read beyond end of LMFS\n");
      if(*DIRHEADER != NULL){ free(*DIRHEADER); }
      *DIRHEADER = NULL;
      return -EINVAL;
    }
    // Go!
    if(block_size[x] != block_byte_size*8){
      fprintf(stderr,"BLOCK BYTE SIZE MISMATCH: Got %d, expected %d\n",
	      (block_byte_size*8),block_size[x]);
      if(*DIRHEADER != NULL){ free(*DIRHEADER); }
      *DIRHEADER = NULL;
      return -EINVAL;
    }      
    if(*DIRHEADER == NULL){
      *DIRHEADER = calloc(block_byte_size,1);
      if(*DIRHEADER == NULL){
	fprintf(stderr,"lmfuse: calloc(): %s\n",strerror(errno));
	return errno;
      }
    }else{
      *DIRHEADER = realloc(*DIRHEADER,(size+block_byte_size));
      if(*DIRHEADER == NULL){
	fprintf(stderr,"lmfuse: realloc(): %s\n",strerror(errno));
	return errno;
      }      
    }
    // Seek to block
    seek_res = lseek(disk_fd,((band_block+block[x])*0x400),SEEK_SET);
    if(seek_res < 0){
      // Seek error!
      fprintf(stderr,"lmfuse: lseek(): %s\n",strerror(errno));
      free(*DIRHEADER);
      *DIRHEADER = NULL;
      return -EIO;      
    }
    // Read in the directory data
    io_res = read(disk_fd,(*DIRHEADER+size),block_byte_size);
    if(io_res < 0){
      fprintf(stderr,"lmfuse: read(): %s\n",strerror(errno));
      free(*DIRHEADER);
      *DIRHEADER = NULL;
      return -EIO;      
    }
    // Got it!
    fprintf(stderr,"lmfuse: Directory header read complete!\n");
    size += block_byte_size;    
    // Next!
    x++;
  }
  // Check structure
  Dir_Header = (Directory_Header *)*DIRHEADER;
  if(strncmp("_THIS_IS_A_DIRECTORY_HEADER_",(char *)*DIRHEADER,28) != 0){
    // No directory header?
    fprintf(stderr,"lmfuse: read_directory_header(): Directory has no header?\n");
    res = -EINVAL;
    goto fail;    
  }
  if(Dir_Header->version < 1 || Dir_Header->version > 2){    
    fprintf(stderr,"lmfuse: read_directory_header(): Unexpected header version %d\n",Dir_Header->version);
    res = -EINVAL;
    goto fail;
  }
  // Done, return size and user keeps buffer  
  res = size;
  return(res);
  // Fail!
 fail:
  free(*DIRHEADER);
  *DIRHEADER = NULL;
  return(res);
}

int read_directory(int blocks,uint32_t *block,uint32_t *block_size,uint8_t **DIRECTORY){
  // Result
  int res = 0;
  // Disk IO
  off_t seek_res; // Offset for seek
  ssize_t io_res; // Result of read/write operations  
  // Buffer for header
  uint8_t *DIRHEADER;
  Directory_Header *Dir_Header = NULL;
  // Submap pointer
  uint8_t *submap = NULL;
  uint16_t submap_size = 0;     // Size of the submap, in block entries
  uint32_t submap_index = 0;    // Index into the submap
  uint32_t submap_blk = 0;      // Block # for this submap entry
  uint32_t submap_blk_size = 0; // Block size for this submap entry
  // Directory buffer size
  int directory_size = 0;
  // Clobber directory pointer
  *DIRECTORY = NULL;
  // Obtain directory header
  res = read_directory_header(blocks,block,block_size,&DIRHEADER);
  if(res < 0){ goto fail; }
  Dir_Header = (Directory_Header *)DIRHEADER;  
  // fprintf(stderr,"lmfuse: read_directory(): Got header version %d\n",Dir_Header->version);
  submap = (DIRHEADER+0x54);
  if(Dir_Header->flag == 1){
    submap_size = (submap[0]<<8);
    submap_size |= submap[1];
    submap_size *= 6; // Bytes per entry.
    // If extended map, skip tail
    if(Dir_Header->version > 1){
      submap_size += 2;
    }
    submap += (submap_size+2); // Skip to next map
    fprintf(stderr,"lmfuse: read_directory(): Skipping submap 0 of size %d\n",submap_size);
  }else{
    if(Dir_Header->flag != 0){
      fprintf(stderr,"lmfuse: read_directory(): Bad flag value %d\n",Dir_Header->flag);
      res = -EINVAL;
      goto fail;
    }
  }
  // Obtain submap info
  submap_size = (submap[0]<<8);
  submap_size |= submap[1];
  submap_index = 0;
  submap += 2;
  while(submap_index < submap_size){
    uint32_t submap_byte_size = 0;
    submap_blk  = (submap[0]<<16);
    submap_blk |= (submap[1]<<8);
    submap_blk |=  submap[2];
    submap_blk_size  = (submap[3]<<16);
    submap_blk_size |= (submap[4]<<8);
    submap_blk_size |=  submap[5];
    submap_byte_size = submap_blk_size/8;
    fprintf(stderr,"SUBMAP: Ent %d Blk 0x%.6X Blksize 0x%.6X\n",
	    submap_index,submap_blk,submap_blk_size);
    if(submap_blk_size != submap_byte_size*8){
      fprintf(stderr,"SUBMAP BYTE SIZE MISMATCH: Got %d, expected %d\n",
	      (submap_byte_size*8),submap_blk_size);
      res = -EINVAL;
      goto fail;
    }
    if(*DIRECTORY == NULL){
      *DIRECTORY = calloc(submap_byte_size,1);
      if(*DIRECTORY == NULL){
	fprintf(stderr,"lmfuse: calloc(): %s\n",strerror(errno));
	res = errno;
	goto fail;
      }
    }else{
      // Need more space!
      *DIRECTORY = realloc(*DIRECTORY,(directory_size+submap_byte_size));
      if(*DIRECTORY == NULL){
	fprintf(stderr,"lmfuse: realloc(): %s\n",strerror(errno));
	res = errno;
	goto fail;
      }
    }
    // Seek to block
    seek_res = lseek(disk_fd,((band_block+submap_blk)*0x400),SEEK_SET);
    if(seek_res < 0){
      // Seek error!
      fprintf(stderr,"lmfuse: lseek(): %s\n",strerror(errno));
      res = -EIO;
      goto fail;
    }
    // Read in the directory data
    io_res = read(disk_fd,(*DIRECTORY+directory_size),submap_byte_size);
    if(io_res < 0){
      fprintf(stderr,"lmfuse: read(): %s\n",strerror(errno));
      res = -EIO;
      goto fail;
    }
    // Got it!
    fprintf(stderr,"SUBMAP: Directory data read complete!\n");
    directory_size += submap_byte_size;
    submap += 6;
    submap_index++;
  }
  // Done
  if(DIRHEADER != NULL){
    free(DIRHEADER); // Header is no longer required, free it
  }  
  res = directory_size;
  return(res);
  // Fail!
 fail:
  if(DIRHEADER != NULL){
    free(DIRHEADER);
  }
  if(*DIRECTORY != NULL){
    free(*DIRECTORY);
    *DIRECTORY = NULL;
  }
  return(res);  
}

// Parse directory entry into provided structure, returning length of entry
int lmfs_parse_dir_entry(uint8_t *DIRECTORY, Directory_Entry *dirent){
  if(dirent == NULL){ return -EINVAL; } // Go away
  if(DIRECTORY == NULL){ return -EINVAL; } // Also go away
  memset((char *)dirent, 0, sizeof(Directory_Entry));
  int dir_index = 0;
  int x = 0;
  // int map_size_bytes = 0;
  // Read to first RETURN (0x8D)
  while(DIRECTORY[dir_index] != 0x8D){
    dirent->fname[x] = DIRECTORY[dir_index];
    x++;
    dir_index++;
  }
  dirent->fname[x] = 0; // Terminate
  x = 0;
  dir_index++;
  // Read to next RETURN
  while(DIRECTORY[dir_index] != 0x8D){
    dirent->ftype[x] = DIRECTORY[dir_index];
    x++;
    dir_index++;
  }
  dirent->ftype[x] = 0; // Terminate
  x = 0;
  dir_index++;
  dirent->version  = (DIRECTORY[dir_index]<<16);
  dirent->version |= (DIRECTORY[dir_index+1]<<8);
  dirent->version |=  DIRECTORY[dir_index+2];
  dir_index += 3;
  dirent->bytesize = DIRECTORY[dir_index];
  dir_index++;
  // Next return is author
  while(DIRECTORY[dir_index] != 0x8D){
    dirent->fauth[x] = DIRECTORY[dir_index];
    x++;
    dir_index++;
  }
  dirent->fauth[x] = 0; // Terminate
  x = 0;
  dir_index++;
  dirent->cdate  = (DIRECTORY[dir_index]<<24);
  dirent->cdate |= (DIRECTORY[dir_index+1]<<16);
  dirent->cdate |= (DIRECTORY[dir_index+2]<<8);
  dirent->cdate |=  DIRECTORY[dir_index+3];
  // fprintf(stderr,"LMFS: cdate %d\n",cdate);
  dir_index += 4;
  dirent->map_size = (DIRECTORY[dir_index]<<8);
  dirent->map_size |= DIRECTORY[dir_index+1];
  dir_index += 2;
  // fprintf(stderr,"LMFS: map_size %d\n",dirent->map_size);
  // map_size_bytes = dirent->map_size*6; // Bytes per entry.
  // Read in map data
  dirent->map_block = calloc(dirent->map_size,4);
  if(dirent->map_block == NULL){
    fprintf(stderr,"LMFS: Unable to calloc() map block list");
    return errno;
  }
  dirent->map_block_size = calloc(dirent->map_size,4);
  if(dirent->map_block_size == NULL){
    fprintf(stderr,"LMFS: Unable to calloc() map block size list");
    return errno;
  }
  x = 0;
  dirent->total_size = 0;
  while(x < dirent->map_size){
    dirent->map_block[x]  = (DIRECTORY[dir_index]<<16);
    dirent->map_block[x] |= (DIRECTORY[dir_index+1]<<8);
    dirent->map_block[x] |=  DIRECTORY[dir_index+2];
    dirent->map_block_size[x]  = (DIRECTORY[dir_index+3]<<16);
    dirent->map_block_size[x] |= (DIRECTORY[dir_index+4]<<8);
    dirent->map_block_size[x] |=  DIRECTORY[dir_index+5];
    dirent->total_size += (dirent->map_block_size[x]/8);
    dir_index += 6;
    x++;
  }
  // dir_index += map_size_bytes; // Skip map
  dirent->attributes = (DIRECTORY[dir_index]<<8);
  dirent->attributes |= DIRECTORY[dir_index+1];
  // fprintf(stderr,"LMFS: attributes %X\n",dirent->attributes);
  dir_index += 2;
  dirent->proplist_len = DIRECTORY[dir_index];
  dir_index++;
  if(dirent->proplist_len > 0){
    int prop_name_len = -1;
    int prop_slot = 0;
    char prop_name[0xFF];
    int prop_opcode = 0;
    x = 0;
    prop_slot = 0;
    fprintf(stderr,"LMFS: proplist_len %d\n",dirent->proplist_len);
    while(prop_slot < dirent->proplist_len){
      prop_name_len = DIRECTORY[dir_index];
      dir_index++;
      x = 0;
      while(x < prop_name_len){
	prop_name[x] = DIRECTORY[dir_index];
	x++;
	dir_index++;
      }
      prop_name[x] = 0; // Terminate
      fprintf(stderr,"LMFS: proplist %d: %s = ",prop_slot,prop_name);
      prop_opcode = DIRECTORY[dir_index];
      dir_index++;
      fprintf(stderr,"opcode %d, ",prop_opcode);
      switch(prop_opcode){
      case 0: // FALSE
	fprintf(stderr,"FALSE\n");
	break;
      case 1: // TRUE
	fprintf(stderr,"TRUE\n");
	break;	
      case 4: // Integer (3 bytes)
	{
	  uint32_t value = 0;
	  value  = (DIRECTORY[dir_index]<<16);
	  value |= (DIRECTORY[dir_index+1]<<8);
	  value |=  DIRECTORY[dir_index+2];
	  dir_index += 3;
	  fprintf(stderr,"integer %d\n",value);	  
	}
	break;	
      case 5: // A string
	{
	  int string_len = 0;
	  char string_val[256];
	  string_len = DIRECTORY[dir_index];
	  dir_index++;
	  x = 0;
	  while(x < string_len){
	    string_val[x] = DIRECTORY[dir_index];
	    x++;
	    dir_index++;
	  }
	  string_val[x] = 0;
	  fprintf(stderr,"string %s\n",string_val);	  
	}
	break;
	
      default:
	fprintf(stderr,"LMFS: Unhandled proplist opcode %d\n",prop_opcode);
	return -ENOSYS; // For now	
      }
      prop_slot++;
    }
    // return -ENOSYS; // Shouldn't get here      
  }
  return dir_index;
}

// Find entry for given path, fill out dirent
int lmfs_getent(char *path,Directory_Entry *dirent){
  int ret = 0;
  int x = 0;
  if(dirent == NULL){ return -EINVAL; } // Go away
  if(path == NULL){ return -EINVAL; } // Also go away
  // Clear
  memset((char *)dirent, 0, sizeof(Directory_Entry));  
  // Is this the root directory?
  if(path[0] == '/' && path[1] == 0){    
    // Yes, fill this in
    uint8_t *RD_HEADER = NULL;
    Directory_Header *Root_Dir_Header = NULL;
    fprintf(stderr,"lmfs_getent(): Search is for root directory\n");
    // int blocks,uint32_t *block,uint32_t *block_size,
    ret = read_directory_header(root_nblks,(uint32_t *)&root_blk,(uint32_t *)&root_blk_size,&RD_HEADER);
    if(ret < 0){
      fprintf(stderr,"lmfs_getent(): Unable to read root directory header\n");
      return ret;
    }
    Root_Dir_Header = (Directory_Header *)RD_HEADER;    
    strcpy(dirent->fname,(char *)Root_Dir_Header->id_string);
    strcpy(dirent->ftype,"DIRECTORY");
    dirent->version = 1;
    dirent->bytesize = 8;
    strcpy(dirent->fauth,"lispm");
    dirent->cdate = Root_Dir_Header->cdate;
    dirent->map_size = root_nblks;
    // Copy over map data
    dirent->map_block = calloc(dirent->map_size,4);
    if(dirent->map_block == NULL){
      fprintf(stderr,"LMFS: Unable to calloc() map block list");
      return errno;
    }
    dirent->map_block_size = calloc(dirent->map_size,4);
    if(dirent->map_block_size == NULL){
      fprintf(stderr,"LMFS: Unable to calloc() map block size list");
      return errno;
    }
    x = 0;
    dirent->total_size = 0;
    while(x < dirent->map_size){
      dirent->map_block[x] = root_blk[x];
      dirent->map_block_size[x] = root_blk_size[x];
      dirent->total_size += (dirent->map_block_size[x]/8);
      x++;
    }    
    dirent->attributes = 0x4042; // HEADER-BLOCK, DIRECTORY, CLOSED
    // Done with header
    free(RD_HEADER);
    fprintf(stderr,"lmfs_getent(): Returning root directory\n");    
    return(0); // OK!
  }else{
    char *path_element = NULL;
    char *next_path_element = "";
    char *saveptr = NULL;
    uint8_t *DIRECTORY = NULL;
    int dir_size = 0;
    // Obtain root
    fprintf(stderr,"lmfs_getent(): Reading root directory\n");  
    ret = read_directory(root_nblks,(uint32_t *)&root_blk,(uint32_t *)&root_blk_size,&DIRECTORY);
    if(ret < 0){
      fprintf(stderr,"LMFS: GETENT(): Unable to get root directory\n");
      return ret;
    }
    dir_size = ret;
    fprintf(stderr,"lmfs_getent(): Beginning path parse\n");    
    path_element = strtok_r((char *)path,"/",&saveptr);
    if(path_element == NULL){
      fprintf(stderr,"LMFS: GETENT(): First path_element is null\n");
    }
    while(path_element != NULL){
      next_path_element = strtok_r(NULL,"/",&saveptr);
      if(path_element == NULL){
	fprintf(stderr,"LMFS: GETENT(): path_element is null\n");
	break;
      }else{
	int dir_index = 0;
	int saved_dir_index = -1;
	int saved_di_version = -1;
	char *path_element_name = NULL;
	char *path_element_type = NULL;
	char *path_element_version = NULL;
	int path_element_ver = -1;
	char *pe_saveptr = NULL;
	// fprintf(stderr,"LMFS: GETENT(): path_element = %s\n",path_element);
	// Does it have a dot?
	if(strstr(path_element,".") != NULL){
	  // Split for filename and file type
	  path_element_name = strtok_r(path_element,".",&pe_saveptr);
	  path_element_type = strtok_r(NULL,"/",&pe_saveptr);
	  if(strstr(path_element_type,"#") != NULL){
	    path_element_type = strtok_r(path_element_type,"#",&pe_saveptr); // Chop off before #
	    path_element_version = strtok_r(NULL,"/",&pe_saveptr);           // Chop off version 
	  }else{
	    path_element_version = NULL;
	  }
	}else{
	  // Otherwise take it as-is
	  path_element_name = path_element;
	  path_element_type = NULL;
	  if(strstr(path_element_name,"#") != NULL){
	    path_element_name = strtok_r(path_element_name,"#",&pe_saveptr);
	    path_element_version = strtok_r(NULL,"/",&pe_saveptr);           // Chop off version	    
	  }else{
	    path_element_version = NULL;
	  }	  
	}
	fprintf(stderr,"LMFS: GETENT: path_element name %s, type %s, version %s\n",
		path_element_name != NULL ? path_element_name : "NULL",
		path_element_type != NULL ? path_element_type : "NULL",
		path_element_version != NULL ? path_element_version : "NULL");
	if(path_element_version != NULL){
	  path_element_ver = atoi(path_element_version);
	}else{
	  path_element_ver = -1;
	}
	dir_index = 0;
	while(dir_index < dir_size){
	  int dirent_len = 0;
	  Directory_Entry tgt_dirent;	  
	  // Parse
	  dirent_len = lmfs_parse_dir_entry(DIRECTORY+dir_index,&tgt_dirent);
	  if(dirent_len < 0){
	    // Error, bail
	    free(DIRECTORY);
	    return dirent_len;
	  }
	  // Name match?
	  if(strcasecmp(tgt_dirent.fname,path_element_name) == 0){
	    // Yes! Do we have a type?
	    if(path_element_type != NULL){
	      // Yes, it must match too
	      if(strcasecmp(tgt_dirent.ftype,path_element_type) == 0){
		// Do we have a version number?
		if(path_element_ver > 0){
		  // Yes, must match exactly
		  if(path_element_ver == tgt_dirent.version){
		    // Match!
		    // Directories cannot have type, so since we have a type we must be at the end of the path.
		    strcpy(dirent->fname,tgt_dirent.fname);
		    strcpy(dirent->ftype,tgt_dirent.ftype);
		    dirent->version = tgt_dirent.version;
		    dirent->bytesize = tgt_dirent.bytesize;
		    strcpy(dirent->fauth,tgt_dirent.fauth);
		    dirent->cdate = tgt_dirent.cdate;
		    dirent->map_size = tgt_dirent.map_size;
		    // Copy over map data pointers
		    dirent->map_block = tgt_dirent.map_block;
		    dirent->map_block_size = tgt_dirent.map_block_size;
		    dirent->total_size = tgt_dirent.total_size;
		    dirent->attributes = tgt_dirent.attributes;
		    dirent->proplist_len = tgt_dirent.proplist_len;
		    free(DIRECTORY);
		    return(0);		    
		  }
		}else{
		  // No, find the newest version
		  if(tgt_dirent.version > saved_di_version){
		    // This is newer, save it for later.
		    fprintf(stderr,"LMFS: Saving dir index %d with version %d\n",dir_index,tgt_dirent.version);		    
		    saved_dir_index = dir_index;
		    saved_di_version = tgt_dirent.version;
		  }
		  // fprintf(stderr,"LMFS: GETENT: NEWEST VERSION CHECK REQUIRED\n");
		}
	      }
	    }else{
	      // No. We can accept a directory.
	      if(strlen(tgt_dirent.ftype) == 0 || strcasecmp(tgt_dirent.ftype,"DIRECTORY") == 0){
		// Match! Is this the end of the line?
		if(next_path_element == NULL){		  
		  // Final destination. Copy to destination dirent
		  strcpy(dirent->fname,tgt_dirent.fname);
		  strcpy(dirent->ftype,tgt_dirent.ftype);
		  dirent->version = tgt_dirent.version;
		  dirent->bytesize = tgt_dirent.bytesize;
		  strcpy(dirent->fauth,tgt_dirent.fauth);
		  dirent->cdate = tgt_dirent.cdate;
		  dirent->map_size = tgt_dirent.map_size;
		  // Copy over map data pointers
		  dirent->map_block = tgt_dirent.map_block;
		  dirent->map_block_size = tgt_dirent.map_block_size;
		  dirent->total_size = tgt_dirent.total_size;
		  dirent->attributes = tgt_dirent.attributes;
		  dirent->proplist_len = tgt_dirent.proplist_len;
		  free(DIRECTORY);
		  return(0);
		}else{
		  // This is our target, but we have another level of subdirectory below it.
		  fprintf(stderr,"LMFS: GETENT: SUB DIRECTORY REQUIRED\n");
		  // We're done with the directory, so release it.
		  free(DIRECTORY);
		  // Reset index
		  dir_index = 0;
		  // Read sub directory
		  fprintf(stderr,"lmfs_getent(): Reading directory\n");
		  ret = read_directory(tgt_dirent.map_size,tgt_dirent.map_block,tgt_dirent.map_block_size,&DIRECTORY);
		  if(ret < 0){
		    fprintf(stderr,"LMFS: GETENT(): Unable to get directory\n");
		    return ret;
		  }
		  dir_size = ret;
		  // Free the block lists, since we don't need them anymore
		  free(tgt_dirent.map_block);
		  free(tgt_dirent.map_block_size);
		  break;
		  // return -ENOENT; // Die for now
		}
	      }
	    }
	  }
	  // Done with this entry
	  free(tgt_dirent.map_block);
	  free(tgt_dirent.map_block_size);
	  // Next!
	  // return -ENOSYS; // Or not...	  
	  dir_index += dirent_len;
	}
	// Did we get here early?
	if(dir_index < dir_size){
	  // Yes, we are going to a subdirectory.
	  fprintf(stderr,"LMFS: GETENT(): Selecting next level\n");
	  // return -ENOENT;	  
	}else{
	  // Do we have a saved item?
	  if(saved_dir_index > -1){
	    int dirent_len = 0;
	    Directory_Entry tgt_dirent;	    
	    fprintf(stderr,"LMFS: GETENT(): SENDING SAVED DIR-INDEX %d (version %d)!\n",saved_dir_index,saved_di_version);
	    dirent_len = lmfs_parse_dir_entry(DIRECTORY+saved_dir_index,&tgt_dirent);
	    if(dirent_len < 0){
	      // Error, bail
	      free(DIRECTORY);
	      return dirent_len;
	    }
	    strcpy(dirent->fname,tgt_dirent.fname);
	    strcpy(dirent->ftype,tgt_dirent.ftype);
	    dirent->version = tgt_dirent.version;
	    dirent->bytesize = tgt_dirent.bytesize;
	    strcpy(dirent->fauth,tgt_dirent.fauth);
	    dirent->cdate = tgt_dirent.cdate;
	    dirent->map_size = tgt_dirent.map_size;
	    // Copy over map data pointers
	    dirent->map_block = tgt_dirent.map_block;
	    dirent->map_block_size = tgt_dirent.map_block_size;
	    dirent->total_size = tgt_dirent.total_size;
	    dirent->attributes = tgt_dirent.attributes;
	    dirent->proplist_len = tgt_dirent.proplist_len;
	    free(DIRECTORY);
	    return(0);
	  }
	  // If we got here, no match this level, bail
	  free(DIRECTORY);
	  return -ENOENT;
	}
      }
      path_element = next_path_element;
      next_path_element = NULL;
    }
    
  }
  return -ENOSYS; // Shouldn't get here
}

static int lmfs_readdir(const char *path, void *buf, fuse_fill_dir_t filler,
			 off_t offset, struct fuse_file_info *fi){
  (void) offset;
  (void) fi;
  // Read the entries in a directory.
  int  ret = 0;
  uint8_t *DIRECTORY = NULL;
  int dir_size = 0;
  int dir_index = 0;

  // Find this directory entry, if it exists
  Directory_Entry dirent;
  fprintf(stderr,"readdir(): searching for %s\n",path);  
  ret = lmfs_getent((char *)path,&dirent);
  if(ret < 0){
    fprintf(stderr,"readdir(): getent blew it!\n");    
    return ret;
  }
  fprintf(stderr,"readdir(): getent found a winner!\n");
  // Is it really a directory?
  if((dirent.attributes&ATTR_DIRECTORY) == 0){
    // No.
    free(dirent.map_block);
    free(dirent.map_block_size);
    return -ENOTDIR;
  }
  // Obtain directory
  ret = read_directory(dirent.map_size,dirent.map_block,dirent.map_block_size,&DIRECTORY);
  if(ret < 0){
    free(dirent.map_block);
    free(dirent.map_block_size);
    return ret; // Bail
  }
  dir_size = ret;
  // All done with directory block lists
  free(dirent.map_block);
  free(dirent.map_block_size);
  // Ready to parse directory
  fprintf(stderr,"readdir(): Directory loaded, %d bytes\n",dir_size);
  // Even the unix root has . and ..
  filler(buf, ".", NULL, 0);
  filler(buf, "..", NULL, 0);
  // Now the rest
  while(dir_index < dir_size){
    // One entry
    int dirent_len = 0;
    struct stat stbuf;
    // Clobber
    memset((char *)&stbuf, 0, sizeof(struct stat));    
    // Parse
    dirent_len = lmfs_parse_dir_entry(DIRECTORY+dir_index,&dirent);
    if(dirent_len < 0){
      // Error, bail
      return dirent_len;
    }
    // Convert stuff to unix
    // if(strcasecmp(dirent.ftype,"DIRECTORY") != 0){
    if((dirent.attributes&ATTR_DIRECTORY) == 0){
      // Regular file. Append type to name.
      strcat(dirent.fname,".");
      strcat(dirent.fname,dirent.ftype);
      // Also append version number
      sprintf(dirent.fname,"%s#%d",dirent.fname,dirent.version);
      // Fill stat
      stbuf.st_mode = S_IFREG | 0660;
      stbuf.st_nlink = 1;
    }else{
      // This is a directory
      stbuf.st_mode = S_IFDIR | 0750;
      stbuf.st_nlink = 2;      
    }
    // Common stuff
    stbuf.st_mtimespec.tv_sec = dirent.cdate-EPOCH_OFFSET;
    stbuf.st_size = dirent.total_size;
    stbuf.st_blocks = dirent.total_size/512;
    if(dirent.total_size > stbuf.st_blocks*512){
      stbuf.st_blocks++;
    }
    stbuf.st_blksize = 1024;
    // Fill in
    filler(buf, dirent.fname, &stbuf, 0);
    // Next!
    free(dirent.map_block);
    free(dirent.map_block_size);
    dir_index += dirent_len;
  }
  // filler(buf, lmfs_path + 1, NULL, 0);  
  // All done!
  free(DIRECTORY);
  return 0;  
}

static int lmfs_getattr(const char *path, struct stat *stbuf){
  // Get attributes. Fill out a stat() buffer.
  int res = -ENOENT;
  Directory_Entry dirent;
  // Clobber result buffer
  memset(stbuf, 0, sizeof(struct stat));

  // Find this directory entry, if it exists
  fprintf(stderr,"getattr(): searching for %s\n",path);  
  res = lmfs_getent((char *)path,&dirent);
  if(res < 0){
    fprintf(stderr,"getattr(): getent blew it!\n");
    return res;
  }
  fprintf(stderr,"getattr(): getent found a winner!\n");  
  // Take it!
  if((dirent.attributes&ATTR_DIRECTORY) == 0){
    // Regular file. Fill stat
    stbuf->st_mode = S_IFREG | 0660;
    stbuf->st_nlink = 1;
  }else{
    // This is a directory
    stbuf->st_mode = S_IFDIR | 0750;
    stbuf->st_nlink = 2;
  }
  // Common stuff
  stbuf->st_mtimespec.tv_sec = dirent.cdate-EPOCH_OFFSET;
  stbuf->st_size = dirent.total_size;
  stbuf->st_blksize = 1024;
  stbuf->st_blocks = dirent.total_size/512;
  if(dirent.total_size > stbuf->st_blocks*512){
    stbuf->st_blocks++;
  }
  // Done
  free(dirent.map_block);
  free(dirent.map_block_size);
  return(0);
}

static int lmfs_open(const char *path, struct fuse_file_info *fi){
  // fprintf(stderr,"LMFS: OPEN\n");
  int res = -ENOENT;
  Directory_Entry dirent;
  // Find this directory entry, if it exists
  fprintf(stderr,"open(): searching for %s\n",path);
  res = lmfs_getent((char *)path,&dirent);
  if(res < 0){
    fprintf(stderr,"open(): getent blew it!\n");
    return res;
  }
  fprintf(stderr,"open(): getent found a winner!\n");
  // Take it!
  if((dirent.attributes&ATTR_DIRECTORY) != 0){
    // This is a directory!
    return -EISDIR;
  }
  // All good!
  return(0);
}

static int lmfs_read(const char *path, char *buf, size_t size, off_t offset,
		      struct fuse_file_info *fi){
  int res = -ENOENT;
  Directory_Entry dirent;
  int blk_index = 0;
  int blk_start_bytes = 0;
  int bytes_read = 0;
  // Find this directory entry, if it exists
  fprintf(stderr,"read(): Want to read %ld bytes at offset %ld in file %s\n",size,offset,path);
  res = lmfs_getent((char *)path,&dirent);
  if(res < 0){
    fprintf(stderr,"read(): getent blew it!\n");
    return res;
  }
  fprintf(stderr,"read(): getent found a winner!\n");
  // Good?
  if((dirent.attributes&ATTR_DIRECTORY) != 0){
    // This is a directory!
    return -EISDIR;
  }  
  // Now we have the blocks for this file
  if(offset > dirent.total_size){
    fprintf(stderr,"read(): Attempt to read past end of file?\n");    
    return(0); // Can't read past EOF
  }  
  // Read size bytes starting at offset
  bytes_read = 0;
  while(blk_index < dirent.map_size && bytes_read < size){
    int blk_size_bytes = dirent.map_block_size[blk_index]/8;
    int offset_into_block = offset-blk_start_bytes;
    int read_size = 0;
    ssize_t io_res; // Result of read/write operations
    off_t seek_res;
    fprintf(stderr,"read(): Block %d has size %d (%d bytes), starts at %d\n",
	    blk_index,dirent.map_block_size[blk_index],blk_size_bytes,blk_start_bytes);
    if(offset_into_block < 0){ offset_into_block = 0; }
    if(offset_into_block > blk_size_bytes){
      // Skip ahead
      blk_start_bytes += blk_size_bytes;
      blk_index++;
      continue;
    }
    // Our start point is either in this block or in a previous block.
    read_size = blk_size_bytes-offset_into_block; // Subtract offset if we have one
    fprintf(stderr,"read(): blksize %d - offset %d = read-size %d bytes\n",
	    blk_size_bytes,offset_into_block,read_size);
    // Will we finish reading inside this block?
    if(bytes_read+read_size > size){
      // Yes. Take off the overage.      
      read_size -= (bytes_read+read_size)-size;
      fprintf(stderr,"read(): bytes_read %d + read_size > size %ld, corrected to %d\n",
	      bytes_read,size,read_size);
    }
    fprintf(stderr,"read(): Reading %d bytes from block %d\n",read_size,blk_index);
    // Seek to block (plus offset)
    seek_res = lseek(disk_fd,(((band_block+dirent.map_block[blk_index])*0x400)+offset_into_block),SEEK_SET);
    if(seek_res < 0){
      // Seek error!
      fprintf(stderr,"lmfuse: lseek(): %s\n",strerror(errno));
      return -EIO;
    }
    // Read in data
    io_res = read(disk_fd,buf+bytes_read,read_size);
    if(io_res < 0){
      fprintf(stderr,"lmfuse: read(): %s\n",strerror(errno));
      return -EIO;
    }
    bytes_read += read_size;
    blk_start_bytes += blk_size_bytes;    
    blk_index++;    
  }
  fprintf(stderr,"read(): Done, got %d bytes\n",bytes_read);
  if((dirent.attributes&ATTR_CHARACTERS) != 0){
    // Do charset translation (LMI-to-unix)
    int x=0;
    while(x < bytes_read){
      if((buf[x]&0x80) != 0){
	// Just unset the high bit?
	buf[x] &= 0x7F;
	// CR becomes LF
	if(buf[x] == 0x0D){ buf[x] = 0x0A; }
      }
      x++;
    }    
  }
  return(bytes_read);
}

static struct fuse_operations lmfs_oper = {
  .getattr= lmfs_getattr,
  .readdir= lmfs_readdir,
  .open= lmfs_open,
  .read= lmfs_read,
};

enum {
  KEY_HELP,
  KEY_VERSION,
};

#define LMFS_OPT(t, p, v) { t, offsetof(struct lmfs_config, p), v }

static struct fuse_opt lmfs_opts[] = {
  LMFS_OPT("disk=%s",disk_fname,0),  
  LMFS_OPT("band=%s",band_name,0),
  FUSE_OPT_KEY("-V",             KEY_VERSION),
  FUSE_OPT_KEY("--version",      KEY_VERSION),
  FUSE_OPT_KEY("-h",             KEY_HELP),
  FUSE_OPT_KEY("--help",         KEY_HELP),
  FUSE_OPT_END
};

static int lmfs_opt_proc(void *data, const char *arg, int key, struct fuse_args *outargs){
  switch (key) {
  case KEY_HELP:
    fprintf(stderr,
	    "usage: %s mountpoint [options]\n"
	    "\n"
	    "LMFS options:\n"
	    "    -o disk=(file name) Disk image containing LMFS filesystem\n"
	    "    -o band=(band name) LMFS band name\n"
	    "\n"
	    "general options:\n"
	    "    -o opt,[opt...]  mount options\n"
	    "    -h   --help      print help\n"
	    "    -V   --version   print version\n"
	    "\n"
	    , outargs->argv[0]);
    fuse_opt_add_arg(outargs, "-ho");
    fuse_main(outargs->argc, outargs->argv, &lmfs_oper, NULL);
    exit(1);

  case KEY_VERSION:
    fprintf(stderr, "LMFuSe version 0.01 (LMFS version 5)\n");
    fuse_opt_add_arg(outargs, "--version");
    fuse_main(outargs->argc, outargs->argv, &lmfs_oper, NULL);
    exit(0);
  }
  return 1;
}

int main(int argc, char *argv[]){
  Disk_Label *Label = NULL;
  int pslot = -1;
  int x;
  struct fuse_args args = FUSE_ARGS_INIT(argc, argv);

  memset(&conf, 0, sizeof(conf));

  fuse_opt_parse(&args, &conf, lmfs_opts, lmfs_opt_proc);

  // Use defaults if name not provided
  if(conf.disk_fname == NULL){
    conf.disk_fname = (char *)lmfs_dnam;
  }
  if(conf.band_name == NULL){
    conf.band_name = (char *)lmfs_dbnd;
  }

  // Obtain disk image
  disk_fd = open(conf.disk_fname,O_RDWR);
  if(disk_fd < 0){
    printf("lmfuse: open() %s: %s\n",conf.disk_fname,strerror(errno));
    return(-1);
  }
  // Obtain band info
  x = read_label_info();
  if(x < 0){ return(x); }
  Label = (Disk_Label *)DISK_BLOCK;
  x = 0;
  while(x < Label->partitions){
    if(strncmp((char *)Label->partent[x].name,conf.band_name,4)==0){
      pslot = x;
      band_block = Label->partent[x].start+label_block;
      band_size = Label->partent[x].size;
      break;
    }
    x++;
  }
  if(pslot < 0){
    printf("lmfuse: Unable to find partition %s\n",conf.band_name);
    close(disk_fd);
    return(-1);
  }
  // Read configuration block
  x = disk_block_read(band_block);
  if(x < 0){ close(disk_fd); return(x); }
  // Fetch fields
  {
    uint32_t version = get24(0);
    uint32_t check = get24(3);
    uint32_t fs_size = get24(6);
    // ssize_t io_res; // Result of read/write operations
    // off_t seek_res;
    put_base = get24(9);
    put_size = get24(12);
    root_nblks = get16(15);
    x = 0;
    while(x < root_nblks){
      root_blk[x] = get24(17+(x*6));
      root_blk_size[x] = get24(20+(x*6)); // Bits!      
      x++;
    }
    if(version != 5){
      printf("lmfuse: Unexpected LMFS version (expected 5, got %d)\n",version);
      close(disk_fd);
      return(-1);
    }
    if(check != 0){
      printf("lmfuse: Unexpected LMFS checkword (expected 0, got 0x%.6X)\n",check);
      close(disk_fd);
      return(-1);
    }
    if(fs_size > band_size){
      printf("lmfuse: LMFS size exceeds partition size (0x%.6X vs 0x%.6X)\n",
	     fs_size,band_size);
      close(disk_fd);
      return(-1);
    }
    if(fs_size < band_size){
      printf("lmfuse: LMFS size smaller than partition size (0x%.6X vs 0x%.6X)\n",
	     fs_size,band_size);
      close(disk_fd);
      return(-1);
    }
    printf("LMFS: PUT at block %.6X, size %.6X\n",put_base,put_size);
    if(root_nblks > 31){
      printf("lmfuse: Unexpected root directory map size (expected <= 31, got %d)\n",root_nblks);
      close(disk_fd);
      return(-1);      
    }
    root_base = get24(17);
    // root_base += band_block;
    root_size = get24(20); // Bits!
    root_size /= 8;
    printf("LMFS: ROOT DIRECTORY at block %.6X, size %d\n",root_base,root_size);
    // Initial checks done here. It's unexpected that the PUT or root directory move during operation.    
  }
  // Should be all ready to rock!  
  x = fuse_main(args.argc, args.argv, &lmfs_oper, NULL);
  // Done, clean up
  // free(ROOT_DIRECTORY);
  close(disk_fd);
  return(x);
}
