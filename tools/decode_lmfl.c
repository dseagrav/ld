/* Copyright 2015-2019
   Daniel Seagraves <dseagrav@lunar-tokyo.net>

   Decode LMFL, Version 16
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

/*
   This expects to be run at the top of a directory tree containing a series
   of files to be extracted. The program will walk the tree and extract
   what it finds to a directory one level below CWD named "ext".
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <dirent.h>
#include <string.h>
#include <errno.h>
#include <utime.h>
#include <sys/types.h>
#include <sys/stat.h>

// Tracelog FH
FILE *logfd;
char logent[512];

// Directory timestamp list
#define MAXDIR 5000
struct dlistent {
  char ufn[512];
  long ts;
};
struct dlistent dlist[MAXDIR];
int topdir=0;

// File directory list
#define MAXFILE 15000
struct filent {
  char ufn[512];
  int version;
};
struct filent fdir[MAXFILE];
int topfile=0;

// File counter for defaulting name
int filecounter=0;

// Check existence of output path, create if needed
void checkpath(char *path,long ts){
  struct stat st;
  int res = stat(path,&st);
  int x=0;
  // Update directory list
  while(x < topdir){
    if(strcmp(dlist[x].ufn,path) == 0){
      break;
    }
    x++;
  }
  if(x == topdir){
    // New dir
    strncpy(dlist[x].ufn,path,512);
    dlist[x].ts = ts;
    topdir++;
  }else{
    if(dlist[x].ts < ts){
      // "Older" dir
      dlist[x].ts = ts;
    }
  }
  if(res != 0){
    // Is it something other than nonexistent?
    if(errno != ENOENT){
      perror("stat()");
      exit(-1);
    }
    // Doesn't exist, create it
    res = mkdir(path,0777);
    if(res != 0){
      perror("mkdir()");
      exit(-1);
    }
  }else{
    // It exists, we should check for directory-ness but I am supremely lazy.
  }
}

// Process the file
int process(char *name){
  FILE *fd,*ofd,*of2;
  unsigned char block[514];
  char *headerword;
  size_t inb;
  // Header information
  char opath[512];    // Original path
  char oname[64];     // Original name
  char otype[64];     // Original type
  char over[64];      // Original version
  int ovn;            // Original version (number)
  long octs;          // Original creation timestamp
  int obytes,oblocks; // Original file size in bytes/blocks
  int obsize;         // Original Byte Size
  int xbytes,xblocks; // Bytes/blocks xferd
  int charmode = 0;   // Character translation mode
  char ufn[512];      // Unix path and filename
  char uf2[512];      // Unix path and filename for of2
  int blksize = 514;  // Read block size
  int x=0;            // Scratch
  struct utimbuf utb; // utime() buffer

  of2 = NULL;         // Clobber of2
  fd = fopen(name,"r");
  if(fd == NULL){
    perror("fopen()");
    return(-1);
  }
  // There's only one header to a file. It's in the first block.
  // The LMI files were padded by two bytes every 512 due to an FTP transfer screw-up. The extra bytes should be CR/LF.
  // Oh, but some files are different!
  inb = fread(block,1,blksize,fd);
  if(inb < blksize){
    if(feof(fd)){
      printf("Unexpected EOF: Expected %d bytes, got %d\n",blksize,(int)inb);
      if(inb == 0){
	printf("Skipping...\n");
	fclose(fd);
	return(0);
      }
    }else{
      perror("fread()");
    }
    return(-1);
  }
  // Check for block tail
  if(block[512] != 0x0D && block[513] != 0x0A){
    // Bad tail?
    if(block[512] == 0x0A && block[513] == 0x00){
      // This is LF padding!
      blksize = 513;      
      fseek(fd,-1,SEEK_CUR); // Back up one byte
    }else{
      if(block[512] == 0x00 && block[513] == 0x00){
	// No padding!
	blksize = 512;
	fseek(fd,-2,SEEK_CUR); // Back up two bytes
      }else{
	printf("Unexpected tail, expected 0D 0A or 0A 00, got %X %X\n",block[512],block[513]);
	return(-1);
      }
    }
  }
  // Check for existence of header
  if(strncmp(block,"LMFL",4) != 0){
    printf("Unexpected header, expected 4C 4D 46 4C, got %X %X %X %X\n",block[0],block[1],block[2],block[3]);
    printf("Skipping...\n");
    fclose(fd);
    return(0);
  }
  // We have a valid header. Process it.
  // Make log entry...
  fprintf(logfd,"%s (%d-byte blocks)\n%s\n",name,blksize,block);
  /* 
     A header looks like these:
     LMFL(:DEVICE "DSK" :DIRECTORY ("L" "NETWORK" "SERIAL-IP") :NAME "SERIAL-PROTOCOL" :TYPE "LISP" :VERSION 42 :CHARACTERS T :CREATION-DATE 2786122897 :AUTHOR "keith" :LENGTH-IN-BYTES 17028 :LENGTH-IN-BLOCKS 17 :BYTE-SIZE 8)
     LMFL(:DEVICE "DSK" :DIRECTORY ("L" "IO" "FILE") :NAME "ACCESS" :TYPE "QFASL" :VERSION 39 :CREATION-DATE 2803576763 :AUTHOR "keith" :LENGTH-IN-BYTES 28345 :LENGTH-IN-BLOCKS 56 :BYTE-SIZE 16)
     LMFL(:DEVICE "DSK" :DIRECTORY ("L" "IO" "FILE") :NAME "BALDIR" :TYPE "LISP" :VERSION 118 :DONT-DELETE T :CHARACTERS T :CREATION-DATE 2747449977 :AUTHOR "pld" :LENGTH-IN-BYTES 28438 :LENGTH-IN-BLOCKS 28 :BYTE-SIZE 8)
     The header is null-terminated in the block.
   */
  // Default file name, type, and version
  filecounter++;
  sprintf(oname,"FILE%d",filecounter);
  strncpy(otype,"BIN",64);
  strncpy(over,"1",64);
  // Now parse header
  headerword = strtok(block+5," ");
  while(headerword != NULL){
    int found = 0;
    if(strcmp(headerword,":DEVICE") == 0){
      // Device name
      headerword = strtok(NULL,"\" ");
      // printf("DEVICE: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":DIRECTORY") == 0){
      // Directory path. This is NOT ALWAYS a list!
      if(headerword[11] == '('){
	// List
	headerword = strtok(NULL,"()");
      }else{
	// String
	headerword = strtok(NULL," ");	
      }
      printf("DIRECTORY: %s\n",headerword);
      strncpy(opath,headerword,512);
      found = 1;
    }
    if(strcmp(headerword,":NAME") == 0){
      // File name. These can have embedded spaces.
      headerword = strtok(NULL,"\"");
      printf("NAME: %s\n",headerword);
      strncpy(oname,headerword,64);
      found = 1;
    }
    if(strcmp(headerword,":TYPE") == 0){
      // File type. (the extension)
      headerword = strtok(NULL,"\" ");
      printf("TYPE: %s\n",headerword);
      strncpy(otype,headerword,64);
      found = 1;
    }
    if(strcmp(headerword,":VERSION") == 0){
      // Version number
      headerword = strtok(NULL,"\" ");
      printf("VERSION: %s\n",headerword);
      strncpy(over,headerword,64);
      ovn = atoi(over);
      found = 1;
    }
    if(strcmp(headerword,":CHARACTERS") == 0){
      // Indicates text-ness?
      headerword = strtok(NULL,"\" ");
      printf("CHARACTERS: %s\n",headerword);
      charmode = 1;
      found = 1;
    }
    if(strcmp(headerword,":CREATION-DATE") == 0){
      // File creation date
      headerword = strtok(NULL,"\" ");
      // printf("CREATION-DATE: %s\n",headerword);
      octs = atol(headerword); // Get lisp timestamp
      octs -= 2208988800;      // Convert to unix timestamp
      found = 1;
    }
    if(strcmp(headerword,":AUTHOR") == 0){
      // Author
      headerword = strtok(NULL,"\" ");
      // printf("AUTHOR: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":SIZE") == 0){
      // From the dist tape. Size in 512-byte blocks.
      // Size will be doubled by byte-size modifier later.
      headerword = strtok(NULL,"\" ");
      printf("SIZE: %s\n",headerword);
      obytes = atoi(headerword);
      oblocks = obytes; 
      obytes *= 512;
      found = 1;      
    }
    if(strcmp(headerword,":LENGTH-IN-BYTES") == 0){
      // Length in bytes
      headerword = strtok(NULL,"\" ");
      printf("LENGTH-IN-BYTES: %s\n",headerword);
      obytes = atoi(headerword);
      found = 1;
    }
    if(strcmp(headerword,":LENGTH-IN-BLOCKS") == 0){
      // Length in blocks
      headerword = strtok(NULL,"\" ");
      printf("LENGTH-IN-BLOCKS: %s\n",headerword);
      oblocks = atoi(headerword);
      found = 1;
    }
    if(strcmp(headerword,":BYTE-SIZE") == 0){
      // Size of a byte
      headerword = strtok(NULL,"\") ");
      printf("BYTE-SIZE: %s\n",headerword);
      obsize = atoi(headerword);
      if(obsize == 16){
	obytes *= 2;
      }
      if(obsize != 8 && obsize != 16){
	printf("Unsupported byte size %s\n",headerword);
	return(-1);	
      }
      found = 1;
    }
    if(strcmp(headerword,":PROTECTION") == 0){
      // Numeric protection mask
      headerword = strtok(NULL,"\" ");
      // printf("PROTECTION: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":GENERATION-RETENTION-COUNT") == 0 || strcmp(headerword,":GENERATION-RETENTATION-COUNT") == 0){
      // How many versions of this file to keep
      headerword = strtok(NULL,"\" ");
      // printf("GENERATION-RETENTION-COUNT: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":BLOCK-SIZE") == 0){
      // File block size, nothing to do with tape.
      headerword = strtok(NULL,"\" ");
      // printf("BLOCK-SIZE: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":REFERENCE-DATE") == 0){
      // ???
      headerword = strtok(NULL,"\" ");
      // printf("REFERENCE-DATE: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":ACCOUNT") == 0){
      // Disk usage accounting?
      // Some wiseacre used this to put cute strings in the directory list.
      // :ACCOUNT "/^V/"W/^Vi/^Vn/^Vn/^Vi/^Vn/^Vg/^V A/^Vw/^Va/^Vy/^V!/^V/"" :C
      headerword = strtok(NULL,"\" ");
      // printf("ACCOUNT: %s\n",headerword);
      // Try to unscrew up that mess. We don't NEED the value so there's no point to dealing with all the escaping crud.
      // Obtain next token
      headerword = strtok(NULL," ");
      // If it's not the next keyword and we haven't hit the end of the string, try again
      while(headerword != NULL && headerword[0] != ':'){
	headerword = strtok(NULL," ");
      }
      continue;
    }
    if(strcmp(headerword,":DONT-DELETE") == 0){
      // Delete prevention flag?
      headerword = strtok(NULL,"\" ");
      // printf("DONT-DELETE: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":T") == 0){
      // No idea what this is
      headerword = strtok(NULL,"\" ");
      // printf("T: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":PHYSICAL-VOLUME-FREE-BLOCKS") == 0){
      // Why is this here?
      headerword = strtok(NULL,"\" ");
      // printf("PHYSICAL-VOLUME-FREE-BLOCKS: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":QFASLP") == 0){
      // QFASL-ness?
      headerword = strtok(NULL,"\" ");
      // printf("QFASLP: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":BACKUP-DATE") == 0){
      headerword = strtok(NULL,"\" ");
      // printf("BACKUP-DATE: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":DONT-SUPERSEDE") == 0){
      headerword = strtok(NULL,"\" ");
      // printf("DONT-SUPERSEDE: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":LMI-TID") == 0){
      headerword = strtok(NULL,"\" ");
      // printf("LMI-TID: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":DONT-REAP") == 0){
      // Did this come from ITS?
      headerword = strtok(NULL,"\" ");
      // printf("DONT-REAP: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":NOT-BACKED-UP") == 0){
      // It is now!
      headerword = strtok(NULL,"\" ");
      // printf("NOT-BACKED-UP: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":TEMPORARY") == 0){
      headerword = strtok(NULL,"\" ");
      // printf("TEMPORARY: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":COMMENT") == 0){
      // Partition comment
      headerword = strtok(NULL,"\"");
      // strncpy(oname,headerword,64);
      // printf("COMMENT: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":PARTITION") == 0){
      // Indicates this is a disk partition and not a file
      headerword = strtok(NULL,"\" ");
      strncpy(otype,"PART",64);
      strncpy(over,"1",64);
      ovn = 1;
      // printf("COMMENT: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":TAPE-NUMBER") == 0){
      headerword = strtok(NULL,"\" ");
      // printf("TAPE-NUMBER: %s\n",headerword);
      found = 1;
    }
    if(strcmp(headerword,":TAPE-ID") == 0){
      headerword = strtok(NULL,"\"");
      // printf("TAPE-ID: %s\n",headerword);
      // strncpy(oname,headerword,64);
      found = 1;
    }
    if(strcmp(headerword,")") == 0){
      // End of the header
      found = 1;
    }
    /*
    if(strcmp(headerword,":X") == 0){
      // 
      headerword = strtok(NULL,"\" ");
      printf(": %s\n",headerword);
      found = 1;
    }
    */
    if(found == 1){
      headerword = strtok(NULL," ");
    }else{
      printf("Unknown header keyword %s\n",headerword);
      return(-1);
    }
  }
  // Check for mode screw
  if(strncmp(otype,"LISP",4) == 0 && charmode != 1){
    printf("Filetype LISP without charmode: Forcing charmode\n");
    charmode = 1;
  }
  // The header is read in. Now build the output path and filename,
  // creating directories (if necessary) as we go
  strcpy(ufn,"../ext/");
  headerword = strtok(opath,"\"");
  while(headerword != NULL){
    x=0;
    while(x < strlen(headerword)){
      headerword[x] = tolower(headerword[x]);
      x++;
    }
    strcat(ufn,headerword);
    checkpath(ufn,octs);
    strcat(ufn,"/");
    headerword = strtok(NULL,"\" ");
  }  
  // Convert name and type to lowercase
  x=0;
  while(x < strlen(oname)){
    oname[x] = tolower(oname[x]);
    x++;
  }
  x=0;
  while(x < strlen(otype)){
    otype[x] = tolower(otype[x]);
    x++;
  }
  // Append filename, type
  strcat(ufn,oname);
  strcat(ufn,".");
  strcat(ufn,otype);
  // At this point, check if we already wrote this file
  x = 0;
  while(x < topfile){
    if(strcmp(fdir[x].ufn,ufn) == 0){
      break;
    }
    x++;
  }
  if(x == topfile){
    // This is a new file.
    strncpy(fdir[x].ufn,ufn,512);
    fdir[x].version = ovn;
    topfile++;
    strncpy(uf2,ufn,512);
    of2 = fopen(uf2,"w");
    if(of2 == NULL){
      perror("fopen()");
      exit(-1);
    }
  }else{
    if(fdir[x].version < ovn){
      // This is a newer file
      fdir[x].version = ovn;
      strncpy(uf2,ufn,512);
      of2 = fopen(uf2,"w");
      if(of2 == NULL){
	perror("fopen()");
	exit(-1);
      }
    }
  }
  // Append version number
  strcat(ufn,".");
  strcat(ufn,over);
  printf("OFN: %s\n",ufn);
  fprintf(logfd," -> %s\n\n",ufn);
  // Obtain output FD
  ofd = fopen(ufn,"w");
  if(ofd == NULL){
    perror("fopen()");
    exit(-1);
  }
  xbytes = 0; xblocks=1;
  while(xbytes < obytes){
    size_t outb,xsize;
    // Obtain input
    inb = fread(block,1,blksize,fd);
    if(inb < blksize){
      if(feof(fd)){
	printf("Unexpected EOF: Expected %d bytes, got %d\n",blksize,(int)inb);
      }else{
	perror("fread()");
      }
      return(-1);
    }
    xblocks++;
    // Data starts 8 blocks in?
    if(xblocks < 9){ continue; }
    // Generate output
    if(xbytes+512 > obytes){
      xsize = obytes-xbytes;
    }else{
      xsize = 512;
    }
    // Charmode translation
    if(charmode == 1){
      x=0;
      while(x < xsize){
	if((block[x]&0x80) != 0){
	  // Just unset the high bit?
	  block[x] &= 0x7F;
	  // CR becomes LF
	  if(block[x] == 0x0D){ block[x] = 0x0A; }	  
	}
	x++;
      }
    }
    // printf("BLOCK %d: XByte %d OByte %d (Xsize %d)\n",xblocks,xbytes,obytes,(int)xsize);
    outb = fwrite(block,1,xsize,ofd);
    if(of2 != NULL){
      outb = fwrite(block,1,xsize,of2);
    }
    xbytes += xsize;
  }
  // Fill out times
  utb.actime = octs;
  utb.modtime = octs;
  // All done. Close files and set times.
  fclose(ofd);
  if(octs > 0){ 
    utime(ufn,&utb);
    if(of2 != NULL){
      fclose(of2); 
      utime(uf2,&utb);
    }
  }
  // Next file!
  fclose(fd);
  return(0);
}

// Walk a directory looking for files we want
void walkdir(DIR *d,char *path){
  char nextpath[1024];
  DIR *nextdir = NULL;
  struct dirent *ent;
  while(ent = readdir(d)){
    // Is this a directory? (linux/bsd specific)
    if(ent->d_type == DT_DIR){
      // If it starts with a dot, disregard it.
      if(ent->d_name[0] == '.'){ continue; }
      // Walk it
      sprintf(nextpath,"%s/%s",path,ent->d_name);
      nextdir = opendir(nextpath);
      if(nextdir == NULL){
	perror("opendir()");
	exit(-1);
      }
      walkdir(nextdir,nextpath);
      closedir(nextdir);
    }else{
      // It's probably a normal file. Does the name end in .DAT?
      char ext[5];
      strncpy(ext,ent->d_name+(strlen(ent->d_name)-4),4);
      if(strcasecmp(ext,".DAT")==0){
	// Yes, this is what we want
	sprintf(nextpath,"%s/%s",path,ent->d_name);
	if(process(nextpath) < 0){
	  printf("Failed to process file %s\n",nextpath);
	  exit(-1);
	}
      }
    }
  }
}

// Main execution
int main(){
  int x=0;
  struct utimbuf utb; // utime() buffer

  // Create output path if it doesn't exist
  checkpath("../ext",0);
  // Create log file
  logfd = fopen("../ext/decode_lmfl.log","w");
  if(logfd == NULL){
    perror("fopen()");
    exit(-1);
  }
  // Open directory
  DIR *d = opendir(".");
  if(d == NULL){
    perror("opendir()");
    return(-1);
  }
  // These boots were made fer walkin'...
  walkdir(d,".");
  // All done!
  closedir(d);
  // Write back directory times
  while(x < topdir){
    if(dlist[x].ts > 0){
      utb.actime = utb.modtime = dlist[x].ts;    
      utime(dlist[x].ufn,&utb);
    }
    x++;
  }
  // Finished
  fclose(logfd);
  return(0);
}
