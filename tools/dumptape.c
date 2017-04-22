#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int tape_fd = -1;            // FD for tape
int out_fd = -1;             // FD for output
int tape_eot = 0;            // At end of tape
int tape_fm = 0;             // At filemark
int tape_reclen = 0;         // Tape record length
int tape_error = 0;          // Tapemaster error code
char tape_block[64*1024];    // Buffer

char tfname[512];
char ofname[512];
int ofnumber = 0;

int tape_read(){
  int reclen;
  ssize_t rv=0;
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
  // printf("TAPE READ: Reclen %d\n",reclen);
  tape_reclen = reclen;
  if(reclen == 0){
    // Found file mark
    tape_fm = 1;
    // tape_error = 0x15; // Unexpected file mark
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
    printf("tape:reclen mismatch: %d vs %d\n",tape_reclen,reclen);
    close(tape_fd);
    if(out_fd != -1){
      close(out_fd);
    }
    exit(-1);
    return(tape_reclen);
  }
  return(reclen);
}

int main(int argc, char *argv[]){
  // Handle command-line options
  if(argc != 2){
    printf("Usage: dumptape (tape file name)\n");
    return(0);
  }else{
    strncpy(tfname,argv[1],512);
    printf("Dumping %s\n",tfname);
  }

  tape_fd = open(tfname,O_RDONLY);
  if(tape_fd < 0){
    perror("Tape:open");
    return(-1);
  }
  
  while(tape_eot != 1){
    tape_read();
    if(tape_reclen == 0){
      if(out_fd != -1){
	printf("EOF/EOT\n");
	close(out_fd);
	out_fd = -1;
	ofnumber++;
      }
    }else{
      // We have a block
      ssize_t rv;
      if(out_fd == -1){
	sprintf(ofname,"file-%d.bin",ofnumber);
	out_fd = open(ofname,O_RDWR|O_CREAT,0660);
	if(out_fd < 0){
	  perror("Output:open");
	  close(tape_fd);
	  return(-1);
	}
	printf("Writing %s (blocksize %d)...\n",ofname,tape_reclen);
      }
      // Write it
      rv = write(out_fd,tape_block,tape_reclen);
      if(rv != tape_reclen){
	  perror("Output:write");
	  close(tape_fd);
	  close(out_fd);	  
	  return(-1);	
      }
    }
  }
  
  return(0);
}
