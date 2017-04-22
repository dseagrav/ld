#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int tape_fd = -1;            // FD for tape
int in_fd = -1;              // FD for output
int in_reclen = -1;          // Input record length
int zero_reclen = 0;         // For writing EOF/EOT
char tape_block[64*1024];    // Buffer

char tfname[512];
char ifname[512];

int main(int argc, char *argv[]){
  // Handle command-line options
  if(argc < 4 || ((argc&0x01) != 0x00)){
    printf("Usage: maketape (tape file name) [(input file name) (block size)]...\n");
    return(0);
  }else{
    int ifnumber = 0;
    ssize_t rv = 0;
    strncpy(tfname,argv[1],512);
    printf("Creating tape %s\n",tfname);
    tape_fd = open(tfname,O_RDWR|O_CREAT,0660);
    if(tape_fd < 0){
      perror("Tape:open");
      return(-1);
    }
    while(2+(ifnumber*2) < argc){
      int eof = 0;
      strncpy(ifname,argv[2+(ifnumber*2)],512);
      in_reclen = atoi(argv[3+(ifnumber*2)]);
      if(in_reclen > 0){
	printf("Adding %s with blocksize %d\n",ifname,in_reclen);
	in_fd = open(ifname,O_RDONLY);
	if(in_fd < 0){
	  perror("Input:open");
	  close(tape_fd);
	  return(-1);
	}
      }else{
	printf("Writing extra EOF\n");
	eof = 1;
      }
      while(eof == 0){
	rv = read(in_fd,tape_block,in_reclen);
	if(rv == -1){
	  perror("Input:read");
	  return(0);
	}
	if(rv == 0){
	  // Found EOF, no block
	  eof = 1;
	}else{
	  if(rv < in_reclen){
	    // Found EOF, short block
	    printf("Short read: Zeroing %d bytes after end of file\n",
		   in_reclen-rv);
	    bzero(tape_block+rv,in_reclen-rv);
	    eof = 1;
	  }
	  // Write block
	  rv = write(tape_fd,(unsigned char *)&in_reclen,4);
	  if(rv != 4){
	    perror("Output:write");
	    close(tape_fd);
	    close(in_fd);	  
	    return(-1);	
	  }
	  rv = write(tape_fd,tape_block,in_reclen);
	  if(rv != in_reclen){
	    perror("Output:write");
	    close(tape_fd);
	    close(in_fd);	  
	    return(-1);	
	  }
	  rv = write(tape_fd,(unsigned char *)&in_reclen,4);	  
	  if(rv != 4){
	    perror("Output:write");
	    close(tape_fd);
	    close(in_fd);	  
	    return(-1);	
	  }
	}
      }
      // EOF
      write(tape_fd,(unsigned char *)&zero_reclen,4);            
      ifnumber++;
    }
    printf("Writing EOT...\n");
    rv = write(tape_fd,(unsigned char *)&zero_reclen,4);            
    if(rv != 4){
      perror("Output:write");
      close(tape_fd);
      close(in_fd);	  
      return(-1);	
    }
    rv = write(tape_fd,(unsigned char *)&zero_reclen,4);            
    if(rv != 4){
      perror("Output:write");
      close(tape_fd);
      close(in_fd);	  
      return(-1);	
    }
    rv = write(tape_fd,(unsigned char *)&zero_reclen,4);            
    if(rv != 4){
      perror("Output:write");
      close(tape_fd);
      close(in_fd);	  
      return(-1);	
    }
    rv = write(tape_fd,(unsigned char *)&zero_reclen,4);            
    if(rv != 4){
      perror("Output:write");
      close(tape_fd);
      close(in_fd);	  
      return(-1);	
    }
    printf("Done!\n");
  }
  // Done
  return(0);
}
