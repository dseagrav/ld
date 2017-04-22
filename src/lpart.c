/* Copyright 2016-2017 
   Daniel Seagraves <dseagrav@lunar-tokyo.net>
   Barry Silverman <barry@disus.com>
   Brad Parker <brad@heeltoe.com>

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
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#define E_BLKSIZE	1024

uint8_t block[E_BLKSIZE];
int fd;
char *filename;
int mark_expt_flag;
int show_flag;
int extract_flag;
int load_flag;
int active_flag;
int deactive_flag;
int debug_flag;
int devblock;

typedef unsigned long uint32;

#if 0
/* virtual format */
struct {
	uint32 rev;
	uint32 flags;
	char dev[32];
	int bytes_per_block;
	int bytes_per_sector;
	int sectors_per_track;
	int reads;
	int cyls;
	int reserved_sectors;
};
#endif

#define E_PART_MAX	32
struct {
	char name[5];
	int start_block;
	int length_blocks;
	char comment[32];
} __attribute__((packed)) part_table[E_PART_MAX];
int part_table_size;

/* raw, actual-bytes format */
struct raw_volume_label_s {
	uint8_t vl_labl[4];
	uint8_t vl_rev[4];
	uint8_t vl_cyls[4];
	uint8_t vl_heads[4]; 
	uint8_t vl_sectors_per_track[4];
	uint8_t vl_block_per_cyl[4];
	uint8_t vl_microload[4];
	uint8_t vl_load_band[4];
	uint8_t vl_disk_name[32];
	uint8_t vl_machine[32];
	uint8_t vl_comment[96];
} __attribute__((packed));

struct part_table_s {
	uint8_t pt_num_entries[4];
	uint8_t pt_size_entry[4];
	uint8_t pt_offset[4];
	uint8_t pt_res[40];
} __attribute__((packed));

struct part_table_entry_s {
	uint8_t pte_name[4];
	uint8_t pte_start_block[4];
	uint8_t pte_length_blocks[4];
	uint8_t pte_comment[32];
} __attribute__((packed));

char *part_gen_func_type[] = {
	/* 00 */ "load band",
	/* 01 */ "microload band",
	/* 02 */ "page band",
	/* 03 */ "file band",
	/* 04 */ "meter band",
	/* 05 */ "test zone band",
	/* 06 */ "format parameter band",
	/* 07 */ "volume label",
	/* 08 */ "save band",
	/* 09 */ "partition band",
	/* 0A */ "configuration band",
	/* 0B */ "user defined type #1",
	/* 0C */ "user defined type #2",
	/* 0D */ "user defined type #3",
	/* 0E */ "user defined type #4",
	/* 0F */ "user defined type #5",
	/* 10 */ "user defined type #6",
	/* 11 */ "user defined type #7",
	/* 12 */ "user defined type #8",
	/* 13 */ "user defined type #9",
	/* 14 */ "user defined type #10",
	"reserved"
};


/*
Volume Label
Partition Table
Save Partition
Test Zone
Format Information partition
*/

int
open_file(char *filename, int need_rw)
{
	if (debug_flag) printf("open '%s'\n", filename);
	fd = open(filename, need_rw ? O_RDWR : O_RDONLY);
	if (fd < 0) {
		perror(filename);
		return -1;
	}

	return 0;
}

int
read_block(int blknum)
{
	int ret;

	off_t offset = blknum * 1024;

	ret = lseek(fd, offset, SEEK_SET);
	if (ret < 0)
		return 1;
	
	ret = read(fd, block, E_BLKSIZE);
	if (ret != E_BLKSIZE)
		return -1;

	return 0;
}

int
write_block(int blknum)
{
	int ret;
	off_t offset = blknum * 1024;

	ret = lseek(fd, offset, SEEK_SET);
	if (ret < 0)
		return 1;
	
	ret = write(fd, block, E_BLKSIZE);
	if (ret != E_BLKSIZE)
		return -1;

	return 0;
}

unsigned int
int4(uint8_t *p)
{
	return (p[3] << 24) | (p[2] << 16) | (p[1] << 8) | (p[0]);
}

unsigned int
int2(uint8_t *p)
{
	return (p[1] << 8) | (p[0]);
}

void
swap4(uint8_t *s, uint8_t *d)
{
	d[0] = s[2];
	d[1] = s[3];
	d[2] = s[0];
	d[3] = s[1];
}

int ptblknum;

int
read_part_table(char *filename)
{
	struct raw_volume_label_s *vl;
	struct part_table_s *pt;
	struct part_table_entry_s *pe;
	int i, count, size_in_words, index;
	int rev, bytes_per_block;

	if (read_block(0))
		return -1;

	vl = (struct raw_volume_label_s *)block;
	if (vl->vl_labl[0] == 'D' && vl->vl_labl[1] == 'A' &&
	    vl->vl_labl[2] == 'T' && vl->vl_labl[3] == 'A')
	{
		printf("%s: DATA disk, no LABL partition\n", filename);
		return -1;
	}

	if (vl->vl_labl[0] != 'L' || vl->vl_labl[1] != 'A' ||
	    vl->vl_labl[2] != 'B' || vl->vl_labl[3] != 'L')
	{
		printf("%s: missing LABL signature\n", filename);

		if (debug_flag) printf("found %c%c%c%c\n",
				       vl->vl_labl[0], vl->vl_labl[1], 
				       vl->vl_labl[2], vl->vl_labl[3]);
		return -1;
	}

	rev = int4(vl->vl_rev);
	printf("rev: %08x\n", rev);
	printf("Disk: '%s'\n", vl->vl_disk_name);

	printf("Machine: '%s'\n", vl->vl_machine);

	bytes_per_block = E_BLKSIZE;
	printf("bytes/block: %d\n", bytes_per_block);

	printf("comment: '%s'\n", vl->vl_comment);
	
	pt = (struct part_table_s *)(&block[0x200]);
	count = int4(pt->pt_num_entries);
	size_in_words = int4(pt->pt_size_entry);
	index = 0;

	printf("ptable count: %d\n", count);
	printf("size (words): %d\n", size_in_words);

	part_table_size = count;

	pe = (struct part_table_entry_s *)(&pt->pt_offset);
	for (i = 0; i < count; i++) {
		int user_type, property;

		memcpy(part_table[index].name,
		       pe->pte_name, 4);
		part_table[index].name[4] = 0;

		part_table[index].start_block = int4(pe->pte_start_block);
		part_table[index].length_blocks = int4(pe->pte_length_blocks);
		memset(part_table[index].comment, 0,
		       sizeof(part_table[index].comment));

		memcpy(part_table[index].comment, 
		       pe->pte_comment, sizeof(pe->pte_comment));

		printf("%-4s %6d/%6d '%s' ",
		       part_table[index].name,
		       part_table[index].start_block,
		       part_table[index].length_blocks,
		       part_table[index].comment);

		printf("\n");
		index++;
		pe = (struct part_table_entry_s *)
			(((uint8_t *)pe) + (size_in_words * sizeof(uint32_t)));
	}

	return 0;
}

int
show_part_table(void)
{
	return 0;
}

int
extract_part(char *partition)
{
	int i, index, blocks, fd2, bpblk, ret;
	off_t offset;

	index = -1;
	for (i = 0; i < part_table_size; i++) {
		if (strcmp(partition, part_table[i].name) == 0) {
			index = i;
			break;
		}
	}

	if (index < 0) {
		printf("can't find partition '%s'\n", partition);
		return -1;
	}

	bpblk = 1024;

	offset = part_table[i].start_block * bpblk;
	lseek(fd, offset, SEEK_SET);

	blocks = part_table[i].length_blocks;

	unlink(partition);
	fd2 = open(partition, O_WRONLY | O_CREAT, 0666);
	if (fd2 == 0) {
		perror(partition);
		return -1;
	}

	for (i = 0; i < blocks; i++) {
		ret = read(fd, block, bpblk);
		if (ret != bpblk) {
			printf("short read?\n");
			break;
		}

		write(fd2, block, bpblk);
	}

	close(fd2);

	return 0;
}

int
load_part(char *partition)
{
	int i, j, index, blocks, fd2, swap;
	uint8_t sblock[E_BLKSIZE];
	off_t offset;
	size_t bpblk;
	ssize_t ret;

	index = -1;
	for (i = 0; i < part_table_size; i++) {
		if (strcmp(partition, part_table[i].name) == 0) {
			index = i;
			break;
		}
	}

	if (index < 0) {
		printf("can't find partition '%s'\n", partition);
		return -1;
	}

	bpblk = 1024;

	offset = part_table[i].start_block * bpblk;
	lseek(fd, offset, SEEK_SET);

	blocks = part_table[i].length_blocks;

	fd2 = open(partition, O_RDONLY);
	if (fd2 == 0) {
		perror(partition);
		return -1;
	}

	for (i = 0; i < blocks; i++) {
		ret = read(fd2, block, bpblk);

		ret = write(fd, block, ret);
		if (ret != bpblk) {
			break;
		}
	}

	close(fd2);

	return 0;
}

int
active_part(char *partition, int onoff)
{
#if 0
	struct part_table_s *pt;
	struct part_table_entry_s *pe;
	int i, count, size_in_words;

	if (read_block(ptblknum))
		return -1;

	printf("\nset/clean active\n");

	pt = (struct part_table_s *)block;
	pe = (struct part_table_entry_s *)(pt+1);

	count = int4(pt->pt_num_entries);
	size_in_words = int4(pt->pt_size_entry);

	for (i = 0; i < count; i++) {
		if (strcmp(part_table[i].name, partition) == 0) {

			if (onoff) {
				pe->pte_attributes[3] |= 0x04;
				printf("setting %s\n", part_table[i].name);
			} else {
				pe->pte_attributes[3] &= ~0x04;
				printf("clearing %s\n", part_table[i].name);
			}
		}
			
		pe = (struct part_table_entry_s *)
			(((uint8_t *)pe) + (size_in_words * sizeof(uint32_t)));
	}

	if (write_block(ptblknum))
		return -1;
#endif
	return 0;
}

int
mark_expt(void)
{
#if 0
	struct part_table_s *pt;
	struct part_table_entry_s *pe;
	int i, count, size_in_words;

	if (read_block(ptblknum))
		return -1;

	printf("\ntrying to mark expts\n");

	pt = (struct part_table_s *)block;
	pe = (struct part_table_entry_s *)(pt+1);

	count = int4(pt->pt_num_entries);
	size_in_words = int4(pt->pt_size_entry);

	for (i = 0; i < count; i++) {
		if (strcmp(part_table[i].name, "EXPT") == 0) {

			if ((pe->pte_attributes[3] & 0x02) == 0) {
				pe->pte_attributes[3] |= 0x02;
				printf("marking %s\n", part_table[i].name);
			}
		}
			
		pe = (struct part_table_entry_s *)
			(((uint32 *)pe) + size_in_words);
	}

	if (write_block(ptblknum))
		return -1;
#endif
	return 0;
}

void
usage(void)
{
	fprintf(stderr, "usage:\n");
	fprintf(stderr, "-d		debug\n");
	fprintf(stderr, "-e <filename>	extract partition\n");
	fprintf(stderr, "-l <filename>	load partition\n");
	fprintf(stderr, "-A <partname>  set active flag\n");
	fprintf(stderr, "-D <partname>  clear active flag\n");
	fprintf(stderr, "-m <partname>	mark expt parition\n");
	exit(1);
}


extern char *optarg;
extern int optind;

main(int argc, char *argv[])
{
	int c, need_write;
	char *partition;

	filename = "disk.img";

	show_flag = 1;
	devblock = 0;

	while ((c = getopt(argc, argv, "de:l:mA:D:s:")) != -1) {
		switch (c) {
		case 'd':
			debug_flag++;
			break;
		case 'l':
			load_flag++;
			partition = strdup(optarg);
			break;
		case 'e':
			extract_flag++;
			partition = strdup(optarg);
			break;
		case 'A':
			active_flag++;
			partition = strdup(optarg);
			break;
		case 'D':
			deactive_flag++;
			partition = strdup(optarg);
			break;
		case 'm':
			mark_expt_flag++;
			break;
		default:
			usage();
		}
	}

	if (argc > optind)
		filename = argv[optind];

	need_write = mark_expt_flag || active_flag || deactive_flag || load_flag;

	if (open_file(filename, need_write) ||
	    read_part_table(filename))
	{
		exit(1);
	}

	if (show_flag) {
		show_part_table();
	}

	if (mark_expt_flag) {
		mark_expt();
	}

	if (deactive_flag) {
		active_part(partition, 0);
	}

	if (active_flag) {
		active_part(partition, 1);
	}

	if (load_flag) {
		load_part(partition);
	}

	if (extract_flag) {
		extract_part(partition);
	}

	exit(0);
}
