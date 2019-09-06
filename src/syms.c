/* Copyright 2016-2017
   Daniel Seagraves <dseagrav@lunar-tokyo.net>
   Barry Silverman <barry@disus.com>
   Greg Gilley

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
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

struct sym_s {
	struct sym_s *next;
	char *name;
	unsigned int v;
	int mtype;
};

struct symtab_s {
	char *name;
	struct sym_s *syms;
	int sym_count;
	struct sym_s **sorted_syms;
};


struct symtab_s sym_prom;
struct symtab_s sym_mcr;


static int
_sym_add(struct symtab_s *tab, int memory, char *name, int v)
{
	struct sym_s *s;

	if (0) printf("%d %s %o\n", memory, name, v);

	s = (struct sym_s *)malloc(sizeof(struct sym_s));
	if (s) {
		tab->sym_count++;

		s->name = strdup(name);
		s->v = v;
		s->mtype = memory;

		s->next = tab->syms;
		tab->syms = s;
	}

	return 0;
}

static char *
_sym_find_by_val(struct symtab_s *tab, int memory, unsigned int v)
{
	struct sym_s *s;

	for (s = tab->syms; s; s = s->next) {
	  if (s->v == v && s->mtype == memory)
			return s->name;
	}

	return 0;
}


char *
_sym_find_last(struct symtab_s *tab, int memory, unsigned int v, int *poffset)
{
	int i;
	struct sym_s *s;

	if (tab->sorted_syms == 0)
		return 0;

	for (i = 0; i < tab->sym_count; i++) {

		s = tab->sorted_syms[i];

		if (s->mtype != memory)
			continue;

		if (v == s->v) {
			*poffset = 0;
			return s->name;
		}

		if (v < s->v || i == tab->sym_count-1) {
			while (tab->sorted_syms[i-1]->mtype != memory)
				i--;

			s = tab->sorted_syms[i-1];
			*poffset = v - s->v;
			return s->name;
		}
	}

	return 0;
}

int
_sym_find(struct symtab_s *tab, char *name, int *pval)
{
	struct sym_s *s;

	for (s = tab->syms; s; s = s->next) {
		if (0) printf("%s %s\n", name, s->name);
		if (strcasecmp(name, s->name) == 0) {
			*pval = s->v;
			return 0;
		}
	}

	return -1;
}

/*
 * read a cadr mcr symbol file
 */
int
_sym_read_file(struct symtab_s *tab, const char *filename)
{
	int first = 1;
	FILE *f;
	char line[8*1024];
	char linebuf[8*1024];

	if (0) printf("tab %p, filename %s\n", tab, filename);

	f = fopen(filename, "r");
	if (f == NULL)
		return -1;

	tab->name = strdup(filename);

	while (fgets(line, sizeof(line), f) != NULL) {
		char sym[64], symtype[64];
		int loc, n;
		char *rv;

		if (first) {
			if (line[0] == 'I'){
			  // Added all this rv stuff to make gcc happy, but we really don't care if it fails.
			  while(line[0] != '-'){
			    rv = fgets(line, sizeof(line), f);
			    if(rv == NULL){
			      // Error
			    }
			  }
			  first = 0;
			  rv = fgets(line, sizeof(line), f);
			  if(rv == NULL){
			    // Error
			  }
			} else {
			  rv = fgets(line, sizeof(line), f);
			  if(rv == NULL){ }
			  rv = fgets(line, sizeof(line), f);
			  if(rv == NULL){ }
			  rv = fgets(line, sizeof(line), f);
			  if(rv == NULL){ }
			  rv = fgets(line, sizeof(line), f);
			  if(rv == NULL){ }
			  // OS X will force an abort if the source and destination overlap, so a buffer must be used.
			  strcpy(linebuf, &line[3]);
			  strcpy(line, linebuf);
			  first = 0;
			}
		}

		if (0) printf("'%s'\n", line);

		n = sscanf(line, "%s %s %o", sym, symtype, &loc);
		if (n == 3) {
			n = 0;
			if (strcmp(symtype, "I-MEM") == 0) n = 1;
			if (strcmp(symtype, "D-MEM") == 0) n = 2;
			if (strcmp(symtype, "A-MEM") == 0) n = 4;
			if (strcmp(symtype, "M-MEM") == 0) n = 5;
			if (strcmp(symtype, "NUMBER") == 0) n = 6;

			if (n == 0) printf("? %s", symtype);

			_sym_add(tab, n, sym, loc);
		}
	}

	fclose(f);
	return 0;
}

static int
_sym_loc_compare(const void *p1, const void *p2)
{
	struct sym_s *s1 = *(struct sym_s **)p1;
	struct sym_s *s2 = *(struct sym_s **)p2;

	if (s1->v < s2->v)
		return -1;

	if (s1->v > s2->v)
		return 1;

	return 0;
}

int
_sym_sort(struct symtab_s *tab)
{
	struct sym_s *s;
	int i;

	/* make vector of ptrs to syms */
	tab->sorted_syms = (struct sym_s **)malloc(sizeof(struct sym_s *) * tab->sym_count);
	if (tab->sorted_syms == 0)
		return -1;

	/* fill in vector */
	i = 0;
	for (s = tab->syms; s; s = s->next) {
		tab->sorted_syms[i++] = s;
	}

	printf("[%s] sort %d symbols (originally %d)\n", tab->name, i, tab->sym_count);

	/* sort the vector */
	qsort((void *)tab->sorted_syms, tab->sym_count, sizeof(void *), _sym_loc_compare);

#if 0
	for (i = 0; i < tab->sym_count; i++) {
		printf("%s %o\n", tab->sorted_syms[i]->name, tab->sorted_syms[i]->v);
	}
#endif

	return 0;
}

int
read_sym_files(void)
{
	_sym_read_file(&sym_mcr, "bootstrap.lam-uload");
	_sym_read_file(&sym_mcr, "ulambda.lmc-sym");
	_sym_sort(&sym_mcr);

	return 0;
}


/* ------------------------------------------------------------- */

int
sym_find(int mcr, char *name, int *pval)
{
	if (mcr)
		return _sym_find(&sym_mcr, name, pval);

	return _sym_find(&sym_prom, name, pval);
}

char *
sym_find_by_val(int mcr, int t, int v)
{
	if (mcr)
		return _sym_find_by_val(&sym_mcr, t, v);

	return _sym_find_by_val(&sym_prom, t, v);
}

char *
sym_find_by_type_val(int mcr, int t, int v)
{
	if (mcr)
		return _sym_find_by_val(&sym_mcr, t, v);

	return _sym_find_by_val(&sym_prom, t, v);
}

char *
sym_find_last(int mcr, int v, int *poffset)
{
	if (mcr)
	  return _sym_find_last(&sym_mcr, 1 /* I-MEM */, v, poffset);

	return _sym_find_last(&sym_prom, 1 /* I-MEM */, v, poffset);
}


