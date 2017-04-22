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

int read_sym_files(void);

int sym_find(int mcr, char *name, int *pval);
char *sym_find_by_val(int mcr, int t, int v);
char *sym_find_by_type_val(int mcr, int t, int v);
char *sym_find_last(int mcr, int v, int *poffset);

int _sym_read_file(struct symtab_s *tab, const char *filename);
int _sym_sort(struct symtab_s *tab);
