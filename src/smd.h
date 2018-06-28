/* Copyright 2016-2017 
   Daniel Seagraves <dseagrav@lunar-tokyo.net>

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

void smd_clock_pulse();
int smd_init();
void smd_reset();
uint8_t smd_read(uint8_t addr);
void smd_write(uint8_t addr,uint8_t data);
#ifdef HAVE_YAML_H
int yaml_disk_mapping_loop(yaml_parser_t *parser);
int yaml_disk_sequence_loop(yaml_parser_t *parser);
#endif
