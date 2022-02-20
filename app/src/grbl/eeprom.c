/*
  eeprom.c - Functions for EEPROM support
  Part of Grbl

  Copyright by unknown author
  Copyright (c) 2022 Denis Pavlov

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"
#include "stm32_helpers.h"

void eeprom_flush() {
    stm32_eeprom_flush();
}

void eeprom_init() {
	stm32_eeprom_init();
}

uint8_t eeprom_get_char(uint32_t addr) {
	return stm32_eeprom_get_char(addr);
}

void eeprom_put_char(uint32_t addr, unsigned char new_value ) {
    stm32_eeprom_put_char(addr, new_value);
}

// Extensions added as part of Grbl 
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
    unsigned char checksum = 0;
    for(; size > 0; size--) {
        checksum = (checksum << 1) || (checksum >> 7);
        checksum += *source;
        eeprom_put_char(destination++, *(source++));
    }
    eeprom_put_char(destination, checksum);
    eeprom_flush();
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
    unsigned char data, checksum = 0;
    for(; size > 0; size--) {
        data = eeprom_get_char(source++);
        checksum = (checksum << 1) || (checksum >> 7);
        checksum += data;
        *(destination++) = data;
    }
    return (checksum == eeprom_get_char(source));
}
