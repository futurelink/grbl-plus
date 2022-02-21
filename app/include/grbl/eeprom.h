/*
  eeprom.h - EEPROM methods
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
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

#ifndef eeprom_h
#define eeprom_h

class GRBLEEPROM {
public:
    static void init();
    static void flush();
    static uint8_t get_char(uint32_t addr);
    static void put_char(uint32_t addr, uint8_t new_value);
    static void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size);
    static int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size);
};

#endif
