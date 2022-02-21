/*
  stm32_routines.h - hardware specific routines
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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

#ifndef _STM32_ROUTINES_H
#define _STM32_ROUTINES_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define F_CPU                   SystemCoreClock
#define PORTPINDEF              uint16_t
#define PSTR(x)                 x
#define pgm_read_byte_near(x)   *(x)
#define printPgmString          printString

void stm32_config_timer(TIM_TypeDef *TIMER, uint16_t Period, uint16_t Prescaler, uint8_t PP);

void stm32_init();

void stm32_system_init();

void stm32_spindle_init();

void stm32_probe_init();

void stm32_coolant_init();

void stm32_stepper_init();

void stm32_limits_init();

void stm32_limits_enable();

void stm32_limits_disable();

void stm32_eeprom_init();

void stm32_eeprom_flush();

uint8_t stm32_eeprom_get_char(uint32_t addr);

void stm32_eeprom_put_char(uint32_t addr, uint8_t value);

uint8_t stm32_get_flood_state();

void stm32_set_flood_state(bool state);

uint8_t stm32_get_mist_state();

void stm32_set_mist_state(bool state);

void stm32_steppers_wake_up(uint8_t step_pulse_time, uint16_t cycles_per_tick);

void stm32_steppers_set(PORTPINDEF dir_bits, PORTPINDEF step_bits);

bool stm32_steppers_pulse_start(bool busy, PORTPINDEF dir_bits, PORTPINDEF step_bits);

void stm32_steppers_pulse_end(PORTPINDEF step_mask);

#ifdef __cplusplus
}
#endif

#endif
