#ifndef _STM32_HELPERS_H
#define _STM32_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define PORTPINDEF uint16_t
#define PSTR(x) x
#define pgm_read_byte_near(x) *(x)
#define false 0
#define true 1
#define printPgmString printString

void TIM_Configuration(TIM_TypeDef *TIMER, uint16_t Period, uint16_t Prescaler, uint8_t PP);

void stm32_init();

void stm32_system_init();

void stm32_spindle_init();

void stm32_probe_init();

void stm32_coolant_init();

void stm32_stepper_init();

void stm32_limits_init();

void stm32_limits_enable();

void stm32_limits_disable();

void stm32_limits_clear();

void stm32_eeprom_init();

void stm32_eeprom_flush();

uint8_t stm32_eeprom_get_char(uint32_t addr);

void stm32_eeprom_put_char(uint32_t addr, uint8_t value);

#ifdef __cplusplus
}
#endif


uint8_t stm32_get_flood_state();

void stm32_set_flood_state(bool state);

uint8_t stm32_get_mist_state();

void stm32_set_mist_state(bool state);

bool stm32_steppers_pulse_start(bool busy, PORTPINDEF dir_bits, PORTPINDEF step_bits);

void stm32_steppers_pulse_end(PORTPINDEF step_mask);

#endif
