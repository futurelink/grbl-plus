/*
  coolant_control.c - coolant control methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

void GRBLCoolant::init() {
    stm32_coolant_init();
    stop();
}

// Returns current coolant output state. Overrides may alter it from programmed state.
uint8_t GRBLCoolant::get_state() {
    uint8_t cl_state = COOLANT_STATE_DISABLE;
    #ifdef INVERT_COOLANT_FLOOD_PIN
    if (HAL_GPIO_ReadPin(COOLANT_FLOOD_PORT), COOLANT_FLOOD_BIT) == GPIO_PIN_RESET)
    #else
    if (HAL_GPIO_ReadPin(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT) == GPIO_PIN_SET)
    #endif
    {
        cl_state |= COOLANT_STATE_FLOOD;
    }

    #ifdef ENABLE_M7
    #ifdef INVERT_COOLANT_MIST_PIN
    if (HAL_GPIO_ReadPin(COOLANT_MIST_PORT, COOLANT_MIST_BIT) == GPIO_PIN_RESET)
    #else
    if (HAL_GPIO_ReadPin(COOLANT_MIST_PORT, COOLANT_MIST_BIT) == GPIO_PIN_SET)
    #endif
    {
      cl_state |= COOLANT_STATE_MIST;
    }
    #endif

    return(cl_state);
}


// Directly called by coolant_init(), coolant_set_state(), and mc_reset(), which can be at
// an interrupt-level. No report flag set, but only called by routines that don't need it.
void GRBLCoolant::stop() {
    #ifdef INVERT_COOLANT_FLOOD_PIN
    HAL_GPIO_WritePin(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, GPIO_PIN_SET);
    #else
  	HAL_GPIO_WritePin(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, GPIO_PIN_RESET);
    #endif

    #ifdef ENABLE_M7
    #ifdef INVERT_COOLANT_MIST_PIN
        HAL_GPIO_WritePin(COOLANT_MIST_PORT, COOLANT_MIST_BIT, GPIO_PIN_SET);
    #else
        HAL_GPIO_WritePin(COOLANT_MIST_PORT, COOLANT_MIST_BIT, GPIO_PIN_RESET);
    #endif
    #endif
}


// Main program only. Immediately sets flood coolant running state and also mist coolant, 
// if enabled. Also sets a flag to report an update to a coolant state.
// Called by coolant toggle override, parking restore, parking retract, sleep mode, g-code
// parser program end, and g-code parser coolant_sync().
void GRBLCoolant::set_state(uint8_t mode) {
    if (grbl.sys.abort) { return; } // Block during abort.
  
    if (mode == COOLANT_DISABLE) {
        stop();
    } else {
        if (mode & COOLANT_FLOOD_ENABLE) {
            #ifdef INVERT_COOLANT_FLOOD_PIN
            HAL_GPIO_WritePin(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, GPIO_PIN_RESET);
            #else
            HAL_GPIO_WritePin(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, GPIO_PIN_SET);
            #endif
        }
  
        #ifdef ENABLE_M7
        if (mode & COOLANT_MIST_ENABLE) {
            #ifdef INVERT_COOLANT_MIST_PIN
            HAL_GPIO_WritePin(COOLANT_MIST_PORT, COOLANT_MIST_BIT, GPIO_PIN_RESET);
            #else
	        HAL_GPIO_WritePin(COOLANT_MIST_PORT, COOLANT_MIST_BIT, GPIO_PIN_SET);
            #endif
        }
    #endif
    }
    grbl.sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting coolant state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
void GRBLCoolant::sync(uint8_t mode) {
    if (grbl.sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.
    set_state(mode);
}
