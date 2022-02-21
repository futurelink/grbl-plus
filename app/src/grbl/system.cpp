/*
  system.c - Handles system level commands and real-time processes
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
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
#include "stm32/stm32_helpers.h"

void GRBLSystem::init() {
    stm32_system_init();
}

// Returns control pin state as a uint8 bitfield. Each bit indicates the input pin state, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Bitfield organization is
// defined by the CONTROL_PIN_INDEX in the header file.
uint8_t GRBLSystem::control_get_state() {
    uint8_t control_state = 0;
    uint16_t pin = CONTROL_PIN_PORT->IDR & CONTROL_MASK;

    #ifdef INVERT_CONTROL_PIN_MASK
    pin ^= INVERT_CONTROL_PIN_MASK;
    #endif
    if (pin) {
        #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
        if (bit_isfalse(pin,(1<<CONTROL_SAFETY_DOOR_BIT))) { control_state |= CONTROL_PIN_INDEX_SAFETY_DOOR; }
        #endif
        if (bit_isfalse(pin,(1<<CONTROL_RESET_BIT))) { control_state |= CONTROL_PIN_INDEX_RESET; }
        if (bit_isfalse(pin,(1<<CONTROL_FEED_HOLD_BIT))) { control_state |= CONTROL_PIN_INDEX_FEED_HOLD; }
        if (bit_isfalse(pin,(1<<CONTROL_CYCLE_START_BIT))) { control_state |= CONTROL_PIN_INDEX_CYCLE_START; }
    }
    return (control_state);
}

// Pin change interrupt for pin-out commands, i.e. cycle start, feed hold, and reset. Sets
// only the realtime command execute variable to have the main program execute these when
// its ready. This works exactly like the character-based realtime commands when picked off
// directly from the incoming serial data stream.
void GRBLSystem::external_interrupts_handle() {
    uint8_t pin = control_get_state();
    if (pin) {
        if (bit_istrue(pin, CONTROL_PIN_INDEX_RESET)) {
            grbl.motion.reset();
        } else if (bit_istrue(pin, CONTROL_PIN_INDEX_CYCLE_START)) {
            bit_true(grbl.sys_rt_exec_state, EXEC_CYCLE_START);
        }
#ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
        else if (bit_istrue(pin, CONTROL_PIN_INDEX_FEED_HOLD)) {
            bit_true(grbl.sys_rt_exec_state, EXEC_FEED_HOLD);
        }
#else
        else if (bit_istrue(pin, CONTROL_PIN_INDEX_SAFETY_DOOR)) {
			bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
		}
#endif

    }
}

// Returns if safety door is ajar(T) or closed(F), based on pin state.
uint8_t GRBLSystem::check_safety_door_ajar() {
    #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
        return(system_control_get_state() & CONTROL_PIN_INDEX_SAFETY_DOOR);
    #else
        return(false); // Input pin not enabled, so just return that it's closed.
    #endif
}

// Executes user startup script, if stored.
void GRBLSystem::execute_startup(char *line) {
    uint8_t n;
    for (n=0; n < N_STARTUP_LINE; n++) {
        if (!(grbl.settings.read_startup_line(n, line))) {
            line[0] = 0;
            GRBLReport::execute_startup_message(line,STATUS_SETTING_READ_FAIL);
        } else {
            if (line[0] != 0) {
                uint8_t status_code = grbl.gcode.execute_line(line);
                GRBLReport::execute_startup_message(line,status_code);
            }
        }
    }
}


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the realtime command module by being susceptible to when Grbl is ready to execute the
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle,
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.
uint8_t GRBLSystem::execute_line(char *line) {
    uint8_t char_counter = 1;
    uint8_t helper_var = 0; // Helper variable
    float parameter, value;

    switch(line[char_counter]) {

        case 0 : GRBLReport::grbl_help(); break;

        case 'J' : // Jogging
            // Execute only if in IDLE or JOG states.
            if (grbl.sys.state != STATE_IDLE && grbl.sys.state != STATE_JOG) { return(STATUS_IDLE_ERROR); }
            if(line[2] != '=') { return(STATUS_INVALID_STATEMENT); }
            return(grbl.gcode.execute_line(line)); // NOTE: $J= is ignored inside g-code parser and used to detect jog motions.
			break;
        case '$': case 'G': case 'C': case 'X':
            if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
            switch( line[1] ) {
                case '$' : // Prints Grbl settings
                    if ( grbl.sys.state & (STATE_CYCLE | STATE_HOLD) ) { return(STATUS_IDLE_ERROR); } // Block during cycle. Takes too long to print.
                    else { GRBLReport::grbl_settings(&grbl.settings); }
                    break;
                case 'G' : // Prints gcode parser state
                    // TODO: Move this to realtime commands for GUIs to request this data during suspend-state.
                    GRBLReport::gcode_modes();
                    break;
                case 'C' : // Set check g-code mode [IDLE/CHECK]
                    // Perform reset when toggling off. Check g-code mode should only work if Grbl
                    // is idle and ready, regardless of alarm locks. This is mainly to keep things
                    // simple and consistent.
                    if ( grbl.sys.state == STATE_CHECK_MODE ) {
                        grbl.motion.reset();
                        GRBLReport::feedback_message(MESSAGE_DISABLED);
                    } else {
                        if (grbl.sys.state) { return(STATUS_IDLE_ERROR); } // Requires no alarm mode.
                        grbl.sys.state = STATE_CHECK_MODE;
                        GRBLReport::feedback_message(MESSAGE_ENABLED);
                    }
                    break;
                case 'X' : // Disable alarm lock [ALARM]
                    if (grbl.sys.state == STATE_ALARM) {

                        // Block if safety door is ajar.
                        if (check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); }

                        GRBLReport::feedback_message(MESSAGE_ALARM_UNLOCK);
                        grbl.sys.state = STATE_IDLE;
                        // Don't run startup script. Prevents stored moves in startup from causing accidents.
                    } // Otherwise, no effect.
                    break;
            }
            break;

        default :
            // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
            if ( !(grbl.sys.state == STATE_IDLE || grbl.sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
            switch( line[1] ) {
                case '#' : // Print Grbl NGC parameters
                    if ( line[2] != 0 ) { return(STATUS_INVALID_STATEMENT); }
                    else { GRBLReport::ngc_parameters(); }
                    break;

                case 'H' : // Perform homing cycle [IDLE/ALARM]
                    if (bit_isfalse(grbl.settings.flags(),BITFLAG_HOMING_ENABLE)) {return(STATUS_SETTING_DISABLED); }

                    if (check_safety_door_ajar()) { return(STATUS_CHECK_DOOR); } // Block if safety door is ajar.

                    grbl.sys.state = STATE_HOMING; // Set system state variable
                    if (line[2] == 0) {
                        grbl.motion.homing_cycle(HOMING_CYCLE_ALL);
                        #ifdef HOMING_SINGLE_AXIS_COMMANDS
                        } else if (line[3] == 0) {
              switch (line[2]) {
                case 'X': mc_homing_cycle(HOMING_CYCLE_X); break;
                case 'Y': mc_homing_cycle(HOMING_CYCLE_Y); break;
                case 'Z': mc_homing_cycle(HOMING_CYCLE_Z); break;
                default: return(STATUS_INVALID_STATEMENT);
              }
                        #endif
                    } else { return(STATUS_INVALID_STATEMENT); }
                    if (!grbl.sys.abort) {  // Execute startup scripts after successful homing.
                        grbl.sys.state = STATE_IDLE; // Set to IDLE when complete.
                        grbl.steppers.go_idle(); // Set steppers to the settings idle state before returning.
                        if (line[2] == 0) { execute_startup(line); }
                    }
                    break;

                case 'S' : // Puts Grbl to sleep [IDLE/ALARM]
                    if ((line[2] != 'L') || (line[3] != 'P') || (line[4] != 0)) { return(STATUS_INVALID_STATEMENT); }
                    set_exec_state_flag(EXEC_SLEEP); // Set to execute sleep mode immediately
                    break;

                case 'I' : // Print or store build info. [IDLE/ALARM]
                    if ( line[++char_counter] == 0 ) {
                        GRBLSettings::read_build_info(line);
                        GRBLReport::build_info(line);
                    #ifdef ENABLE_BUILD_INFO_WRITE_COMMAND
                    } else { // Store startup line [IDLE/ALARM]
                        if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
                        helper_var = char_counter; // Set helper variable as counter to start of user info line.
                        do {
                            line[char_counter-helper_var] = line[char_counter];
                        } while (line[char_counter++] != 0);
                        grbl.settings.store_build_info(line);
                    #endif
                    }
                    break;

                case 'R' : // Restore defaults [IDLE/ALARM]
                    if ((line[2] != 'S') || (line[3] != 'T') || (line[4] != '=') || (line[6] != 0)) { return(STATUS_INVALID_STATEMENT); }
                    switch (line[5]) {
                        #ifdef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS
                        case '$': grbl.settings.restore(SETTINGS_RESTORE_DEFAULTS); break;
                        #endif
                        #ifdef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS
                        case '#': grbl.settings.restore(SETTINGS_RESTORE_PARAMETERS); break;
                        #endif
                        #ifdef ENABLE_RESTORE_EEPROM_WIPE_ALL
                        case '*': grbl.settings.restore(SETTINGS_RESTORE_ALL); break;
                        #endif
                        default: return(STATUS_INVALID_STATEMENT);
                    }
                    GRBLReport::feedback_message(MESSAGE_RESTORE_DEFAULTS);
                    grbl.motion.reset(); // Force reset to ensure settings are initialized correctly.
                    break;

                case 'N' : // Startup lines. [IDLE/ALARM]
                    if ( line[++char_counter] == 0 ) { // Print startup lines
                        for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
                            if (!(GRBLSettings::read_startup_line(helper_var, line))) {
                                GRBLReport::status_message(STATUS_SETTING_READ_FAIL);
                            } else {
                                GRBLReport::startup_line(helper_var,line);
                            }
                        }
                        break;
                    } else { // Store startup line [IDLE Only] Prevents motion during ALARM.
                        if (grbl.sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } // Store only when idle.
                        helper_var = true;  // Set helper_var to flag storing method.
                        // No break. Continues into default: to read remaining command characters.
                    }

                default :  // Storing setting methods [IDLE/ALARM]
                    if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
                    if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
                    if (helper_var) { // Store startup line
                        // Prepare sending gcode block to gcode parser by shifting all characters
                        helper_var = char_counter; // Set helper variable as counter to start of gcode block
                        do {
                            line[char_counter-helper_var] = line[char_counter];
                        } while (line[char_counter++] != 0);
                        // Execute gcode block to ensure block is valid.
                        helper_var = grbl.gcode.execute_line(line); // Set helper_var to returned status code.
                        if (helper_var) { return(helper_var); }
                        else {
                            helper_var = truncf(parameter); // Set helper_var to int value of parameter
                            GRBLSettings::store_startup_line(helper_var,line);
                        }
                    } else { // Store global setting.
                        if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
                        if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
                        return(grbl.settings.store_global((uint8_t)parameter, value));
                    }
                }
            }
    return(STATUS_OK); // If '$' command makes it to here, then everything's ok.
}

void GRBLSystem::flag_wco_change() {
    #ifdef FORCE_BUFFER_SYNC_DURING_WCO_CHANGE
    GRBLProtocol::buffer_synchronize();
    #endif
    grbl.sys.report_wco_counter = 0;
}


// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
float GRBLSystem::convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx) {
    float pos;
    #ifdef COREXY
    if (idx == X_AXIS) {
        pos = (float)system_convert_corexy_to_x_axis_steps(steps) / settings.steps_per_mm[idx];
    } else if (idx == Y_AXIS) {
        pos = (float)system_convert_corexy_to_y_axis_steps(steps) / settings.steps_per_mm[idx];
    } else {
        pos = steps[idx]/settings.steps_per_mm[idx];
    }
    #else
    pos = steps[idx]/grbl.settings.steps_per_mm(idx);
    #endif
    return(pos);
}


void GRBLSystem::convert_array_steps_to_mpos(float *position, int32_t *steps) {
    uint8_t idx;
    for (idx=0; idx<N_AXIS; idx++) {
        position[idx] = convert_axis_steps_to_mpos(steps, idx);
    }
	return;
}


// CoreXY calculation only. Returns x or y-axis "steps" based on CoreXY motor steps.
#ifdef COREXY
  int32_t GRBLSystem::convert_corexy_to_x_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] + steps[B_MOTOR])/2 );
  }
  int32_t GRBLSystem::convert_corexy_to_y_axis_steps(int32_t *steps)
  {
    return( (steps[A_MOTOR] - steps[B_MOTOR])/2 );
  }
#endif

// Checks and reports if target array exceeds machine travel limits.
uint8_t GRBLSystem::check_travel_limits(float *target) {
    uint8_t idx;
    for (idx=0; idx<N_AXIS; idx++) {
        #ifdef HOMING_FORCE_SET_ORIGIN
        // When homing forced set origin is enabled, soft limits checks need to account for directionality.
      // NOTE: max_travel is stored as negative
      if (bit_istrue(settings.homing_dir_mask,bit(idx))) {
        if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { return(true); }
      } else {
        if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { return(true); }
      }
        #else
        // NOTE: max_travel is stored as negative
        if (target[idx] > 0 || target[idx] < grbl.settings.max_travel(idx)) { return(true); }
        #endif
    }
    return(false);
}

// Special handlers for setting and clearing Grbl's real-time execution flags.
void GRBLSystem::set_exec_state_flag(uint8_t mask) {
    __disable_irq();
    grbl.sys_rt_exec_state |= (mask);
    __enable_irq();
}

void GRBLSystem::clear_exec_state_flag(uint8_t mask) {
    __disable_irq();
    grbl.sys_rt_exec_state &= ~(mask);
    __enable_irq();
}

void GRBLSystem::set_exec_alarm(uint8_t code) {
    __disable_irq();
    grbl.sys_rt_exec_alarm |= (code);
    __enable_irq();
}

void GRBLSystem::clear_exec_alarm() {
    __disable_irq();
    grbl.sys_rt_exec_alarm = 0;
    __enable_irq();
}

void GRBLSystem::set_exec_motion_override_flag(uint8_t mask) {
    __disable_irq();
    grbl.sys_rt_exec_motion_override |= (mask);
    __enable_irq();
}

void GRBLSystem::set_exec_accessory_override_flag(uint8_t mask) {
    __disable_irq();
    grbl.sys_rt_exec_accessory_override |= (mask);
    __enable_irq();
}

void GRBLSystem::clear_exec_motion_overrides() {
    __disable_irq();
    grbl.sys_rt_exec_motion_override = 0;
    __enable_irq();
}

void GRBLSystem::clear_exec_accessory_overrides() {
    __disable_irq();
    grbl.sys_rt_exec_accessory_override = 0;
    __enable_irq();
}
