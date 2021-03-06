/*
  grbl.c - main Grbl file
  Part of Grbl

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

#include "grbl/grbl.h"

#include <cstdint>

GRBLMain::GRBLMain() {
    stm32_init();

    // Initialize system upon power-up.
    settings.init();    // Load Grbl settings from EEPROM
    steppers.init();    // Configure stepper pins and interrupt timers
    system.init();      // Configure pinout pins and pin-change interrupt

    // Initialize system state.
#ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = STATE_ALARM;
#else
    system.state = STATE_IDLE;
#endif

    // Check for power-up and set system alarm if homing is enabled to force homing cycle
    // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
    // startup scripts, but allows access to settings and internal commands. Only a homing
    // cycle '$H' or kill alarm locks '$X' will disable the alarm.
    // NOTE: The startup script will run after successful completion of the homing cycle, but
    // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
    // things uncontrollably. Very bad.
#ifdef HOMING_INIT_LOCK
    if (bit_istrue(grbl.settings.flags(), BITFLAG_HOMING_ENABLE)) { system.state = STATE_ALARM; }
#endif
}

// Grbl initialization loop upon power-up or a system abort.
// For the latter, all processes will return to this loop to be cleanly re-initialized.
void GRBLMain::run() {
    for (;;) {
        // Reset Grbl primary systems.
        system.reset(system.state);
        serial.reset_read_buffer(); // Clear serial read buffer

        gcode.init(); // Set g-code parser to default state
        spindle.init();
        coolant.init();
        limits.init();
        probe.init();

        planner.reset(); // Clear block buffer and planner variables
        steppers.reset(); // Clear stepper subsystem variables.

        // Sync cleared gcode and planner positions to current system position.
        planner.sync_position();
        gcode.sync_position();

        // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
        GRBLReport::init_message();

        // Start Grbl main loop. Processes program inputs and executes them.
        protocol.main_loop();
    }
}
