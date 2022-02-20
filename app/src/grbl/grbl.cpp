//
// Created by depavlov on 20.02.2022.
//

#include "grbl.h"

#include <cstdint>

GRBLMain::GRBLMain() {
    stm32_init();

    // Initialize system upon power-up.
    settings.init(); // Load Grbl settings from EEPROM
    steppers.init();  // Configure stepper pins and interrupt timers
    system_init();   // Configure pinout pins and pin-change interrupt

    memset(sys_position, 0, sizeof(sys_position)); // Clear machine position.

    // Initialize system state.
#ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = STATE_ALARM;
#else
    sys.state = STATE_IDLE;
#endif

    // Check for power-up and set system alarm if homing is enabled to force homing cycle
    // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
    // startup scripts, but allows access to settings and internal commands. Only a homing
    // cycle '$H' or kill alarm locks '$X' will disable the alarm.
    // NOTE: The startup script will run after successful completion of the homing cycle, but
    // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
    // things uncontrollably. Very bad.
#ifdef HOMING_INIT_LOCK
    if (bit_istrue(grbl.settings.flags(), BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
#endif
}

void GRBLMain::run() {
    // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
    // will return to this loop to be cleanly re-initialized.
    for (;;) {
        // Reset system variables.
        uint8_t prior_state = sys.state;
        memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
        sys.state = prior_state;
        sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
        sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
        sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%

        memset(sys_probe_position, 0, sizeof(sys_probe_position)); // Clear probe position.

        sys_probe_state = 0;
        sys_rt_exec_state = 0;
        sys_rt_exec_alarm = 0;
        sys_rt_exec_motion_override = 0;
        sys_rt_exec_accessory_override = 0;

        // Reset Grbl primary systems.
        serial_reset_read_buffer(); // Clear serial read buffer
        gc_init(); // Set g-code parser to default state
        spindle.init();
        coolant.init();
        limits_init();
        probe.init();
        planner.reset(); // Clear block buffer and planner variables
        steppers.reset(); // Clear stepper subsystem variables.

        // Sync cleared gcode and planner positions to current system position.
        planner.sync_position();
        gc_sync_position();

        // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
        report_init_message();

        // Start Grbl main loop. Processes program inputs and executes them.
        protocol_main_loop();
    }
}