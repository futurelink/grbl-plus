/*
  settings.h - settings handling
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

#ifndef settings_h
#define settings_h

#include "grbl.h"

#ifdef __cplusplus
extern "C" {
#endif

// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 10  // NOTE: Check settings_reset() when moving to next version.

// Define bit flag masks for the boolean settings in settings.flag.
#define BITFLAG_REPORT_INCHES      bit(0)
#define BITFLAG_LASER_MODE         bit(1)
#define BITFLAG_INVERT_ST_ENABLE   bit(2)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(3)
#define BITFLAG_HOMING_ENABLE      bit(4)
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(5)
#define BITFLAG_INVERT_LIMIT_PINS  bit(6)
#define BITFLAG_INVERT_PROBE_PIN   bit(7)

// Define status reporting boolean enable bit flags in settings.status_report_mask
#define BITFLAG_RT_STATUS_POSITION_TYPE     bit(0)
#define BITFLAG_RT_STATUS_BUFFER_STATE      bit(1)

// Define settings restore bitflags.
#define SETTINGS_RESTORE_DEFAULTS bit(0)
#define SETTINGS_RESTORE_PARAMETERS bit(1)
#define SETTINGS_RESTORE_STARTUP_LINES bit(2)
#define SETTINGS_RESTORE_BUILD_INFO bit(3)
#ifndef SETTINGS_RESTORE_ALL
#define SETTINGS_RESTORE_ALL 0xFF // All bitflags
#endif

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future
// developments.
#define EEPROM_ADDR_GLOBAL         1U
#define EEPROM_ADDR_PARAMETERS     512U
#define EEPROM_ADDR_STARTUP_BLOCK  768U
#define EEPROM_ADDR_BUILD_INFO     942U

// Define EEPROM address indexing for coordinate parameters
#define N_COORDINATE_SYSTEM 6  // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)

// Define Grbl axis settings numbering scheme. Starts at START_VAL, every INCREMENT, over N_SETTINGS.
#define AXIS_N_SETTINGS          4
#define AXIS_SETTINGS_START_VAL  100 // NOTE: Reserving settings values >= 100 for axis settings. Up to 255.
#define AXIS_SETTINGS_INCREMENT  10  // Must be greater than the number of axis settings

typedef struct {
    // Axis settings
    float steps_per_mm[N_AXIS];
    float max_rate[N_AXIS];
    float acceleration[N_AXIS];
    float max_travel[N_AXIS];

    // Remaining Grbl settings
    uint8_t pulse_microseconds;
    uint8_t step_invert_mask;
    uint8_t dir_invert_mask;
    uint8_t stepper_idle_lock_time; // If max value 255, steppers do not disable.
    uint8_t status_report_mask; // Mask to indicate desired report data.
    float junction_deviation;
    float arc_tolerance;

    float rpm_max;
    float rpm_min;

    uint8_t flags;  // Contains default boolean settings

    uint8_t homing_dir_mask;
    float homing_feed_rate;
    float homing_seek_rate;
    uint16_t homing_debounce_delay;
    float homing_pulloff;
} settings_t;

class GRBLSettings {
private:
    settings_t settings;

public:

    // Axis settings
    float steps_per_mm(uint8_t axis) { return settings.steps_per_mm[axis]; }
    float max_rate(uint8_t axis) { return settings.max_rate[axis]; }
    float acceleration(uint8_t axis) { return settings.acceleration[axis]; }
    float max_travel(uint8_t axis) { return settings.max_travel[axis]; }

    float *accelerations() { return settings.acceleration; }
    float *max_rates() { return settings.max_rate; }

    // Remaining Grbl settings
    uint8_t pulse_microseconds() { return settings.pulse_microseconds; }
    uint8_t step_invert_mask() { return settings.step_invert_mask; }
    uint8_t dir_invert_mask() { return settings.dir_invert_mask; }
    uint8_t stepper_idle_lock_time() { return settings.stepper_idle_lock_time; }
    uint8_t status_report_mask() { return settings.status_report_mask; }
    float junction_deviation() { return settings.junction_deviation; }
    float arc_tolerance() { return settings.arc_tolerance; }

    float rpm_max() { return settings.rpm_max; }
    float rpm_min() { return settings.rpm_min; }

    uint8_t flags() { return settings.flags; }

    uint8_t homing_dir_mask(){ return settings.homing_dir_mask; }
    float homing_feed_rate(){ return settings.homing_feed_rate; }
    float homing_seek_rate(){ return settings.homing_seek_rate; }
    uint16_t homing_debounce_delay(){ return settings.homing_debounce_delay; }
    float homing_pulloff(){ return settings.homing_pulloff; }

    // Initialize the configuration subsystem (load settings from EEPROM)
    void init();

    // Helper function to clear and restore EEPROM defaults
    void restore(uint8_t restore_flag);

    // A helper method to set new settings from command line
    uint8_t store_global(uint8_t parameter, float value);

    void write_global();

    uint8_t read_global();

    // Stores the protocol line variable as a startup line in EEPROM
    static void store_startup_line(uint8_t n, char *line);

    // Reads an EEPROM startup line to the protocol line variable
    static uint8_t read_startup_line(uint8_t n, char *line);

    // Stores build info user-defined string
    static void store_build_info(char *line);

    // Reads build info user-defined string
    static uint8_t read_build_info(char *line);

    // Writes selected coordinate data to EEPROM
    static void write_coord_data(uint8_t coord_select, float *coord_data);

    // Reads selected coordinate data from EEPROM
    static uint8_t read_coord_data(uint8_t coord_select, float *coord_data);
};

#endif

#ifdef __cplusplus
}
#endif
