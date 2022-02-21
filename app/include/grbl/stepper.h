/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
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

#ifndef stepper_h
#define stepper_h

#include "../stm32/stm32_helpers.h"

#ifndef SEGMENT_BUFFER_SIZE
#define SEGMENT_BUFFER_SIZE 10
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Some useful constants.
#define DT_SEGMENT              (1.0f/(ACCELERATION_TICKS_PER_SECOND*60.0f)) // min/segment
#define REQ_MM_INCREMENT_SCALAR 1.25f
#define RAMP_ACCEL              0
#define RAMP_CRUISE             1
#define RAMP_DECEL              2
#define RAMP_DECEL_OVERRIDE     3

#define PREP_FLAG_RECALCULATE bit(0)
#define PREP_FLAG_HOLD_PARTIAL_BLOCK bit(1)
#define PREP_FLAG_PARKING bit(2)
#define PREP_FLAG_DECEL_OVERRIDE bit(3)

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
#define MAX_AMASS_LEVEL 3
// AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
#define AMASS_LEVEL1 (F_CPU/8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
#define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
#define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)

#if MAX_AMASS_LEVEL <= 0
error "AMASS must have 1 or more levels to operate correctly."
#endif
#endif

// Stores the planner block Bresenham algorithm execution data for the segments in the segment
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use.
typedef struct {
    uint32_t steps[N_AXIS];
    uint32_t step_event_count;
    uint8_t direction_bits;
#ifdef VARIABLE_SPINDLE
    uint8_t is_pwm_rate_adjusted; // Tracks motions that require constant laser power/rate
#endif
} st_block_t;

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
// the planner, where the remaining planner block steps still can.
typedef struct {
    uint16_t n_step;           // Number of step events to be executed for this segment
    uint16_t cycles_per_tick;  // Step distance traveled per ISR tick, aka step rate.
    uint8_t  st_block_index;   // Stepper block data index. Uses this information to execute this segment.
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
#else
    uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
#endif
#ifdef VARIABLE_SPINDLE
    uint8_t spindle_pwm;
#endif
} segment_t;

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct {
    // Used by the bresenham line algorithm
    uint32_t counter_x,        // Counter variables for the bresenham line tracer
            counter_y,
            counter_z;
#ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay
#endif

    uint8_t execute_step;     // Flags step execution for each interrupt.
    uint8_t step_pulse_time;  // Step pulse reset time after step rise

    PORTPINDEF step_outbits;         // The next stepping-bits to be output
    PORTPINDEF dir_outbits;
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
#endif

    uint16_t step_count;       // Steps remaining in line segment motion
    uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
    st_block_t *exec_block;   // Pointer to the block data for the segment being executed
    segment_t *exec_segment;  // Pointer to the segment being executed
} stepper_t;

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
    uint8_t st_block_index;  // Index of stepper common data block being prepped
    uint8_t recalculate_flag;

    float dt_remainder;
    float steps_remaining;
    float step_per_mm;
    float req_mm_increment;

#ifdef PARKING_ENABLE
    uint8_t last_st_block_index;
    float last_steps_remaining;
    float last_step_per_mm;
    float last_dt_remainder;
#endif

    uint8_t ramp_type;      // Current segment ramp state
    float mm_complete;      // End of velocity profile from end of current planner block in (mm).
    // NOTE: This value must coincide with a step(no mantissa) when converted.
    float current_speed;    // Current speed at the end of the segment buffer (mm/min)
    float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
    float exit_speed;       // Exit speed of executing block (mm/min)
    float accelerate_until; // Acceleration ramp end measured from end of block (mm)
    float decelerate_after; // Deceleration ramp start measured from end of block (mm)

#ifdef VARIABLE_SPINDLE
    float inv_rate;    // Used by PWM laser mode to speed up segment calculations.
    uint8_t current_spindle_pwm;
#endif
} st_prep_t;

class GRBLSteppers {
    const PORTPINDEF step_pin_mask[N_AXIS] = {
            1 << X_STEP_BIT,
            1 << Y_STEP_BIT,
            1 << Z_STEP_BIT,
    };

    const PORTPINDEF direction_pin_mask[N_AXIS] = {
            1 << X_DIRECTION_BIT,
            1 << Y_DIRECTION_BIT,
            1 << Z_DIRECTION_BIT,
    };

public:
    // Step and direction port invert masks.
    PORTPINDEF step_port_invert_mask;
    PORTPINDEF dir_port_invert_mask;

    st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];
    segment_t segment_buffer[SEGMENT_BUFFER_SIZE];
    stepper_t st;

    // Step segment ring buffer indices
    volatile uint8_t segment_buffer_tail;
    uint8_t segment_buffer_head;
    uint8_t segment_next_head;

    // Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
    volatile uint8_t busy;

    // Pointers for the step segment being prepped from the planner buffer. Accessed only by the
    // main program. Pointers may be planning segments or planner blocks ahead of what being executed.
    plan_block_t *pl_block;     // Pointer to the planner block being prepped
    st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped
    st_prep_t prep;

    static uint8_t next_block_index(uint8_t block_index);

    void pulse_end() const;
    void pulse_start();

    // Initialize and setup the stepper motor subsystem
    static void init();

    // Enable steppers, but cycle does not start unless called by motion control or realtime command.
    void wake_up();

    // Immediately disables steppers
    void go_idle();

    // Generate the step and direction port invert masks.
    void generate_step_dir_invert_masks();

    // Reset the stepper subsystem variables
    void reset();

#ifdef PARKING_ENABLE
    // Changes the run state of the step segment buffer to execute the special parking motion.
    void parking_setup_buffer();

    // Restores the step segment buffer to the normal run state after a parking motion.
    void parking_restore_buffer();
#endif

    // Reloads step segment buffer. Called continuously by realtime execution system.
    void prep_buffer();

    // Called by planner_recalculate() when the executing block is updated by the new plan.
    void update_plan_block_parameters();

    // Called by realtime status reporting if realtime rate reporting is enabled in config.h.
    float get_realtime_rate() const;

    uint8_t step_pin_mask_bit(PORTPINDEF axis_bit);
    uint8_t dir_pin_mask_bit(PORTPINDEF axis_bit);
};

#endif

#ifdef __cplusplus
}
#endif
