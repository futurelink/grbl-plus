/*
  limits.h - code pertaining to limit-switches and performing the homing cycle
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

#ifndef limits_h
#define limits_h

#ifdef __cplusplus
extern "C" {
#endif

class GRBLLimits {
    const uint16_t limit_pin_mask[N_AXIS] = {
            1 << X_LIMIT_BIT,
            1 << Y_LIMIT_BIT,
            1 << Z_LIMIT_BIT,
    };
public:
    // Initialize the limits module
    static void init();

    // Disables hard limits.
    static void disable();

    // Returns limit state as a bit-wise uint8 variable.
    uint8_t get_state();

    // Perform one portion of the homing cycle based on the input settings.
    void go_home(uint8_t cycle_mask);

    // Check for soft limit violations
    static void soft_check(float *target);

    static void external_interrupt_handle();
};

#ifdef __cplusplus
}
#endif

#endif
