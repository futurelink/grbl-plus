/*
  protocol.h - controls Grbl execution protocol and procedures
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef protocol_h
#define protocol_h

#ifdef __cplusplus
extern "C" {
#endif

// Define line flags. Includes comment type tracking and line overflow detection.
#define LINE_FLAG_OVERFLOW              bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES   bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON     bit(2)

// Line buffer size from the serial input stream to be executed.
// NOTE: Not a problem except for extreme cases, but the line buffer size can be too small
// and g-code blocks can get truncated. Officially, the g-code standards support up to 256
// characters. In future versions, this will be increased, when we know how much extra
// memory space we can invest into here or we re-write the g-code parser not to have this
// buffer.
#ifndef LINE_BUFFER_SIZE
#define LINE_BUFFER_SIZE 80
#endif

class GRBLProtocol {
private:
    char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.

public:
    // Starts Grbl main loop. It handles all incoming characters from the serial port and executes
    // them as they complete. It is also responsible for finishing the initialization procedures.
    void main_loop();

    // Checks and executes a realtime command at various stop points in main program
    static void execute_realtime();

    static void exec_rt_system();

    // Executes the auto cycle feature, if enabled.
    static void auto_cycle_start();

    static void exec_rt_suspend();

    // Block until all buffered steps are executed
    static void buffer_synchronize();
};

#ifdef __cplusplus
}
#endif

#endif
