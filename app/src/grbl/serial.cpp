/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
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

#include "grbl/grbl.h"

#include "core_cm3.h"
#include "stm32/usb/usbd_cdc_if.h"

// Returns the number of bytes available in the RX serial buffer.
uint8_t GRBLSerial::get_rx_buffer_available() {
    uint8_t rtail = rx_buffer_tail; // Copy to limit multiple calls to volatile
    if (rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (rx_buffer_head - rtail)); }
    return((rtail - rx_buffer_head - 1));
}

// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t GRBLSerial::get_rx_buffer_count() {
    uint8_t rtail = rx_buffer_tail; // Copy to limit multiple calls to volatile
    if (rx_buffer_head >= rtail) { return(rx_buffer_head - rtail); }
    return (RX_BUFFER_SIZE - (rtail - rx_buffer_head));
}

// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t GRBLSerial::get_tx_buffer_count() {
    uint8_t ttail = tx_buffer_tail; // Copy to limit multiple calls to volatile
    if (tx_buffer_head >= ttail) { return(tx_buffer_head - ttail); }
    return (TX_RING_BUFFER - (ttail - tx_buffer_head));
}

// Writes one byte to the TX serial buffer. Called by main program.
void GRBLSerial::write(uint8_t data) {
    // Calculate next head
    uint8_t next_head = tx_buffer_head + 1;
    if (next_head == TX_RING_BUFFER) { next_head = 0; }

    // Wait until there is space in the buffer
    while (next_head == tx_buffer_tail) {
        // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
        if (grbl.sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
    }

    // Store data and advance head
    tx_buffer[tx_buffer_head] = data;
    tx_buffer_head = next_head;
}

// Circular buffer should be sent as linear
void GRBLSerial::transmit() {
    if (tx_buffer_head > tx_buffer_tail) {
        if (CDC_Transmit_FS(tx_buffer + tx_buffer_tail, tx_buffer_head - tx_buffer_tail) == USBD_OK) {
            tx_buffer_tail = tx_buffer_head;
        }
    } else if (tx_buffer_head < tx_buffer_tail) {
        // Send first part from data tail to end of the buffer
        if (CDC_Transmit_FS(tx_buffer + tx_buffer_tail, TX_RING_BUFFER - tx_buffer_tail) == USBD_OK) {
            tx_buffer_tail = 0;
            // Send second part from buffer start to data head
            if (CDC_Transmit_FS(tx_buffer, tx_buffer_head) == USBD_OK) {
                tx_buffer_tail = tx_buffer_head;
            }
        }
    }
}

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t GRBLSerial::read() {
    uint8_t tail = rx_buffer_tail; // Temporary rx_buffer_tail (to optimize for volatile)
    if (rx_buffer_head == tail) {
        return SERIAL_NO_DATA;
    } else {
        uint8_t data = rx_buffer[tail];

        tail++;
        if (tail == RX_RING_BUFFER) { tail = 0; }
        rx_buffer_tail = tail;

        return data;
    }
}

void OnUsbDataRx(uint8_t* dataIn, uint8_t length) {
	uint8_t next_head;
    uint8_t data;

	// Write data to buffer unless it is full.
	while (length != 0) {
        data = *dataIn ++;

        // Pick off realtime command characters directly from the serial stream. These characters are
        // not passed into the main buffer, but these set system state flag bits for realtime execution.
        switch (data) {
            case CMD_RESET:         grbl.motion.reset(); break; // Call motion control reset routine.
            case CMD_STATUS_REPORT: grbl.system.set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
            case CMD_CYCLE_START:   grbl.system.set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
            case CMD_FEED_HOLD:     grbl.system.set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
            default : if (data > 0x7F) { // Real-time control characters are extended ACSII only.
                switch(data) {
                case CMD_SAFETY_DOOR:   grbl.system.set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
                case CMD_JOG_CANCEL:
                    if (grbl.sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
                        grbl.system.set_exec_state_flag(EXEC_MOTION_CANCEL);
                    }
                    break;
                #ifdef DEBUG
                    case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
                #endif
                case CMD_FEED_OVR_RESET: grbl.system.set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
                case CMD_FEED_OVR_COARSE_PLUS: grbl.system.set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
                case CMD_FEED_OVR_COARSE_MINUS: grbl.system.set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
                case CMD_FEED_OVR_FINE_PLUS: grbl.system.set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
                case CMD_FEED_OVR_FINE_MINUS: grbl.system.set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
                case CMD_RAPID_OVR_RESET: grbl.system.set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
                case CMD_RAPID_OVR_MEDIUM: grbl.system.set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
                case CMD_RAPID_OVR_LOW: grbl.system.set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
                case CMD_SPINDLE_OVR_RESET: grbl.system.set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
                case CMD_SPINDLE_OVR_COARSE_PLUS: grbl.system.set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
                case CMD_SPINDLE_OVR_COARSE_MINUS: grbl.system.set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
                case CMD_SPINDLE_OVR_FINE_PLUS: grbl.system.set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
                case CMD_SPINDLE_OVR_FINE_MINUS: grbl.system.set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
                case CMD_SPINDLE_OVR_STOP: grbl.system.set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
                case CMD_COOLANT_FLOOD_OVR_TOGGLE: grbl.system.set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
                #ifdef ENABLE_M7
                    case CMD_COOLANT_MIST_OVR_TOGGLE: grbl.system.set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
                #endif
                default: break;
                }
                // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
            } else { // Write character to buffer
                next_head = grbl.serial.rx_buffer_head + 1;
                if (next_head == RX_RING_BUFFER) { next_head = 0; }

                // Write data to buffer unless it is full.
                if (next_head != grbl.serial.rx_buffer_tail) {
                    grbl.serial.rx_buffer[grbl.serial.rx_buffer_head] = data;
                    grbl.serial.rx_buffer_head = next_head;
                }
            }
        }
        length--;
    }
}

void GRBLSerial::reset_read_buffer() {
    rx_buffer_tail = rx_buffer_head;
}
