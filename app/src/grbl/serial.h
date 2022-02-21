/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
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

#ifndef serial_h
#define serial_h

#define SERIAL_NO_DATA  0xff
#define RX_BUFFER_SIZE  254
#define TX_BUFFER_SIZE  128    // Do not try 256 it will not work for STM32.

#ifdef __cplusplus
extern "C" {
#endif

#define RX_RING_BUFFER (RX_BUFFER_SIZE)
#define TX_RING_BUFFER (TX_BUFFER_SIZE)

void OnUsbDataRx(uint8_t* dataIn, uint8_t length);

class GRBLSerial {
public:
    uint8_t rx_buffer[RX_RING_BUFFER];
    uint8_t rx_buffer_head = 0;
    volatile uint8_t rx_buffer_tail = 0;

    uint8_t tx_buffer[TX_RING_BUFFER];
    uint8_t tx_buffer_head = 0;
    volatile uint8_t tx_buffer_tail = 0;

    // Writes one byte to the TX serial buffer. Called by main program.
    void write(uint8_t data);

    // Fetches the first byte in the serial read buffer. Called by main program.
    uint8_t read();

    // Reset and empty data in read buffer. Used by e-stop and reset.
    void reset_read_buffer();

    // Returns the number of bytes available in the RX serial buffer.
    uint8_t get_rx_buffer_available();

    // Returns the number of bytes used in the RX serial buffer.
    // NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
    uint8_t get_rx_buffer_count();

    // Returns the number of bytes used in the TX serial buffer.
    // NOTE: Not used except for debugging and ensuring no TX bottlenecks.
    uint8_t get_tx_buffer_count();

    void transmit();
};

#ifdef __cplusplus
}
#endif

#endif

