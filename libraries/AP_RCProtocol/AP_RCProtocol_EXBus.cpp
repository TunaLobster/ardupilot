/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 /*
  Jeti EXBus implementation, with thanks to Jeti for
  specification
 */

 /*
  Example channel value packet provided by Jeti

  0x3E 0x03 0x28 0x06 0x31 0x20 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x82 0x1F 0x4F 0xE2
  0x3E 0x03–Packet header that forbids answering
  0x28–Message length(40)
  0x06–Packet ID
  0x31–The data identifier–channel values
  0x20–Length of data blocks (32) -16 channelsx 2B
  0x1F82–Value of the 1st channel(8066)/8000000 = 1,00825ms...
  0xE24F-CRC16-CCITT
 */

#include "AP_RCProtocol_EXBus.h"

#define EXBUS_FRAME_SIZE		56 // maximum frame size if the user selected 24 channel mode
#define EXBUS_INPUT_CHANNELS	16

// constructor
AP_RCProtocol_EXBus::AP_RCProtocol_EXBus(AP_RCProtocol &_frontend, bool _inverted) :
    AP_RCProtocol_Backend(_frontend),
    inverted(_inverted)
{}

// decode a full EXBus packet
bool AP_RCProtocol_EXBus::exbus_decode(const uint8_t packet[56], uint16_t *values, uint16_t *num_values)
{
    /* check that this is a channel data packet */
    if (packet[0] != 0x3e) {
        return false;
    }
    if (packet[1] != 0x01 || packet[1] != 0x03) {
        return false;
    }
    
    /* check that we are getting a 16 channel packet. We don't support more than 16 right now. */
    // packet length (should be 40)
    if (packet[2] != 0x28) {
        return false;
    }
    // packet[3] is packet id. Use that for logging?
    // packet data identifier. 0x31 is channel values
    if (packet[4] != 0x31) {
        return false;
    }

    // check the crc
    uint16_t sent_crc = packet[39] << 8 | packet[38]; // LSB, MSB sequence
    uint16_t calc_crc = get_crc16z(packet, 40);
    if (sent_crc != calc_crc) {
        return false;
    }
    

    /* now for all the channel data. 2 bytes into 16-bit channel value with 16 channels.*/
    
}

// crc code from Jeti EXBus v1.21 doc
// I can tell this is weird and possible garbage
uint16_t crc_ccitt_update(uint16_t crc, uint8_t data)
{
    uint16_t ret_val;
    data ^= (uint8_t) (crc) & (uint8_t) (0xff);
    data ^= data << 4;
    ret_val = ((((uint16_t) data << 8) | ((crc & 0xFF00) >> 8))
              ^ (uint8_t) (data >> 4)
              ^ ((uint16_t) data << 3));
    return ret_val;
}

uint16_t get_crc16z(const uint8_t *p, uint16_t len)
{
    uint16_t crc16_data = 0;

    while(len--) {
        // crc16_data = crc16_update(crc16_data, p[0]); // this is how Jeti wrote it
        crc16_data = crc_ccitt_update(crc16_data, p[0]);
        p++;
    }
}

/*
  process a EXBUS input pulse of the given width
 */
void AP_RCProtocol_EXBus::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint32_t w0 = width_s0;
    uint32_t w1 = width_s1;
    if (inverted) {
        w0 = saved_width;
        w1 = width_s0;
        saved_width = width_s1;
    }
    uint8_t b;
    if (ss.process_pulse(w0, w1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

// support byte input
void AP_RCProtocol_EXBus::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    // gap before a new EXBus frame is not defined. Just have to look for the correct header.

    // const bool have_frame_gap = (timestamp_us - byte_input.last_byte_us >= 2000U);
    // byte_input.last_byte_us = timestamp_us;

    // if (have_frame_gap) {
    //     // if we have a frame gap then this must be the start of a new
    //     // frame
    //     byte_input.ofs = 0;
    // }
    // if (b != 0x3E && byte_input.ofs == 0) {
    //     // definately not EXBux, missing header byte
    //     return;
    // }
    // if (byte_input.ofs == 0 && !have_frame_gap) {
    //     // must have a frame gap before the start of a new SBUS frame
    //     return;
    // }

    byte_input.buf[byte_input.ofs++] = b;

    if (byte_input.ofs == sizeof(byte_input.buf)) {
        log_data(AP_RCProtocol::EXBUS, timestamp_us, byte_input.buf, byte_input.ofs);
        uint16_t values[EXBUS_INPUT_CHANNELS];
        uint16_t num_values=0;
        bool sbus_failsafe = false;
        bool sbus_frame_drop = false;
        if (exbus_decode(byte_input.buf, values, &num_values) &&
            num_values >= MIN_RCIN_CHANNELS) {
            add_input(num_values, values, sbus_failsafe);
        }
        byte_input.ofs = 0;
    }
}

// support byte input
void AP_RCProtocol_EXBus::process_byte(uint8_t b, uint32_t baudrate)
{
    if (baudrate != 125000 || baudrate != 250000) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}