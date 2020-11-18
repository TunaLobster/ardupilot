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
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#pragma once

#include "AP_RCProtocol.h"
#include "SoftSerial.h"

// #define EXBus_CONTROL_PACKET_SIZE 38

// struct EXBus_Packet;

class AP_RCProtocol_EXBus : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_EXBus(AP_RCProtocol &_frontend, bool inverted);
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;

private:
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    bool exbus_decode(const uint8_t packet[56], uint16_t *values, uint16_t *num_values);

    bool inverted;
    // assuming that the speed is high
    // if the crc is wrong, swap to low speed (125000)
    SoftSerial ss{250000, SoftSerial::SERIAL_CONFIG_8N1};
    uint32_t saved_width;

    struct {
        uint8_t buf[56];
        uint8_t ofs;
        uint32_t last_byte_us;
    } byte_input;
};
