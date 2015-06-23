/*
    This file is part of Polybox.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

/**
 * Configuration file.
 * Variable you can adjust, like delay timer.
 *
 * **/

#ifndef __CONFIGURATION_EPS_H__
#define __CONFIGURATION_EPS_H__

//#define IS_SLAVE            1
#define IS_MASTER            1

#ifdef IS_SLAVE
 #define board boards[0]
 #define WRITE_PIN(p,v) boards[0].write_bpin(p,v)
 #define PIN_MODE(p,t) boards[0].write_bpin_type(p,t)
#endif

#define BOARD_ID            1
#define NUM_BOARD           5 // number of board
#define PINS_PER_BOARD      (54+16)

#define BUFFER_LENGTH       32


#define EPS_BASETIME        (410) // ~10 ms

#define HANDSHAKE_TIME      (1)
#define PING_PONG_DELAY     (500)    // delay for pingpong process
#define GET_DELAY           (2000)    // delay for all pin update slave -> master
#define SEND_UPDATE_DELAY   (1)      // delay for master to slave
#define TOKEN_DELAY         2        // delay for slave to Master

// emulate slavering system/ Dont send I2C or push data
// 0 = use full system
// 1 = just don't send/read I2C data
// 2 = dont send/read, and dont push update in queue
#define EMULATE_SLAVE       0
#endif // __CONFIGURATION_EPS_H__
