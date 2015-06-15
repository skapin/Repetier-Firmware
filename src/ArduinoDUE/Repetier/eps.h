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

#ifndef __eps_H__
#define __eps_H__

/***********************************************************************
 * EPS : Extension Pin System
 * 
 * Provide more pin for a arduino master. You can use as much Arduino 
 * slave you like.
 * 
 * Slave doesnt know his pin-mapping, and all the configuration is 
 * provided by the Master. A time_out system detect when a slave/master
 *  is deconnected.
 * 
 *  A Pin can be OUT or INPUT.
 * 
 *  INPUT pin are monitored and sent to master periodicaly when values
 * changed. Futhermore, user can specify FAST or STANDARD (default) 
 * monitoring. COUNTER pin is inherited from PIN, and provide a counter.
 * Check doc/code for futher information.
 * 
 * Handshake :
 * 
 * Wait > Check Version > Init > Running
 * 
 * ********************************************************************/

#if defined(ARDUINO) && ARDUINO >= 100
#define I2C_WRITE write 
#define I2C_READ read
#include "Arduino.h"
#else
#include "WProgram.h"
#define I2C_WRITE send
#define I2C_READ receive
#endif

#include <inttypes.h>


#include "Configuration_eps.h"
#include "Board.h"
#include <Wire.h>


/***********************************************************************
 * 
 * Defines
 * 
 * ********************************************************************/

#define EPS_PROTOCOL_VERSION 1
#define EPS_MIN_VERSION_REQUIRE   1

#define EPS_SETUP           1
#define EPS_SET             2
#define EPS_GET             3
#define EPS_ALL             4
#define EPS_PING            5
#define EPS_PONG            6
#define EPS_RESET           7
#define EPS_VERSION         8
#define EPS_INIT            9
#define EPS_TOKEN			10
#define EPS_ACK				31

#define MASTER_ID           1

#define WRITE_VPIN( p, v) eps_set_vpin_value( p, v)
#define READ_VPIN( p ) eps_read_vpin_value( p )
#define READ_VPIN_MODE( p ) eps_read_vpin_type( p )
#define VPIN_MODE( p, v) eps_write_vpin_type( p, v)

class Board;

/***********************************************************************
 * 
 * Extern Variables & Variables
 * 
 * ********************************************************************/

extern Board boards[NUM_BOARD];
extern bool send_entries_flag;

inline uint8_t vpin2bpin(int vpin)
{
	if ( vpin < (PINS_PER_BOARD*2) )
	{
		vpin -= PINS_PER_BOARD;
	}
	else if ( vpin < (PINS_PER_BOARD*3) )
	{
		vpin -= (PINS_PER_BOARD*2);
	}
	else if ( vpin < (PINS_PER_BOARD*4) )
	{
		vpin -= (PINS_PER_BOARD*3);
	}
	else if ( vpin < (PINS_PER_BOARD*5) )
	{
		vpin -= (PINS_PER_BOARD*4);
	}
}
inline uint8_t vpin2board(int16_t vpin)
{
	if ( vpin < (PINS_PER_BOARD*2) )
	{
		return 1;
	}
	else if ( vpin < (PINS_PER_BOARD*3) )
	{
		return  2;
	}
	else if ( vpin < (PINS_PER_BOARD*4) )
	{
		return 3;
	}
	else if ( vpin < (PINS_PER_BOARD*5) )
	{
		return 4;
	}
	
}

/***********************************************************************
 * 
 * Read/Write Functions
 * 
 * ********************************************************************/

// READ
int board_read_bpin_value( uint8_t b, uint8_t pin );
uint8_t board_read_bpin_type( uint8_t b, uint8_t pin );

int eps_read_vpin_value( int pin );
uint8_t eps_read_vpin_type( int pin );

// WRITE
void eps_set_vpin_value( int pin, int value);
void eps_write_vpin_type( int pin, uint8_t type);

/***********************************************************************
 * 
 * Functions
 * 
 * ********************************************************************/
 
void eps_manage();

void eps_send_action( uint8_t dest, uint8_t action );
void eps_send_version( int dest );

void eps_send_board_update(uint8_t dest);
byte eps_send_board_value(uint8_t dest);

void eps_push_all_pin();
void eps_clear_queue();

void eps_process_incoming_datas(uint8_t board);

void i2cReceiveEvent(int howMany);
void setup_slave_master();

int get_update_queues_size();

/***********************************************************************
 * 
 * ACK 
 * 
 * ********************************************************************/
 
void eps_check_ack();
void eps_ack_reset();
/*
#ifdef IS_MASTER && ARDUINO < 100 // welll...arduino dont know new/delete...but it's ok for INO...
 void operator delete(void * p);
 void * operator new(size_t size); 
#endif
*/

#endif
