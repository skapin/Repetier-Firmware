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

#ifndef __POLYBOX_H__
#define __POLYBOX_H__

#include "eps.h" // Extension pin system, allow to use external board as input/output device.
#include "lvm.h" // LabViewModule, handle light.
#include "DOF.h"
#include "Table.h"

#define POLY_MCODE_ISCLOGGED  676

/***********************************************************************
 *
 *    Variables
 *
 **********************************************************************/
class ChamberTempController;

extern volatile float filamentPrinted_lastCheck; ///< mm of filament printed since counting started but at previous check (n-1).
extern volatile long encoder_currentSteps; ///< steps count by encoder.
extern volatile long encoder_lastSteps; ///< steps count by encoder at last check.

extern volatile byte isClogged;
extern volatile uint8_t i2c_update_time;

extern ChamberTempController chamber;

extern Table table;

#define MODE_CN_ON          1
#define MODE_PRINTER_ON         2


extern volatile uint8_t polybox_mode;

/***********************************************************************
 *
 *    Initilization functions
 *
 **********************************************************************/

void init_polybox(); ///< Initilize polybox machine.
void init_printer(); ///< Initilize printer module.
void init_cn(); ///<  Initilize CNC module.
void init_atu_inter(); ///< Initilize button, inter and emergency stop.
void init_mon(); ///< Initilize monitor variables & modules.
void init_slaves(); ///< Initilize slave boards (pin, structures, class).
void init_scanner(); ///< Initilize scanner module.
void init_therm(); ///< Initilize all the thermistors.

/***********************************************************************
 *
 *    Functions.
 *
 **********************************************************************/

void encoder_incr(); ///< Incremente the encoder for clogging.
void pin_x_steps( int PIN , int steps ); ///< Move motor at PIn by the given steps.
void executeAction(int action, int param); ///<
void manage_mode();
void set_mode( uint8_t mode_flag);
bool is_printer_mode();
void set_atu ( bool enable );
void enable_PSU( bool enable );
void reset_slaves();
/***********************************************************************
 *
 *    Check & Get Functions
 *
 **********************************************************************/

byte is_clogged(); ///< Return true if the current extruder is clogged.
byte laser_detected( uint8_t laser_id ); ///< Return true if the laser is detected.
byte is_box_open(); ///< Return true if the box/printer/machine is Open.
byte is_ic_open(); ///< Return true if the electrical box is open.
byte bed_detected( uint8_t bed_id); ///< Return true if heated beds are detected.
uint8_t get_lub_level(); ///< Return the currrent lubricant level .

byte check_clogged(); ///< Check if the current extruder is clogged, using an encoder near the filament.
void check_boards_connected(); ///< Process the keep-alive protocol with slaves.
void check_all_ATU(); ///< Check all the ATU and stop the machine if needed.

/***********************************************************************
 *
 *    Manage Functions
 *
 **********************************************************************/

void manage_ic_temp();
void manage_pwm();




#endif // __POLYBOX_H__
