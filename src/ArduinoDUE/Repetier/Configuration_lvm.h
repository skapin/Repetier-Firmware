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

#ifndef __CONFIGURATION_LVM_H__
#define __CONFIGURATION_LVM_H__


#define LVM_FACES_NUM    			5 // number of Face used. (back, top, bot, right, left, front)
#define LVM_DEFAULT_H    			10 // default horizontale intensity
#define LVM_DEFAULT_V   			10 // default verticale intensity
#define LVM_DEFAULT_COLOR			0 // default color

#define LVM_BOARD_ID				1
#define TARGET_BOARD_INDEX			(PINS_PER_BOARD*LVM_BOARD_ID)
#define ID_BOARD_W				1
#define ID_BOARD_RGB				4
	
#define LVM_FACE_0_W_DETECTED			(34+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_0_RGB_DETECTED 		(32+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_0_H    			(13+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_0_V				(12+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_0_R				(46+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_0_G				(45+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_0_B				(44+PINS_PER_BOARD*ID_BOARD_RGB)
	
#define LVM_FACE_1_W_DETECTED			(36+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_1_RGB_DETECTED 		(38+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_1_H				(8+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_1_V				(11+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_1_R				(8+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_1_G				(10+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_1_B				(9+PINS_PER_BOARD*ID_BOARD_RGB)

#define LVM_FACE_2_W_DETECTED 			(38+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_2_RGB_DETECTED 		(22+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_2_H				(5+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_2_V				(6+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_2_R				(5+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_2_G				(7+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_2_B				(6+PINS_PER_BOARD*ID_BOARD_RGB)

#define LVM_FACE_3_W_DETECTED 			(40+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_3_RGB_DETECTED 		(50+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_3_H				(7+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_3_V				(46+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_3_R				(2+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_3_G				(4+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_3_B				(3+PINS_PER_BOARD*ID_BOARD_RGB)

#define LVM_FACE_4_W_DETECTED 			(42+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_4_RGB_DETECTED 		(52+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_4_H				(45+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_4_V				(44+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_4_R				(11+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_4_G				(13+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_4_B				(12+PINS_PER_BOARD*ID_BOARD_RGB)

#define LVM_FACE_5_W_DETECTED 			(-1+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_5_RGB_DETECTED 		(-1+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_5_H				(-1+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_5_V				(-1+PINS_PER_BOARD*ID_BOARD_W)
#define LVM_FACE_5_R				(-1+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_5_G				(-1+PINS_PER_BOARD*ID_BOARD_RGB)
#define LVM_FACE_5_B				(-1+PINS_PER_BOARD*ID_BOARD_RGB)


#define SELECT_DEFINE_FACE( face, type)  ( LVM_FACE_ ## face ## _ ## type ) // where type = L/H/R/G/B/DETECTED

#endif // __CONFIGURATION_H
