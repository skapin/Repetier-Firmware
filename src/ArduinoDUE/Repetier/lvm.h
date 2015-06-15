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

#ifndef __LVM_H__
#define __LVM_H__

#include "Configuration_lvm.h"
#include "Face.h" 

extern Face faces[LVM_FACES_NUM];

void lvm_set_global_intensity( uint8_t h, uint8_t v ); 
void lvm_set_global_color(  Color &c );

void lvm_set_face_intensity(uint8_t id, uint8_t h, uint8_t v);
void lvm_set_face_color(  uint8_t id, Color &c );

Color lvm_get_global_color();
Color lvm_get_face_color(  uint8_t id );

uint8_t lvm_get_global_h_intensity();
uint8_t lvm_get_global_v_intensity();
uint8_t lvm_get_face_h_intensity(uint8_t id);
uint8_t lvm_get_face_v_intensity(uint8_t id);

void lvm_set_connected_light();
void lvm_set_unconnected_light();
void lvm_set_light( Color &c );

void init_lvm();

#endif // __LVM_H__
