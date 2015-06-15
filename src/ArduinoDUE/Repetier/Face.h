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

#ifndef __FACE_H__
#define __FACE_H__

#include <inttypes.h>
#include "Configuration_lvm.h" 
#include "Color.h" 
#include "eps.h" 

class Face
{
public:
    Face();    
    
    void set_color( Color &c );
    void set_intensity( uint8_t nh, uint8_t nv);
    Color get_color();
    uint8_t get_h_intensity();
    uint8_t get_v_intensity();
    uint8_t get_detected();
    void init();
    
    private:
    uint8_t id; // id auto-incr.
    uint8_t h; // Horizontale
    uint8_t v; // Verticale
    Color color; // Color
    
    static uint8_t current_id;
};
#endif // __FACE_H__
