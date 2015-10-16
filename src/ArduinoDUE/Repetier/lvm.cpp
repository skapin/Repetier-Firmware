#include "lvm.h"

#include "Repetier.h"

Face faces[LVM_FACES_NUM];
bool lvm_is_on_state = true;

void lvm_check_inter()
{
    if ( READ_VPIN( INTER_LVM ) == 0 && lvm_is_on_state ) // 1 means OFF, 0 means ON 
    {
        Color c = { 0, 0, 0, 0 };
        lvm_set_global_color( c ); 
        lvm_set_global_intensity( 0, 0 );
        lvm_is_on_state = false;
    }
    else if ( !lvm_is_on )
    {
        lvm_is_on_state = true;
    }
    
}

void init_lvm()
{
	for ( uint8_t i =0; i < LVM_FACES_NUM  ; ++i)
    {
        faces[i].init();
    }
    lvm_set_unconnected_light();
	
}

bool lvm_is_on()
{
    return lvm_is_on;
}
void lvm_set_face_intensity(uint8_t id, uint8_t h, uint8_t v)
{
    if ( id >= 0 && id < LVM_FACES_NUM  )
    {
        faces[id].set_intensity( h,v );
    }
}
void lvm_set_face_color(  uint8_t id, Color &c )
{
    if ( id >= 0 && id < LVM_FACES_NUM )
    {
        faces[id].set_color( c );
    }	
}

void lvm_set_global_intensity( uint8_t h, uint8_t v )
{
    for ( uint8_t i =0; i < LVM_FACES_NUM  ; ++i)
    {
        faces[i].set_intensity( h, v );
    }
}
uint8_t lvm_get_face_h_intensity(uint8_t id)
{
    if ( id >= 0 && id < LVM_FACES_NUM )
    {
        return faces[id].get_h_intensity();
    }
}

uint8_t lvm_get_face_v_intensity(uint8_t id)
{
    if ( id >= 0 && id < LVM_FACES_NUM )
    {
        return faces[id].get_v_intensity();
    }
}

void lvm_set_global_color(  Color &c )
{
    for ( uint8_t i =0; i < LVM_FACES_NUM  ; ++i)
    {
        faces[i].set_color( c );
    }
}

Color lvm_get_global_color()
{
    return faces[0].get_color();
}

Color lvm_get_face_color(  uint8_t id )
{
    if ( id >= 0 && id < LVM_FACES_NUM )
    {
        return faces[id].get_color( );
    }	
}

uint8_t lvm_get_global_h_intensity()
{
    return faces[0].get_h_intensity();
}

uint8_t lvm_get_global_v_intensity()
{
    return faces[0].get_v_intensity();
}

void lvm_set_connected_light()
{
	Color c = { 0, 110, 80, 255 };
	lvm_set_global_color( c ); // blu/green
	lvm_set_global_intensity( 110, 110 );
}

void lvm_set_unconnected_light()
{
	//Color c = { 220, 0, 0, 255 };
	Color c = { 250, 85, 0, 255 };
	lvm_set_global_color( c ); 
	lvm_set_global_intensity( 60, 60 );
}

void lvm_set_light( Color &c)
{
	lvm_set_global_color( c ); 
	lvm_set_global_intensity( 10, 10 );
}
