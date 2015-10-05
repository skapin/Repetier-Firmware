
#include "Face.h"

uint8_t Face::current_id = 0;

Face::Face()
{
    id = current_id;
    h = LVM_DEFAULT_H;
    v = LVM_DEFAULT_V;
    color.r = LVM_DEFAULT_COLOR;
    color.g = LVM_DEFAULT_COLOR;
    color.b = LVM_DEFAULT_COLOR;
    color.i = LVM_DEFAULT_COLOR;

    ++current_id;
}
void Face::init()
{
    switch ( id )
    {
        case 0:
            SETUP_PIN( SELECT_DEFINE_FACE( 0, R ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 0, G ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 0, B ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 0, H ), 67/*PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG*/ );
            SETUP_PIN( SELECT_DEFINE_FACE( 0, V ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 0, W_DETECTED ), PIN_TYPE_INPUT );
        break;
        case 1:
            SETUP_PIN( SELECT_DEFINE_FACE( 1, R ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 1, G ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 1, B ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 1, H ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 1, V ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 1, W_DETECTED ), PIN_TYPE_INPUT );
        break;
        case 2:
            SETUP_PIN( SELECT_DEFINE_FACE( 2, R ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 2, G ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 2, B ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 2, H ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 2, V ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 1, W_DETECTED ), PIN_TYPE_INPUT );
        break;
        case 3:
            SETUP_PIN( SELECT_DEFINE_FACE( 3, R ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 3, G ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 3, B ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 3, H ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 3, V ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 1, W_DETECTED ), PIN_TYPE_INPUT );
        break;
        case 4:
            SETUP_PIN( SELECT_DEFINE_FACE( 4, R ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 4, G ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 4, B ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 4, H ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 4, V ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 1, W_DETECTED ), PIN_TYPE_INPUT);
        break;
        case 5:
            SETUP_PIN( SELECT_DEFINE_FACE( 5, R ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 5, G ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 5, B ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 5, H ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );
            SETUP_PIN( SELECT_DEFINE_FACE( 5, V ), PIN_TYPE_OUTPUT| PIN_TYPE_ANALOG );

            SETUP_PIN( SELECT_DEFINE_FACE( 1, W_DETECTED ), PIN_TYPE_INPUT );
        break;
        default:
        break;
    }
}
uint8_t Face::get_detected()
{
    switch ( id )
    {
        case 0:
            READ_VPIN( SELECT_DEFINE_FACE( 0, W_DETECTED ));
        break;
        case 1:
            READ_VPIN( SELECT_DEFINE_FACE( 1, W_DETECTED ));
        break;
        case 2:
            READ_VPIN( SELECT_DEFINE_FACE( 2, W_DETECTED ));
        break;
        case 3:
            READ_VPIN( SELECT_DEFINE_FACE( 3, W_DETECTED ));
        break;
        case 4:
            READ_VPIN( SELECT_DEFINE_FACE( 4, W_DETECTED ));
        break;
        case 5:
            READ_VPIN( SELECT_DEFINE_FACE( 5, W_DETECTED ));
        break;
        default:
        return 1;
        break;
    }
}

// I shoudl WRAP #define....
void Face::set_color( Color &c )
{
     switch ( id )
    {
        case 0:
            WRITE_VPIN( SELECT_DEFINE_FACE( 0, R ), c.r );
            WRITE_VPIN( SELECT_DEFINE_FACE( 0, G ), c.g );
            WRITE_VPIN( SELECT_DEFINE_FACE( 0, B ), c.b );
        break;
        case 1:
            WRITE_VPIN( SELECT_DEFINE_FACE( 1, R ), c.r );
            WRITE_VPIN( SELECT_DEFINE_FACE( 1, G ), c.g );
            WRITE_VPIN( SELECT_DEFINE_FACE( 1, B ), c.b );
        break;
        case 2:
            WRITE_VPIN( SELECT_DEFINE_FACE( 2, R ), c.r );
            WRITE_VPIN( SELECT_DEFINE_FACE( 2, G ), c.g );
            WRITE_VPIN( SELECT_DEFINE_FACE( 2, B ), c.b );
        break;
        case 3:
            WRITE_VPIN( SELECT_DEFINE_FACE( 3, R ), c.r );
            WRITE_VPIN( SELECT_DEFINE_FACE( 3, G ), c.g );
            WRITE_VPIN( SELECT_DEFINE_FACE( 3, B ), c.b );
        break;
        case 4:
            WRITE_VPIN( SELECT_DEFINE_FACE( 4, R ), c.r );
            WRITE_VPIN( SELECT_DEFINE_FACE( 4, G ), c.g );
            WRITE_VPIN( SELECT_DEFINE_FACE( 4, B ), c.b );
        break;
        case 5:
            WRITE_VPIN( SELECT_DEFINE_FACE( 5, R ), c.r );
            WRITE_VPIN( SELECT_DEFINE_FACE( 5, G ), c.g );
            WRITE_VPIN( SELECT_DEFINE_FACE( 5, B ), c.b );
        break;
        default:
        break;
    }
    color = c;
}

Color Face::get_color()
{
    return color;
}

uint8_t Face::get_h_intensity()
{
    return h;
}

uint8_t Face::get_v_intensity()
{
    return v;
}

void Face::set_intensity( uint8_t nh, uint8_t nv)
{
    h = nh;
    v = nv;
    switch ( id )
    {
        case 0:
            WRITE_VPIN( SELECT_DEFINE_FACE( 0, V ), nv );
            WRITE_VPIN( SELECT_DEFINE_FACE( 0, H ), nh );
        break;
        case 1:
            WRITE_VPIN( SELECT_DEFINE_FACE( 1, V ), nv );
            WRITE_VPIN( SELECT_DEFINE_FACE( 1, H ), nh );
        break;
        case 2:
            WRITE_VPIN( SELECT_DEFINE_FACE( 2, V ), nv );
            WRITE_VPIN( SELECT_DEFINE_FACE( 2, H ), nh );
        break;
        case 3:
            WRITE_VPIN( SELECT_DEFINE_FACE( 3, V ), nv );
            WRITE_VPIN( SELECT_DEFINE_FACE( 3, H ), nh );
        break;
        case 4:
            WRITE_VPIN( SELECT_DEFINE_FACE( 4, V ), nv );
            WRITE_VPIN( SELECT_DEFINE_FACE( 4, H ), nh );
        break;
        case 5:
            WRITE_VPIN( SELECT_DEFINE_FACE( 5, V ), nv );
            WRITE_VPIN( SELECT_DEFINE_FACE( 5, H ), nh );
        break;
        default:
        break;
    }

}
