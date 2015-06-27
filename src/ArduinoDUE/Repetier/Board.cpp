#include "Board.h"

uint8_t Board::next_id = 1;

Board::Board()
{
    for (uint8_t i = 0; i < PINS_PER_BOARD ; ++i )
    {
        pin_values[i] = new Pin();
    }
    this->connected = false;
    this->check_state = BOARD_W8_MASTER;
    this->i2c_id = next_id;
    next_id++;

}
Board::~Board()
{
// delete Pin()
}

void Board::manage_ping_pong()
{
    switch ( check_state )
    {
        case BOARD_WAIT_PONG :
            connected = false ;
            check_state = BOARD_WAIT_VERSION;
        break;
        case BOARD_OK :
            eps_send_action( this->i2c_id, EPS_PING );
            Wire.requestFrom((uint8_t)this->i2c_id, (uint8_t)1);
            while(Wire.available())
            {
                char c = Wire.I2C_READ_FUNC();
                if ( c == EPS_PONG )
                {
                    check_state = BOARD_OK;
                }
                else
                {
                    check_state = BOARD_WAIT_PONG;
                }
            }
        break;
        default:
            check_state = BOARD_W8_MASTER;
        break;
    }
}

void Board::check_pins_update(uint8_t type)
{
    #ifdef IS_SLAVE
    int value=0;
    for ( uint8_t i = 0; i < PINS_PER_BOARD ; ++i )
    {
        // we process only INPUT type pin. Output are controled by master.
        if ( pin_values[i]->GET_IO_TYPE == PIN_TYPE_INPUT )
        {
            // Standard pin
            if ( IS_STANDARD(pin_values[i]->type)  &&  ( ( pin_values[i]->type & PIN_TYPE_FAST_CHECK ) == type) )
            {
                if ( IS_DIGITAL( pin_values[i]->type ) )
                {
                    value = digitalRead( i );
                    if ( read_bpin( i ) != value )
                    {
                        Update u = { i, EPS_SET };
                        pin_update_queue.push( u );
                        write_bpin( i, value );
                    }
                }
            }
            // check Counter type
            else if ( IS_COUNTER(pin_values[i]->type)  )
            {
                // update counter value if FAST check
                CounterPin *c = static_cast<CounterPin*>(pin_values[i]);
                if (  (type == PIN_TYPE_FAST_CHECK) )
                {
                    value = digitalRead( i );
                    if ( value != c->lastState )
                    {
                        c->lastState =  !c->lastState;
                        c->updated = true; // set updated to true, so firmware will queue data
                        write_bpin( i, read_bpin( i )+1 );
                    }
                }
                // push updatequeue, so counter can be sent to master asap
                else if ( c->updated )
                {
                    c->updated = false;
                    Update u = { i, EPS_SET };
                    pin_update_queue.push( u );
                }
            }
        }
    }
    #endif
}
void Board::init_pin_table()
{
    for( int i =0; i<PINS_PER_BOARD ; ++i ) // Loop to send all entries
    {
        write_bpin_type( i, PIN_TYPE_INPUT );
        write_bpin( i, 0 );
    }
}
// READ
int Board::read_bpin( uint8_t pin )
{
    return pin_values[pin]->value;
}
uint8_t Board::read_bpin_type( uint8_t pin )
{
    return pin_values[pin]->type;
}
// WRITE
int Board::write_bpin(  uint8_t pin, int value )
{
    pin_values[pin]->value = value;
}
uint8_t Board::write_bpin_type( uint8_t pin, uint8_t type )
{
    pin_values[pin]->type = type;
}

void Board::push_all_pin( )
{
    for ( uint8_t pin=0; pin<PINS_PER_BOARD ; ++pin)
    {
		if ( pin == 20  || pin == 21) //SDA SCL
			continue;
		
        Update ut = {pin, EPS_SETUP};
        this->pin_update_queue.push( ut );

        Update u = {pin, EPS_SET};
        this->pin_update_queue.push( u );
    }

}
void Board::clear_queue()
{
    while( ! pin_update_queue.isEmpty() )
    {
        pin_update_queue.pop();
    }
}


void Board::manage_status( )
{
    switch ( this->check_state )
    {
        case BOARD_WAIT_VERSION :
            eps_send_version( this->i2c_id );
            Wire.requestFrom((uint8_t)this->i2c_id, (uint8_t)1);
            while ( Wire.available() )
            {
                uint8_t value = Wire.I2C_READ_FUNC();
                if ( value < EPS_MIN_VERSION_REQUIRE )
                {
                    this->check_state = BOARD_BAD_VERSION;
                }
                else
                {
                    this->check_state = BOARD_WAIT_INIT;
                }
            }
        break;
        case BOARD_WAIT_INIT :
            eps_send_action( this->i2c_id, EPS_INIT );
            Wire.requestFrom((uint8_t)this->i2c_id, (uint8_t)1);
            while ( Wire.available() )
            {
                uint8_t value = Wire.I2C_READ_FUNC();
                if ( value == EPS_INIT )
                {
                    this->check_state = BOARD_OK;
                    this->connected = true;
                    this->clear_queue();
                    this->push_all_pin();
                }
                else
                {
                    this->check_state = BOARD_WAIT_INIT;
                }
            }
        break;
        case BOARD_OK :
        break;
        case BOARD_W8_MASTER :
        {
            Wire.beginTransmission(this->i2c_id);
            uint8_t error = Wire.endTransmission();// endTransmission return 0 if the device is connected
            if ( error != 0 ) // no response from the slave
            {
                return;
            }
            else
            {
                check_state = BOARD_WAIT_VERSION;
            }
        }
        break;
        default:
        break;
    }
}
