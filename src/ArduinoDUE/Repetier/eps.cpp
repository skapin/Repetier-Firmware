#include "eps.h"
#include "Repetier.h"
#include <Wire.h>



Board boards[NUM_BOARD];
bool send_entries_flag = false;
int next_send_pin=0;

uint8_t i2c_current_board = 1;

///< ACK system
bool send_ack = false;
uint8_t i2c_destination = 0;
uint8_t i2c_current_buffer_size = 0;
uint8_t i2c_buffer[BUFFER_LENGTH];
uint32_t i2c_ack_resend_count = 0;
uint32_t i2c_timer = 0;
uint32_t i2c_timer_delay = 0;
uint8_t i2c_ack_reply_destination = 1;

void(* reset_board) (void) = 0;

void setup_slave_master(  ) {
    Wire.begin( BOARD_ID );          // join i2c bus
    //Wire.onReceive(i2cReceiveEvent); // register event
}


void eps_manage()
{
    #if EMULATE_SLAVE >= 1
        return;
    #endif

    i2c_timer_delay++;
    if ( i2c_timer_delay % EPS_BASETIME )
    {
        return;
    }

    /*uint32_t time;
    uint32_t start;
    start = micros();
*/
    uint8_t pong_flag = !(i2c_timer % PING_PONG_DELAY);
    uint8_t update_flag = !(i2c_timer % SEND_UPDATE_DELAY);
    uint8_t get_flag = !(i2c_timer % GET_DELAY);
    uint8_t token_flag = !(i2c_timer % TOKEN_DELAY);

    if ( i2c_current_board >= NUM_BOARD )
    {
        i2c_current_board = 1;
        i2c_timer++;
    }

    //for ( uint8_t i =1 ; i<NUM_BOARD ; ++i )
    //{
        // If board connected, do.
        //hardware check to do.
        HAL::pingWatchdog();
        if ( boards[i2c_current_board].connected )
        {
            if ( update_flag )
            {
                eps_send_board_update(i2c_current_board);
            }
            if ( get_flag )
            {
          /*      eps_send_action( i2c_current_board+1, EPS_GET );
                Wire.requestFrom(i2c_current_board+1,32);
                eps_process_incoming_datas( i2c_current_board );*/
            }
            if ( token_flag )
            {
                eps_send_action( i2c_current_board+1, EPS_TOKEN );
                HAL::pingWatchdog();
                if ( Wire.requestFrom(i2c_current_board+1,32) )
                {
                    eps_process_incoming_datas( i2c_current_board );
                }
                else
                {
                    Com::printFLN(Com::tError, " I2C:requestFrom " );
                }
            }
            if ( pong_flag )
            {
                boards[i2c_current_board].manage_ping_pong();
            }

        }
        else
        {
            // Process HandShake states
            if ( !(i2c_timer % HANDSHAKE_TIME) )
            {
                boards[i2c_current_board].manage_status( );
            }
        }
    //eps_check_ack();

    i2c_current_board++;
    HAL::pingWatchdog();
   /* time = micros() - start;
    Com::printF(Com::tError,time);
    Com::printF(" D: ");
    Com::printF(Com::tError,i2c_timer_delay);
    Com::printFLN(".");*/
}

void eps_process_incoming_datas(uint8_t board)
{
    byte pin = 0;
    int value = 0;
    byte action = 0;

    while ( Wire.available() )
    {
        action = Wire.I2C_READ_FUNC();
        if ( action == EPS_SET ) {
            pin = Wire.I2C_READ_FUNC();
            value = Wire.I2C_READ_FUNC() << 8;
            value += Wire.I2C_READ_FUNC();
            boards[board].write_bpin( pin, value );
        }
        else if ( action == EPS_SETUP )
        {
            pin = Wire.I2C_READ_FUNC();
            value = Wire.I2C_READ_FUNC();
            boards[board].write_bpin_type( pin, value );
        }
        else if ( action == EPS_RESET ) // Slave is not sync. anymore, need a protocol restart. Go back to init phase for this board. The reason can be a slave restart
        {
            boards[board].connected = false;
            boards[board].check_state = BOARD_W8_MASTER;
        }
        else if ( action == EPS_SET_END )
        {
        }
        else
        {
        }
    }
}
void eps_check_ack()
{

}
void eps_ack_reset()
{
}

int get_update_queues_size()
{
    uint8_t size = 0;
    for ( uint8_t i =0; i<NUM_BOARD ; ++i)
    {
        size += boards[i].pin_update_queue.count();
    }
    return size;
}

// READ
int eps_read_vpin_value( int pin )
{
    if ( pin >= (PINS_PER_BOARD*NUM_BOARD) ) // error
    {
        return 0;
    }
    else if ( pin < PINS_PER_BOARD ) // Master board
    {
        /*if ( IS_DIGITAL(boards[0].pin_values[pin]->type) ) // digital
        {
            boards[0].write_bpin( pin, HAL::digitalRead(pin) ) ;
        }*/
        //
        // handle ANALOG ?????????
        //

        boards[0].write_bpin( pin, HAL::digitalRead(pin) ) ;
        return boards[0].read_bpin( pin );
    }
    else // virtual pin / slave boards
    {
        return boards[vpin2board(pin)].read_bpin( vpin2bpin(pin) );
    }

}
 uint8_t eps_read_vpin_type( int pin )
{
    return boards[vpin2board(pin)].read_bpin_type( vpin2bpin(pin) );
}

int board_read_bpin_value( uint8_t b, uint8_t pin )
{
    return boards[b].read_bpin( pin );
}
 uint8_t board_read_bpin_type( uint8_t b, uint8_t pin )
{
    return boards[b].read_bpin_type( pin );
}

// WRITE

void eps_set_vpin_value( int pin, int value)
{
    if ( pin >= (PINS_PER_BOARD*NUM_BOARD) )
    {
        return;
    }
    else if ( pin < PINS_PER_BOARD ) // Master board
    {
        //analogWrite( pin, value );
        HAL::digitalWrite( pin, value);
        boards[0].write_bpin( pin, value );
    }
    else // start virtual pin (i.e other Arduino board)
    {
        uint8_t real_pin = vpin2bpin(pin);
        uint8_t board_n = vpin2board(pin);

        // same value as before, we stop now, no time to waste
        if ( value == boards[board_n].pin_values[real_pin]->value )
        {
            return;
        }
        boards[board_n].write_bpin( real_pin, value );
        #if EMULATE_SLAVE <= 1
        Update u = {real_pin, EPS_SET};
        boards[board_n].pin_update_queue.push( u );
        #endif

    }
}

void eps_write_vpin_type( int pin, uint8_t type)
{
    if ( pin >= (PINS_PER_BOARD*NUM_BOARD) )
    {
        return;
    }
    else if ( pin >= PINS_PER_BOARD ) // start virtual pin (i.e other Arduino board)
    // So we need to update the fifo queue.
    {
        #if EMULATE_SLAVE <= 1
        uint8_t real_pin = vpin2bpin(pin);
        uint8_t board_n = vpin2board(pin);
        Update u = {real_pin, EPS_SETUP};
        boards[board_n].pin_update_queue.push( u );
        boards[board_n].pin_values[real_pin]->type = type ;
        #endif
    }
    else
    {
        if ( (type & PIN_TYPE_IO_MASK )== PIN_TYPE_OUTPUT )
        {
            SET_OUTPUT_VAR(pin);
        }
        else if ( (type & PIN_TYPE_IO_MASK ) == PIN_TYPE_INPUT )
        {
            SET_INPUT_VAR(pin);
        }
        else
        {
            return;
        }
        //pinMode(real_pin, type & PIN_TYPE_IO_MASK );
        boards[0].pin_values[pin]->type = type ;
    }

}

void eps_send_action( uint8_t dest, uint8_t action )
{
    Wire.beginTransmission( dest );
    Wire.I2C_WRITE_FUNC( BOARD_ID );
    Wire.I2C_WRITE_FUNC( action );
    Wire.endTransmission();
    delayMicroseconds(2);  //needs at least 1.3us free time between start and stop /** Credits : http://www.bajdi.com/arduino-sketches/mag3110.ino **/
}

void eps_send_version( int dest )
{
    Wire.beginTransmission( dest );
    Wire.I2C_WRITE_FUNC( BOARD_ID );
    Wire.I2C_WRITE_FUNC( EPS_VERSION );
    Wire.I2C_WRITE_FUNC( EPS_PROTOCOL_VERSION );
    Wire.endTransmission();
    delayMicroseconds(2);  //needs at least 1.3us free time between start and stop /** Credits : http://www.bajdi.com/arduino-sketches/mag3110.ino **/
}

void eps_send_board_update(uint8_t dest)
{
    if ( ! boards[dest].pin_update_queue.isEmpty() )
    {
        Update up;
        byte count=2;
        int value=0;
        Wire.beginTransmission( dest+1 ); // Open th I2C link (i.e master)
        Wire.I2C_WRITE_FUNC( BOARD_ID );
        Wire.I2C_WRITE_FUNC( EPS_ALL );

        while ( !boards[dest].pin_update_queue.isEmpty() && count < BUFFER_LENGTH-2)
        {
            up = boards[dest].pin_update_queue.pop();
            Wire.I2C_WRITE_FUNC( up.type );
            Wire.I2C_WRITE_FUNC( up.pin );
            if ( up.type == EPS_SET ) {
                value = boards[dest].read_bpin(up.pin);
                Wire.I2C_WRITE_FUNC( (byte) ( ( value & 0xFF00)>>8 ) );
                Wire.I2C_WRITE_FUNC( (byte) ( value & 0x00FF ) );
                count = count+2+2;
            }
            else if ( up.type == EPS_SETUP )
            {
                Wire.I2C_WRITE_FUNC( boards[dest].read_bpin_type(up.pin) );
                count = count+2+1;
            }
        }
        Wire.endTransmission();
    }
}

void eps_push_all_pin()
{
    for ( uint8_t board_idx=1; board_idx<NUM_BOARD ; ++board_idx)
    {
        boards[board_idx].push_all_pin();
    }
}

void eps_clear_queue()
{
    for ( uint8_t board_idx=1; board_idx<NUM_BOARD ; ++board_idx)
    {
        boards[board_idx].clear_queue();
    }
}
/*
byte eps_send_board_value(uint8_t dest)
{
    uint8_t j;
    Wire.beginTransmission( dest+1 ); // Open th I2C link
    ///< ACK
    i2c_destination = dest+1;
    Wire.I2C_WRITE_FUNC( BOARD_ID );
    Wire.I2C_WRITE_FUNC( EPS_SET );
    for( j =0; j< ((BUFFER_LENGTH-2)/3) ; ++j ) // Loop to send all entries. TWI buffer is only 32 Byte. We send 3 byte during each "for" loop
    {
        if ( ( boards[dest].read_bpin_type(next_send_pin)  & PIN_TYPE_IO_MASK ) == PIN_TYPE_OUTPUT )
        {
            Wire.I2C_WRITE_FUNC( next_send_pin );
            Wire.I2C_WRITE_FUNC( (byte) ((boards[dest].pin_values[next_send_pin]->value & 0xFF00) >> 8)  );
            Wire.I2C_WRITE_FUNC( (byte) (boards[dest].pin_values[next_send_pin]->value & 0x00FF) );
        }
        next_send_pin = (next_send_pin+1) % PINS_PER_BOARD;
        if ( next_send_pin == 0 )
        {
            Wire.endTransmission();
            return false;
        }
    }
    Wire.endTransmission();
    return true; // true
}*/

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void i2cReceiveEvent(int howMany)
{
    /*
    int sender = Wire.I2C_READ() -1;
    byte action = Wire.I2C_READ();

    if ( action == EPS_SET )
    {
        byte pin = 0;
        int value = 0;
        while ( Wire.available() )
        {
            pin = Wire.I2C_READ();
            value = Wire.I2C_READ() << 8;
            value += Wire.I2C_READ();
            boards[sender].write_bpin( pin, value );
        }
    }
    else if (action == EPS_PONG && boards[sender].check_state==BOARD_WAIT_PONG )
    {
        boards[sender].check_state = BOARD_OK;
    }
    else if (action == EPS_RESET )
    {
        reset_board();
    }
    else
    {
    }
    */
}

/*
#ifdef IS_MASTER && ARDUINO < 100
void operator delete(void * p) // or delete(void *, std::size_t)
{
  free(p);
}
void * operator new(size_t size)
{
  return malloc(size);
}
#endif
*/
