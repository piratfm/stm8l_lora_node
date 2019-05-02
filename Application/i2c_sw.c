
#include "stm8l15x.h"
#include "stm8l15x_it.h"    /* SDCC patch: required by SDCC for interrupts */
#include "stdio.h"
#include "main.h"

//#define I2CSPEED 25
#define I2CSPEED 20

#define I2C_SET_SCL GPIO_SetBits(I2C_PORT, I2C_SCL_PIN)
#define I2C_CLR_SCL GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN)

#define I2C_SET_SDA GPIO_SetBits(I2C_PORT, I2C_SDA_PIN)
#define I2C_CLR_SDA GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN)

#define I2C_DELAY { volatile uint16_t v;  for (v=0; v < I2CSPEED/2; v++) nop(); }

#define I2C_READ_SDA GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN)
#define I2C_READ_SCL GPIO_ReadInputDataBit(I2C_PORT, I2C_SCL_PIN)

#define DEBUGPRINT(str) printf(str"\r\n")


void i2c_init()
{
  GPIO_SetBits(I2C_PORT, I2C_SDA_PIN | I2C_SCL_PIN);
  
  GPIO_Init(I2C_PORT, I2C_SCL_PIN, GPIO_Mode_Out_OD_HiZ_Slow);
  GPIO_Init(I2C_PORT, I2C_SDA_PIN, GPIO_Mode_Out_OD_HiZ_Slow);
  DEBUGPRINT("I2C: init");
  I2C_DELAY;
}


// Initiating an I2C start condition:
void i2c_start_condition()
{
    I2C_SET_SCL;
    I2C_SET_SDA;
    I2C_DELAY;
    I2C_CLR_SDA;
    I2C_DELAY;
}

// Initiating an I2C stop condition:
void i2c_stop_condition()
{
    I2C_CLR_SCL;
    I2C_CLR_SDA;
    I2C_DELAY;
    I2C_SET_SCL;
    I2C_DELAY;
    I2C_SET_SDA;
    I2C_DELAY;
}

// Writing a byte with I2C:
bool i2c_write_byte( uint8_t B,
                     bool start,
                     bool stop )
{
  bool nack = false;
  if( start )
    i2c_start_condition();

  I2C_DELAY;
  uint8_t i;
  for( i = 0; i < 8; i++ ) {
    I2C_CLR_SCL;

    // write the most-significant bit
    if(B & 0x80)
      I2C_SET_SDA;
    else
      I2C_CLR_SDA;

    I2C_DELAY;
    I2C_SET_SCL;
    I2C_DELAY;
    B <<= 1;
  }
    
  I2C_CLR_SCL;
  I2C_SET_SDA;
  I2C_DELAY;
  I2C_SET_SCL;
  I2C_DELAY;
  if(I2C_READ_SDA)
    nack = true;
  I2C_CLR_SCL;

  if( stop )
    i2c_stop_condition();
  return nack;
}

// Reading a byte with I2C:
uint8_t i2c_read_byte( bool ack,
                       bool stop )
{
    uint8_t B = 0;

    I2C_SET_SDA;

    uint8_t i;
    for( i = 0; i < 8; i++ )
    {
      I2C_CLR_SCL;
      I2C_DELAY;
      I2C_SET_SCL;
      I2C_DELAY;

      B <<= 1;
      if(I2C_READ_SDA)
        B |= 1;
    }

    I2C_CLR_SCL;
    if(ack)
      I2C_CLR_SDA;
    else
      I2C_SET_SDA;

    I2C_DELAY;
    I2C_SET_SCL;
    I2C_DELAY;
    I2C_CLR_SCL;
    I2C_SET_SDA;

    if( stop ) i2c_stop_condition();
    return B;
}


// Sending a byte of data with I2C:
bool i2c_write_byte_data( uint8_t address,
                         uint8_t reg,
                         uint8_t data )
{
    if( !i2c_write_byte( address << 1, true, false ) )   // start, send address, write
    {
        if( !i2c_write_byte( reg, false, false ) )   // send desired register
        {
            if( !i2c_write_byte( data, false, true ) ) return true;   // send data, stop
        } else {
          DEBUGPRINT("I2C W: can't write 2nd byte");
        }
    } else {
      DEBUGPRINT("I2C W: can't write 1st byte");
    }

    i2c_stop_condition();
    DEBUGPRINT("I2C W: stopping");
    return false;
}

// Receiving a byte of data with I2C:
uint8_t i2c_read_byte_data( uint8_t address,
                               uint8_t reg )
{
    if( !i2c_write_byte( address << 1, true, false ) )   // start, send address, write
    {
        if( !i2c_write_byte( reg, false, false ) )   // send desired register
        {
            if( !i2c_write_byte( ( address << 1) | 0x01, true, false ) )   // start again, send address, read
            {
              return i2c_read_byte( false, true );   // read data
            } else {
              DEBUGPRINT("I2C R: can't write 3rd byte");
            }
        } else {
          DEBUGPRINT("I2C R: can't write 2nd byte");
        }
    } else {
      DEBUGPRINT("I2C R: can't write 1st byte");
    }

    i2c_stop_condition();
    DEBUGPRINT("I2C R: stopping");
    return 0;   // return zero if NACKed
}

uint16_t i2c_read_short_data( uint8_t address,
                               uint8_t reg, bool swap )
{
#if 0
  uint16_t val=0;
  
    if( !i2c_write_byte( address << 1, true, false ) )   // start, send address, write
    {
        if( !i2c_write_byte( reg, false, false ) )   // send desired register
        {
            if( !i2c_write_byte( ( address << 1) | 0x01, true, false ) )   // start again, send address, read
            {
                val=((uint16_t)i2c_read_byte( true, false ) << 8) |\
                     i2c_read_byte( false, true );
                return val;
            }
        }
    }

    i2c_stop_condition();
    return 0;   // return zero if NACKed
#endif
  i2c_write_byte( address << 1, true, false );
  i2c_write_byte( reg, false, false );
  i2c_write_byte( ( address << 1) | 0x01, true, false );
  //uint16_t val = i2c_read_byte( true, false );
  //val = val << 8 |  i2c_read_byte( false, true );
  if(swap) {
    return        i2c_read_byte( true, false ) | \
       ((uint16_t)i2c_read_byte( false, true )) << 8;
  } else {
    return ((uint16_t)i2c_read_byte( true, false )) << 8 | \
                      i2c_read_byte( false, true );
  }
}

uint32_t i2c_read_3bytes_data( uint8_t address,
                               uint8_t reg )
{
#if 0
  uint16_t val=0;
    if( !i2c_write_byte( address << 1, true, false ) )   // start, send address, write
    {
        if( i2c_write_byte( reg, false, false ) )   // send desired register
        {
            if( !i2c_write_byte( ( address << 1) | 0x01, true, false ) )   // start again, send address, read
            {
                val=((uint32_t)i2c_read_byte( true, false ) << 16) | ((uint32_t)i2c_read_byte( true, false ) << 8) | \
                     i2c_read_byte( false, true );
                return val;
            }
        }
    }

    i2c_stop_condition();
    return 0;   // return zero if NACKed
#endif
  i2c_write_byte( address << 1, true, false );
  i2c_write_byte( reg, false, false );
  i2c_write_byte( ( address << 1) | 0x01, true, false );
  return ((uint32_t)i2c_read_byte( true, false )) << 16 | \
         ((uint32_t)i2c_read_byte( true, false )) << 8  | \
                    i2c_read_byte( false, true );
}

void i2c_scan()
{
  printf("    00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n\r");
  uint8_t i;
  for (i=0;i<128;i++) {
    uint8_t val = i2c_write_byte( (i << 1) | 0x00, true, true);
    if( i % 16 == 0)
      printf("%02x: ", i);
    if(val)
      printf("-- ");
    else
      printf("%02x ", i);
    if( i % 16 == 15)
      printf("\n\r");
  }
}
