/*
 *****************************************************************************
 * Copyright by ams OSRAM AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/semphr.h"
 #include "freertos/queue.h"
 #include "freertos/event_groups.h"
 #include "esp_log.h"
 #include "esp_err.h"
 #include "driver/gpio.h"
 
 #include "gpi2c.h"

#include "../tmf8828_shim.h"
#include "../tmf8828.h"

#define TAG "tmf882xdriver"
// #include "Arduino.h"
// #include <Wire.h>

extern tmf8828Driver tmf8828[];

void delayInMicroseconds ( uint32_t wait )
{
  // ESP_LOGI(TAG, "Wait Microseconds: %i", (int)wait);
  uint32_t delaytime = wait/portTICK_PERIOD_MS/1000;
  delaytime = delaytime < 1 ? 1:delaytime;
  // ESP_LOGI(TAG, "Waittime in ticks: %i", (int)delaytime);
  vTaskDelay(delaytime);
  // delayMicroseconds( wait );
}

uint32_t getSysTick ( )
{
  return xTaskGetTickCount() * 1000;
  // return micros( );
}

uint8_t readProgramMemoryByte ( const uint8_t * ptr )
{
  uint32_t address = (uint32_t)ptr;
  // return pgm_read_byte( address );
  return *ptr;
  // return 0;
}

void enablePinHigh ( void * dptr )
{
  if ( dptr == &(tmf8828[0]) )
  {
    // digitalWrite( ENABLE_PIN, HIGH ); 
    gpio_set_level((gpio_num_t)ENABLE_PIN, 1);
  }
  else if ( dptr == &(tmf8828[1]) )
  {
    gpio_set_level( (gpio_num_t)ALT_ENABLE_PIN, 1 ); 
  }
  else if ( dptr == &(tmf8828[2]) )
  {
    gpio_set_level( (gpio_num_t)ALT2_ENABLE_PIN, 1 ); 
  }
}

void enablePinLow ( void * dptr )
{
  if ( dptr == &(tmf8828[0]) )
  {
    gpio_set_level( (gpio_num_t)ENABLE_PIN, 0 ); 
  }
  else if ( dptr == &(tmf8828[1]) )
  {
    gpio_set_level( (gpio_num_t)ALT_ENABLE_PIN, 0 ); 
  }
  else if ( dptr == &(tmf8828[2]) )
  {
    gpio_set_level( (gpio_num_t)ALT2_ENABLE_PIN, 0 ); 
  }
}

void configurePins ( void * dptr )
{
  (void)dptr; // not used here
  // configure ENABLE pin and interupt pin
  pinOutput( ENABLE_PIN );
  pinInput( INTERRUPT_PIN );
  pinOutput( ALT_ENABLE_PIN );
  pinInput( ALT_INTERRUPT_PIN );
  pinOutput( ALT2_ENABLE_PIN );
  pinInput( ALT2_INTERRUPT_PIN );
#if ( defined( USE_INTERRUPT_TO_TRIGGER_READ ) && (USE_INTERRUPT_TO_TRIGGER_READ != 0) )                       // if 3 TMF8828 are used, the PIN2 is used for INT of TMF8828[2]
  pinInput( TRIGGER_INTERRUPT_PIN );                 // if interrupt PIN is used
#endif
}


void i2cOpen ( void * dptr, uint32_t i2cClockSpeedInHz )
{
  (void)dptr; // not used here
  gpi2c_init(GPIO_I2C_SDA_TMF, GPIO_I2C_SCL_TMF, i2cClockSpeedInHz);
}

void i2cClose ( void * dptr )
{
  (void)dptr; // not used here
  // Wire.end( );
}


void printChar ( char c ) 
{
  // Serial.print( c );
  // printf("%c", c);
}

void printInt ( int32_t i )
{
  // Serial.print( i, DEC );
  // printf("%i", (int)i);
}

void printUint ( uint32_t i )
{
  // Serial.print( i, DEC );
  // printf("%i", (int)i);
}

void printUintHex ( uint32_t i )
{
  // Serial.print( i, HEX );
  // printf("%X", (unsigned int)i);
}

void printStr ( char * str )
{
  // Serial.print( str );    // use only for printing zero-terminated strings: (const char *)
  // printf("%s", str);
}

void printLn ( void )
{
  // Serial.print( '\n' );
  // printf("\n");
}


// function prints a single result, and returns incremented pointer
static uint8_t * print_result ( tmf8828Driver * driver, uint8_t * data )
{
  uint8_t confidence = data[0];               // 1st byte is confidence
  uint16_t distance = data[2];                // 3rd byte is MSB distance
  distance = (distance << 8) + data[1];       // 2nd byte is LSB distnace
  distance = tmf8828CorrectDistance( driver, distance );
  PRINT_CHAR( SEPARATOR );
  PRINT_INT( distance );
  PRINT_CHAR( SEPARATOR );
  PRINT_INT( confidence );
  return data+3;                              // for convenience only, return the new pointer
}



tofResult_t tofResult;
// Results printing:
// #Obj,<i2c_slave_address>,<result_number>,<temperature>,<number_valid_results>,<systick>,<distance_0_mm>,<confidence_0>,<distance_1_mm>,<distance_1>, ...
void printResults ( void * dptr, uint8_t * data, uint8_t len )
{
  tmf8828Driver * driver = (tmf8828Driver *)dptr;  
  if ( len >= TMF8828_COM_CONFIG_RESULT__measurement_result_size )
  {
    // int8_t i;
    uint32_t sysTick = tmf8828GetUint32( data + RESULT_REG( SYS_TICK_0 ) );
    tofResult.temperature = data[ RESULT_REG( TEMPERATURE )];
    tofResult.numberOfValidResults = data[ RESULT_REG( NUMBER_VALID_RESULTS )];
    data += RESULT_REG( RES_CONFIDENCE_0 );
    for(int i = 0; i < 9; i++) {
      uint8_t confidence = data[0];               // 1st byte is confidence
      uint16_t distance = data[2];                // 3rd byte is MSB distance
      distance = (distance << 8) + data[1];       // 2nd byte is LSB distnace
      distance = tmf8828CorrectDistance( driver, distance );
      tofResult.confidence[i] = confidence;
      tofResult.distance[i] = distance;
      data += 3;
    }
    // PRINT_STR( "#Obj" );
    // PRINT_CHAR( SEPARATOR );
    // PRINT_INT( driver->i2cSlaveAddress );
    // PRINT_CHAR( SEPARATOR );
    // PRINT_INT( data[ RESULT_REG( RESULT_NUMBER) ] );
    // PRINT_CHAR( SEPARATOR );
    // PRINT_INT( data[ RESULT_REG( TEMPERATURE )] );
    // PRINT_CHAR( SEPARATOR );
    // PRINT_INT( data[ RESULT_REG( NUMBER_VALID_RESULTS )] );
    // PRINT_CHAR( SEPARATOR );
    // PRINT_INT( sysTick );
    // data = data + RESULT_REG( RES_CONFIDENCE_0 );
    // for ( i = 0; i < PRINT_NUMBER_RESULTS ; i++ )
    // {
    //   data = print_result( driver, data );
    // }
    // PRINT_LN( );
  }
  else // result structure too short
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPARATOR );
    PRINT_STR( "result too short" );
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( len );
    PRINT_LN( );
  }
}

// Print histograms:
// #Raw,<i2c_slave_address>,<sub_packet_number>,<data_0>,<data_1>,..,,<data_127>
// #Cal,<i2c_slave_address>,<sub_packet_number>,<data_0>,<data_1>,..,,<data_127>
void printHistogram ( void * dptr, uint8_t * data, uint8_t len )
{
  tmf8828Driver * driver = (tmf8828Driver *)dptr;  
  if ( len >= TMF8828_COM_HISTOGRAM_PACKET_SIZE )
  {
    uint8_t i;
    uint8_t * ptr = &( data[ RESULT_REG( SUBPACKET_PAYLOAD_0 ) ] );
    if ( data[0] & TMF8828_COM_HIST_DUMP__histogram__raw_24_bit_histogram )
    { 
      PRINT_STR( "#Raw" );
    }
    else if ( data[0] & TMF8828_COM_HIST_DUMP__histogram__electrical_calibration_24_bit_histogram )
    {
      PRINT_STR( "#Cal" );
    }
    else 
    {
      PRINT_STR( "#???" );
    }
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( driver->i2cSlaveAddress );
    PRINT_CHAR( SEPARATOR );
    PRINT_INT( data[ RESULT_REG( SUBPACKET_NUMBER ) ] );          // print the sub-packet number indicating the third-of-a-channel/tdc the histogram belongs to  
    
    for ( i = 0; i < TMF8828_NUMBER_OF_BINS_PER_CHANNEL ; i++, ptr++ )
    {
      PRINT_CHAR( SEPARATOR );
      PRINT_INT( *ptr );
    }
    PRINT_LN( );
  }
  // else structure too short
}

void inputOpen ( uint32_t baudrate )
{
  // Serial.end( );                                     // this clears any old pending data 
  // Serial.begin( baudrate );
}

void inputClose ( )
{
  // Serial.end( );
}

int8_t inputGetKey ( char *c )
{
  // *c = 0;
  // if ( Serial.available() )
  // {
  //   *c = Serial.read();
  //   return 1;
  // }
  return 0;
}

void printConstStr ( const char * str )
{
  /* casting back to Arduino specific memory */
  // Serial.print( reinterpret_cast<const __FlashStringHelper *>( str ) );
  // printf("%s", str);
}

void pinOutput ( uint8_t pin )
{
  // pinMode( pin, OUTPUT );      /* define a pin as output */
  gpio_config_t io_conf = {};
  //disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = pin;
  //disable pull-down mode
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  //disable pull-up mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}

void pinInput ( uint8_t pin )
{ 
  // pinMode( pin, INPUT );      /* define a pin as input */
  gpio_config_t io_conf = {};
  //disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_INPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = pin;
  //disable pull-down mode
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  //disable pull-up mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}

void setInterruptHandler( void (* handler)( void ) )
{
  // attachInterrupt( digitalPinToInterrupt( INTERRUPT_PIN ), handler, FALLING );
}

void disableInterruptHandler( uint8_t pin )
{
  // detachInterrupt( digitalPinToInterrupt( pin ) );
}

void disableInterrupts ( void )
{
  // noInterrupts( );
}

void enableInterrupts ( void )
{
  // interrupts( );
}

// int8_t inputGetKey ( )
// {
//   // if ( Serial.available() )
//   // {
//   //   char c = Serial.read();
//   //   return c;
//   // }
//   return 0;
// }


// ----------------------------------------- i2c ---------------------------------------

// static int8_t i2cTxOnly ( uint8_t logLevel, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, const uint8_t * txData )
// {  // split long transfers into max of 32-bytes: 1 byte is register address, up to 31 are payload.
//   int8_t res = I2C_SUCCESS;
//   uint8_t tx;
//   if ( toTx > ARDUINO_MAX_I2C_TRANSFER - 1) 
//   {
//     tx = ARDUINO_MAX_I2C_TRANSFER - 1;
//   }
//   else 
//   {
//     tx = toTx; // less than 31 bytes 
//   }
//   if ( logLevel & TMF8828_LOG_LEVEL_I2C ) 
//   {
//     PRINT_STR( "I2C-TX (0x" );
//     PRINT_UINT_HEX( slaveAddr );
//     PRINT_STR( ")" );
//     PRINT_STR( " tx=" );
//     PRINT_INT( tx+1 );          // +1 for regAddr
//     PRINT_STR( " 0x" );
//     PRINT_UINT_HEX( regAddr );
//     if ( logLevel >= TMF8828_LOG_LEVEL_DEBUG ) 
//     {
//       uint8_t dumpTx = tx;
//       const uint8_t * dump = txData;
//       while ( dumpTx-- )
//       {
//         PRINT_STR( " 0x" );
//         PRINT_UINT_HEX( *dump );
//         dump++;
//       }
//     }
//     PRINT_LN( );
//   }
//   //   Wire.beginTransmission( slaveAddr );
//   //   Wire.write( regAddr );
//   //   if ( tx )
//   //   {
//   //     Wire.write( txData, tx );
//   //   }
//   //   toTx -= tx;
//   //   txData += tx;
//   //   regAddr += tx;
//   //   res = Wire.endTransmission( );
//   // } while ( toTx && res == I2C_SUCCESS );
//   uint8_t data[toTx+1];
//   data[0] = regAddr;
//   for(int i = 0; i < toTx; i++) {
//     data[1+i] = txData[i];
//   }
//   if(gpi2c_writeData(slaveAddr, &data[0], toTx)!= ESP_OK) {
//     return I2C_ERR_OTHER;
//   }
//   return I2C_SUCCESS;
// }

// static int8_t i2cRxOnly ( uint8_t logLevel, uint8_t slaveAddr, uint16_t toRx, uint8_t * rxData )
// {   // split long transfers into max of 32-bytes
//   uint8_t expected = 0;
//   uint8_t rx = 0;
//   int8_t res = I2C_SUCCESS;
//   do 
//   {
//     uint8_t * dump = rxData; // in case we dump on uart, we need the pointer
//     if ( toRx > ARDUINO_MAX_I2C_TRANSFER ) 
//     {
//       expected = ARDUINO_MAX_I2C_TRANSFER;
//     }
//     else 
//     {
//       expected = toRx; // less than 32 bytes 
//     }
//     Wire.requestFrom( slaveAddr, expected );
//     rx = 0;
//     while ( Wire.available() ) 
//     {  // read in all available bytes
//       *rxData = Wire.read();
//       rxData++;
//       toRx--;
//       rx++;
//     }
//     if ( logLevel & TMF8828_LOG_LEVEL_I2C ) 
//     {
//       PRINT_STR( "I2C-RX (0x" );
//       PRINT_UINT_HEX( slaveAddr );
//       PRINT_STR( ")" );
//       PRINT_STR( " toRx=" );
//       PRINT_INT( rx );
//       if ( logLevel >= TMF8828_LOG_LEVEL_DEBUG ) 
//       {
//         uint8_t dumpRx = rx;
//         while ( dumpRx-- )
//         {
//           PRINT_STR( " 0x" );
//           PRINT_UINT_HEX( *dump );
//           dump++;
//         }
//       }
//       PRINT_LN( );
//     }
//   } while ( toRx && expected == rx );
//   if ( toRx || expected != rx )
//   {
//     res = I2C_ERR_TIMEOUT;
//   }
//   return res;
// }

int8_t i2cTxReg ( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toTx, uint8_t * txData )
{  // split long transfers into max of 32-bytes
  // tmf8828Driver * driver = (tmf8828Driver *)dptr;
  // return i2cTxOnly( driver->logLevel, slaveAddr, regAddr, toTx, txData ); 
  if(gpi2c_writeRegister(slaveAddr, regAddr, txData, toTx) != ESP_OK) {
    ESP_LOGE(TAG, "i2cTxReg error");
    return I2C_ERR_OTHER;
  }
  return I2C_SUCCESS;
}

int8_t i2cRxReg ( void * dptr, uint8_t slaveAddr, uint8_t regAddr, uint16_t toRx, uint8_t * rxData )
{   // split long transfers into max of 32-bytes
  // tmf8828Driver * driver = (tmf8828Driver *)dptr;
  // int8_t res = i2cTxOnly( driver->logLevel, slaveAddr, regAddr, 0, 0 ); 
  // if ( res == I2C_SUCCESS )
  // {
  //   res = i2cRxOnly( driver->logLevel, slaveAddr, toRx, rxData );
  // }
  // ESP_LOGI(TAG, "i2cRxReg: addr: %02X - reg: %02X n: %i", slaveAddr, regAddr, toRx);
  if(gpi2c_readRegister(slaveAddr, regAddr, rxData, toRx) != ESP_OK) {
    ESP_LOGE(TAG, "i2cRxReg error");
    return I2C_ERR_OTHER;
  }
  return I2C_SUCCESS;
}

// int8_t i2cTxRx ( void * dptr, uint8_t slaveAddr, uint16_t toTx, const uint8_t * txData, uint16_t toRx, uint8_t * rxData )
// {
//   tmf8828Driver * driver = (tmf8828Driver *)dptr;
//   int8_t res = I2C_SUCCESS;
//   if ( toTx )
//   {
//     res = i2cTxOnly( driver->logLevel, slaveAddr, *txData, toTx-1, txData+1 );
//   }
//   if ( toRx && res == I2C_SUCCESS )
//   {
//     res = i2cRxOnly( driver->logLevel, slaveAddr, toRx, rxData );
//   }
//   return res;
// }
