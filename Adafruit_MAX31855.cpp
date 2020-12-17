/***************************************************
  This is a library for the Adafruit Thermocouple Sensor w/MAX31855K

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/products/269

  These displays use SPI to communicate, 3 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MAX31855.h"

#ifndef __AVR
  #include "../../../../Marlin/src/HAL/shared/HAL_SPI.h"
#endif

#include "../../../../Marlin/src/HAL/shared/Delay.h"

#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include <stdlib.h>
#include <SPI.h>

#ifdef __AVR
  static SPISettings max31855_spisettings =
      SPISettings(4000000, MSBFIRST, SPI_MODE0);
#else
  static SPISettings max31855_spisettings =
      SPISettings(SPI_QUARTER_SPEED, MSBFIRST, SPI_MODE0);
#endif

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI for
    PIN values which are larger than 127. If you have PIN values less than
    or equal to 127 use the other call for SW SPI.
    @param spi_cs the SPI CS pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_clk the SPI clock pin to use
    @param pin_mappping set to 1 for positive pin values
*/
/**************************************************************************/
Adafruit_MAX31865::Adafruit_MAX31855(uint32_t spi_cs, uint32_t spi_miso,
                                     uint32_t spi_clk, uint8_t pin_mapping) {
  __cs = spi_cs;
  __miso = spi_miso;
  __sclk = spi_clk;
  __pin_mapping = pin_mapping;

  if (__pin_mapping == 0) {
    _cs = __cs;
    _miso = __miso;
    _sclk = __sclk;
  }

  initialized = false;
}

/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI for PIN value
    which are larger than 127. If you have PIN values less than
    or equal to 127 use the other call for HW SPI
    @param spi_cs the SPI CS pin to use along with the default SPI device
    @param pin_mapping set to 1 for positive pin values
*/
/**************************************************************************/
Adafruit_MAX31865::Adafruit_MAX31855(uint32_t spi_cs, uint8_t pin_mapping) {
  __cs = spi_cs;
  __sclk = __miso = -1UL;  //-1UL or 0xFFFFFFFF
  __pin_mapping = pin_mapping;

  if (__pin_mapping == 0) {
    _cs = __cs;
    _miso = __miso;
    _sclk = __sclk;
  }

  initialized = false;
}

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI for PIN
    values less than or equal to 127.
    @param spi_cs the SPI CS pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_clk the SPI clock pin to use
*/
/**************************************************************************/
//
Adafruit_MAX31855::Adafruit_MAX31855(int8_t spi_cs, int8_t spi_miso, int8_t spi_sclk) {
  _cs = spi_cs;
  _miso = spi_miso;
  _sclk = spi_sclk;

  initialized = false;
}

/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI for PIN
    values less than or equal to 127.
    @param spi_cs the SPI CS pin to use along with the default SPI device
*/
/**************************************************************************/
//
Adafruit_MAX31855::Adafruit_MAX31855(int8_t spi_cs) {
  _cs = spi_cs;
  _sclk = _miso = -1;

  initialized = false;
}

/**************************************************************************/
/*!
    @brief Initialize the SPI interface
*/
/**************************************************************************/
void Adafruit_MAX31855::begin(void) {
  //define pin modes
  if (!__pin_mapping) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
  }
  else {
    pinMode(__cs, OUTPUT);
    digitalWrite(__cs, HIGH);
  }

  if (_sclk == -1 || __sclk == -1UL) {
    // hardware SPI
    //start and configure hardware SPI
    SPI.begin();
  }
  else {
    if (!__pin_mapping) {
      pinMode(_sclk, OUTPUT);
      pinMode(_miso, INPUT);
    }
    else {
      pinMode(__sclk, OUTPUT);
      pinMode(__miso, INPUT);
    }
  }
  initialized = true;
}

/**************************************************************************/
/*!
    @brief Read the reference junction temperature in C
    @returns Reference junction Temperature in C
*/
/**************************************************************************/
double Adafruit_MAX31855::readInternal(void) {
  uint32_t v;

  v = spiread32();

  // ignore bottom 4 bits - they're just thermocouple data
  v >>= 4;

  // pull the bottom 11 bits off
  float internal = v & 0x7FF;
  // check sign bit!
  if (v & 0x800) {
    // Convert to negative value by extending sign and casting to signed type.
    int16_t tmp = 0xF800 | (v & 0x7FF);
    internal = tmp;
  }
  internal *= 0.0625; // LSB = 0.0625 degrees
  //Serial.print("\tInternal Temp: "); Serial.println(internal);
  return internal;
}

/**************************************************************************/
/*!
    @brief Read the cold-junction compensated thermocouple temperature
    @returns cold-junction compensated thermocouple Temperature in C
*/
/**************************************************************************/
double Adafruit_MAX31855::readCelsius(void) {

  int32_t v;

  v = spiread32();

  //Serial.print("0x"); Serial.println(v, HEX);

  /*
  float internal = (v >> 4) & 0x7FF;
  internal *= 0.0625;
  if ((v >> 4) & 0x800)
    internal *= -1;
  Serial.print("\tInternal Temp: "); Serial.println(internal);
  */

  if (v & 0x7) {
    // uh oh, a serious problem!
    return NAN;
  }

  if (v & 0x80000000) {
    // Negative value, drop the lower 18 bits and explicitly extend sign bits.
    v = 0xFFFFC000 | ((v >> 18) & 0x00003FFF);
  }
  else {
    // Positive value, just drop the lower 18 bits.
    v >>= 18;
  }
  //Serial.println(v, HEX);

  double centigrade = v;

  // LSB = 0.25 degrees C
  centigrade *= 0.25;

  return centigrade;
}

/**************************************************************************/
/*!
    @brief Read the error bits of the data packet
    @returns the the 3 error bits from the data packet
*/
/**************************************************************************/
uint8_t Adafruit_MAX31855::readError(void) {
  return spiread32() & 0x7;
}

/**************************************************************************/
/*!
    @brief Read cold-junction compensated thermocouple temperature
    @returns cold-junction compensated thermocouple Temperature in F
*/
/**************************************************************************/
double Adafruit_MAX31855::readFarenheit(void) {
  float f = readCelsius();
  f *= 9.0;
  f /= 5.0;
  f += 32;
  return f;
}

/**************************************************************************/
/*!
    @brief Read the 32-bit raw data packet
    @returns the 32-bit raw data packet for the user to process
*/
/**************************************************************************/
uint32_t Adafruit_MAX31855::readRaw32(void) {
  return spiread32();
}

/**************************************************************************/

uint32_t Adafruit_MAX31855::spiread32(void) {
  int i;
  uint32_t d = 0;

  // backcompatibility!
  if (! initialized) {
    begin();
  }

  if (!__pin_mapping)
    digitalWrite(_cs, LOW);
  else
    digitalWrite(__cs, LOW);
  DELAY_US(1000);

  if(_sclk == -1 || __sclk == -1UL) {
    // hardware SPI

    SPI.beginTransaction(max31855_spisettings);

    d = SPI.transfer(0);
    d <<= 8;
    d |= SPI.transfer(0);
    d <<= 8;
    d |= SPI.transfer(0);
    d <<= 8;
    d |= SPI.transfer(0);

    SPI.endTransaction();
  }
  else {
    // software SPI

    if (!__pin_mapping)
      digitalWrite(_sclk, LOW);
    else
      digitalWrite(__sclk, LOW);
    DELAY_US(1000);

    for (i=31; i>=0; i--) {

      if (!__pin_mapping)
        digitalWrite(_sclk, LOW);
      else
        digitalWrite(__sclk, LOW);
      DELAY_US(1000);

      d <<= 1;

      if (!__pin_mapping) {
        if (digitalRead(_miso)) {
	        d |= 1;
        }
      }
      else {
        if (digitalRead(__miso)) {
	        d |= 1;
        }
      }

     if (!__pin_mapping)
        digitalWrite(_sclk, HIGH);
     else
        digitalWrite(__sclk, HIGH);
      DELAY_US(1000);

    }
  }

  if (!__pin_mapping)
    digitalWrite(_cs, HIGH);
  else
    digitalWrite(__cs, HIGH);

  //Serial.println(d, HEX);
  return d;
}
