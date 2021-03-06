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

#ifndef ADAFRUIT_MAX31855_H
#define ADAFRUIT_MAX31855_H

#if (ARDUINO >= 100 || ARDUINOLPC)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

class Adafruit_MAX31855 {
 public:
  Adafruit_MAX31855(uint32_t spi_cs, uint32_t spi_miso, uint32_t spi_sclk, uint8_t pin_mapping);
  Adafruit_MAX31855(uint32_t spi_cs, uint8_t pin_mapping);

  Adafruit_MAX31855(int8_t spi_cs, int8_t spi_miso, int8_t spi_sclk);
  Adafruit_MAX31855(int8_t spi_cs);

  void begin(void);
  double readInternal(void);
  double readCelsius(void);
  double readFarenheit(void);
  uint8_t readError(void);
  uint32_t readRaw32(void);

 private:
  bool initialized;

  int8_t _sclk, _miso, _cs;
  uint32_t __sclk, __miso, __cs;
  uint8_t __pin_mapping = 0x00;
  bool first_reading = true;

  uint32_t spiread32(void);
};

#endif
