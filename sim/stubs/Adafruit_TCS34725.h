// Simulator version of the Adafruit TCS34725 colour sensor library.
// The colour returned is the floor colour under the sensor in the simulated maze.
#pragma once
#include <Arduino.h>
#include <Wire.h>

#define TCS34725_ADDRESS (0x29)
#define TCS34725_INTEGRATIONTIME_2_4MS (0xFF)
#define TCS34725_INTEGRATIONTIME_24MS (0xF6)
#define TCS34725_INTEGRATIONTIME_50MS (0xEB)
#define TCS34725_INTEGRATIONTIME_60MS (0xE7)
#define TCS34725_INTEGRATIONTIME_101MS (0xD6)
#define TCS34725_INTEGRATIONTIME_120MS (0xCE)
#define TCS34725_INTEGRATIONTIME_154MS (0xC0)
#define TCS34725_INTEGRATIONTIME_180MS (0xB5)
#define TCS34725_INTEGRATIONTIME_199MS (0xAD)
#define TCS34725_INTEGRATIONTIME_240MS (0x9C)
#define TCS34725_INTEGRATIONTIME_300MS (0x83)
#define TCS34725_INTEGRATIONTIME_428MS (0x52)
#define TCS34725_INTEGRATIONTIME_600MS (0x06)
#define TCS34725_INTEGRATIONTIME_614MS (0x00)

typedef enum {
  TCS34725_GAIN_1X = 0x00,
  TCS34725_GAIN_4X = 0x01,
  TCS34725_GAIN_16X = 0x02,
  TCS34725_GAIN_60X = 0x03
} tcs34725Gain_t;

class Adafruit_TCS34725 {
 public:
  Adafruit_TCS34725(uint8_t it = TCS34725_INTEGRATIONTIME_2_4MS, tcs34725Gain_t gain = TCS34725_GAIN_1X);
  bool begin(uint8_t addr = TCS34725_ADDRESS, TwoWire* theWire = &Wire);
  bool init() { return begin(); }
  void setIntegrationTime(uint8_t it) { it_ = it; }
  void setGain(tcs34725Gain_t gain) { gain_ = gain; }
  void getRawData(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c);
  void getRawDataOneShot(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) { getRawData(r, g, b, c); }
  void getRGB(float* r, float* g, float* b);
  uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b);
  uint16_t calculateColorTemperature_dn40(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
  uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b);
  void setInterrupt(bool flag);
  void clearInterrupt();
  void setIntLimits(uint16_t l, uint16_t h) { (void)l; (void)h; }
  void enable();
  void disable();

 private:
  uint8_t it_;
  tcs34725Gain_t gain_;
  bool initialised_ = false;
};
