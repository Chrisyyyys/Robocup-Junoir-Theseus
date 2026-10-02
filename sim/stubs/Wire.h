// Simulator version of Wire.h: a virtual I2C bus (100 kHz timing) with the
// robot's devices behind a TCA9548A multiplexer.
#pragma once
#include <Arduino.h>

class TwoWire {
 public:
  void begin();
  void begin(uint8_t address);
  void end() {}
  void setClock(uint32_t hz);
  void beginTransmission(uint8_t address);
  void beginTransmission(int address) { beginTransmission((uint8_t)address); }
  uint8_t endTransmission(bool sendStop = true);
  uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop = true);
  uint8_t requestFrom(int address, int quantity) { return requestFrom((uint8_t)address, (uint8_t)quantity); }
  size_t write(uint8_t data);
  size_t write(const uint8_t* data, size_t quantity);
  int available();
  int read();
  int peek();

 private:
  uint8_t tx_addr_ = 0;
  uint8_t tx_buf_[32] = {0};
  int tx_len_ = 0;
  uint8_t rx_buf_[32] = {0};
  int rx_len_ = 0;
  int rx_pos_ = 0;
};

extern TwoWire Wire;
extern TwoWire Wire1;
