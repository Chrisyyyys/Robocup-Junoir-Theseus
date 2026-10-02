// Simulator version of the SparkFun Qwiic Mux (TCA9548A) library.
#pragma once
#include <Arduino.h>
#include <Wire.h>

class QWIICMUX {
 public:
  bool begin(uint8_t deviceAddress = 0x70, TwoWire& wirePort = Wire);
  bool isConnected();
  bool setPort(uint8_t portNumber);
  bool setPortState(uint8_t portBits);
  uint8_t getPort();
  uint8_t getPortState();
  bool enablePort(uint8_t portNumber);
  bool disablePort(uint8_t portNumber);

 private:
  uint8_t addr_ = 0x70;
};
