// Simulator version of the Pololu VL53L0X library (time-of-flight distance sensor).
// Behaves like the real library: readRangeContinuousMillimeters() waits until the
// sensor has a new measurement (about every 33 ms) and returns 65535 on timeout.
#pragma once
#include <Arduino.h>
#include <Wire.h>

class VL53L0X {
 public:
  enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };
  uint8_t last_status = 0;

  VL53L0X();
  void setBus(TwoWire* bus) { (void)bus; }
  void setAddress(uint8_t new_addr);
  uint8_t getAddress() { return address_; }
  bool init(bool io_2v8 = true);
  bool setSignalRateLimit(float limit_Mcps) { (void)limit_Mcps; return true; }
  float getSignalRateLimit() { return 0.25f; }
  bool setMeasurementTimingBudget(uint32_t budget_us);
  uint32_t getMeasurementTimingBudget() { return budget_us_; }
  bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks) { (void)type; (void)period_pclks; return true; }
  void startContinuous(uint32_t period_ms = 0);
  void stopContinuous();
  uint16_t readRangeContinuousMillimeters();
  uint16_t readRangeSingleMillimeters();
  void setTimeout(uint16_t timeout) { io_timeout_ = timeout; }
  uint16_t getTimeout() { return io_timeout_; }
  bool timeoutOccurred() { bool t = did_timeout_; did_timeout_ = false; return t; }

 private:
  uint8_t address_ = 0x29;
  uint16_t io_timeout_ = 0;
  bool did_timeout_ = false;
  uint32_t budget_us_ = 33000;
};
