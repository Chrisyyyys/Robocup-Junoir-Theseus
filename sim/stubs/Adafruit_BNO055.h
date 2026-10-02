// Simulator version of the Adafruit BNO055 library. orientation.x = heading
// (0..360, clockwise), orientation.z = pitch, like the real library.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)

class Adafruit_BNO055 : public Adafruit_Sensor {
 public:
  typedef enum {
    OPERATION_MODE_CONFIG = 0x00,
    OPERATION_MODE_ACCONLY = 0x01,
    OPERATION_MODE_MAGONLY = 0x02,
    OPERATION_MODE_GYRONLY = 0x03,
    OPERATION_MODE_ACCMAG = 0x04,
    OPERATION_MODE_ACCGYRO = 0x05,
    OPERATION_MODE_MAGGYRO = 0x06,
    OPERATION_MODE_AMG = 0x07,
    OPERATION_MODE_IMUPLUS = 0x08,
    OPERATION_MODE_COMPASS = 0x09,
    OPERATION_MODE_M4G = 0x0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
    OPERATION_MODE_NDOF = 0x0C
  } adafruit_bno055_opmode_t;
  typedef enum {
    VECTOR_ACCELEROMETER = 0x08,
    VECTOR_MAGNETOMETER = 0x0E,
    VECTOR_GYROSCOPE = 0x14,
    VECTOR_EULER = 0x1A,
    VECTOR_LINEARACCEL = 0x28,
    VECTOR_GRAVITY = 0x2E
  } adafruit_vector_type_t;

  Adafruit_BNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A, TwoWire* theWire = &Wire);
  bool begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF);
  void setMode(adafruit_bno055_opmode_t mode) { mode_ = mode; }
  adafruit_bno055_opmode_t getMode() { return mode_; }
  void setExtCrystalUse(bool usextal) { (void)usextal; }
  bool getEvent(sensors_event_t* event) override;
  bool getEvent(sensors_event_t* event, adafruit_vector_type_t type);
  void getSensor(sensor_t* sensor) override { (void)sensor; }
  imu::Vector<3> getVector(adafruit_vector_type_t type);
  void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);
  bool isFullyCalibrated() { return true; }
  int8_t getTemp() { return 25; }

 private:
  int32_t id_;
  uint8_t addr_;
  adafruit_bno055_opmode_t mode_ = OPERATION_MODE_NDOF;
};
