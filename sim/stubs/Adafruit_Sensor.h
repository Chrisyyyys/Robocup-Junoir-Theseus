// Simulator version of Adafruit_Sensor.h (unified sensor event types).
#pragma once
#include <stdint.h>

typedef struct {
  union {
    float v[3];
    struct {
      float x;
      float y;
      float z;
    };
    struct {
      float roll;
      float pitch;
      float heading;
    };
  };
  int8_t status;
  uint8_t reserved[3];
} sensors_vec_t;

typedef struct {
  int32_t version;
  int32_t sensor_id;
  int32_t type;
  int32_t reserved0;
  int32_t timestamp;
  union {
    float data[4];
    sensors_vec_t acceleration;
    sensors_vec_t magnetic;
    sensors_vec_t orientation;
    sensors_vec_t gyro;
    float temperature;
    float distance;
    float light;
    float pressure;
    float relative_humidity;
    float current;
    float voltage;
  };
} sensors_event_t;

typedef struct {
  char name[12];
  int32_t version;
  int32_t sensor_id;
  int32_t type;
  float max_value;
  float min_value;
  float resolution;
  int32_t min_delay;
} sensor_t;

class Adafruit_Sensor {
 public:
  virtual ~Adafruit_Sensor() {}
  virtual void enableAutoRange(bool enabled) { (void)enabled; }
  virtual bool getEvent(sensors_event_t*) = 0;
  virtual void getSensor(sensor_t*) = 0;
};
