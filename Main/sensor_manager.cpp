#include "sensor_manager.h"
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>

QWIICMUX mux;
VL53L0X sensor;

int dist[7];  // This actually stores the distances

void readAllSensors() {
    for (int p = 0; p < 7; p++) {
        mux.setPort(p);
        delay(2);

        int d = sensor.readRangeSingleMillimeters();
        if (sensor.timeoutOccurred()) d = -1;

        dist[p] = d;  // Store in array
    }
}
