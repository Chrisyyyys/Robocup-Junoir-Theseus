#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <VL53L0X.h>

QWIICMUX mux;
VL53L0X sensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mux.begin()) {
    Serial.println("MUX not detected!");
    while (1);
  }
  Serial.println("MUX detected.");

  // initialize all needed ports
  for (int p = 0; p < 8; p++) {
    mux.setPort(p);
    delay(10);

    if (!sensor.init()) {
      Serial.print("Sensor NOT found on port ");
      Serial.println(p);
    } else {
      Serial.print("Sensor found on port ");
      Serial.println(p);

      sensor.setTimeout(500);
      sensor.stopContinuous();   // VERY IMPORTANT
    }
  }

  Serial.println("Setup complete.");
}

void loop() {
  for (int p = 0; p < 8; p++) {
    mux.setPort(p);
    delay(5);

    int dist = sensor.readRangeSingleMillimeters();

    Serial.print("Port ");
    Serial.print(p);
    Serial.print(": ");

    if (sensor.timeoutOccurred()) {
      Serial.println("TIMEOUT");
    } else {
      Serial.print(dist);
      Serial.println(" mm");
    }
  }

  Serial.println("--------------------------");
  delay(200);
}
