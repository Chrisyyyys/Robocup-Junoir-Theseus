// #include <Wire.h>
// #include <SparkFun_I2C_Mux_Arduino_Library.h>
// #include <VL53L0X.h>

// QWIICMUX mux;
// VL53L0X sensor;

// int dist0, dist1, dist2, dist3, dist4, dist5, dist6;  // 7 variables

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();

//   if (!mux.begin()) {
//     Serial.println("MUX not detected!");
//     while (1);
//   }
//   Serial.println("MUX detected.");

//   // Initialize sensors on ports 0–6
//   for (int p = 0; p < 7; p++) {
//     mux.setPort(p);
//     delay(10);

//     if (!sensor.init()) {
//       Serial.print("Sensor NOT found on port ");
//       Serial.println(p);
//     } else {
//       Serial.print("Sensor found on port ");
//       Serial.println(p);

//       sensor.setTimeout(500);
//       sensor.stopContinuous();
//     }
//   }

//   Serial.println("Setup complete.");
// }

// void loop() {

//   for (int p = 0; p < 7; p++) {
//     mux.setPort(p);
//     delay(5);

//     int d = sensor.readRangeSingleMillimeters();

//     if (sensor.timeoutOccurred()) d = -1;

//     // Store into the correct variable
//     switch (p) {
//       case 0: dist0 = d; break;
//       case 1: dist1 = d; break;
//       case 2: dist2 = d; break;
//       case 3: dist3 = d; break;
//       case 4: dist4 = d; break;
//       case 5: dist5 = d; break;
//       case 6: dist6 = d; break;
//     }
//   }

//   // Print values
//   Serial.print("0: "); Serial.print(dist0);
//   Serial.print(" | 1: "); Serial.print(dist1);
//   Serial.print(" | 2: "); Serial.print(dist2);
//   Serial.print(" | 3: "); Serial.print(dist3);
//   Serial.print(" | 4: "); Serial.print(dist4);
//   Serial.print(" | 5: "); Serial.print(dist5);
//   Serial.print(" | 6: "); Serial.println(dist6);

//   Serial.println("------------------------------------");

//   delay(150);
// }

