#ifndef gyro_h
#define gyro_h
#include <math.h>

// Wrap an angle difference into [-180, 180).
inline double wrap180(double deg){
  deg = fmod(deg + 180.0, 360.0);
  if (deg < 0) deg += 360.0;
  return deg - 180.0;
}

// timer class
class gyro{
  public:

    gyro();
    void init_Gyro();
    double heading();            // maze-frame heading in [0, 360): 0 = maze NORTH, clockwise positive
    double rawHeading();         // BNO055 heading in [0, 360), before the maze offset
    void setMapHeading(double);  // re-zero so the robot's current heading reads as this maze angle
    double pitch_heading();
    int inverse(int,bool);
    int modulus(int);
    int headingToCardinal(double);
    void reset_accel_filter();
    double opposite_heading(double);
    private:
      double headingOffset = 0.0; // raw heading that corresponds to maze NORTH (see setMapHeading)
      bool accelFilterInitialized = false;
      double accelFiltered = 0.0;
      double v;
      unsigned long lastTime;

};
#endif