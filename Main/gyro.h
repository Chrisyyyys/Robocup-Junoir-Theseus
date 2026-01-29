#ifndef gyro_h
#define gyro_h
// timer class
class gyro{
  public:
    double headingOffset;
    gyro();
    void init_Gyro();
    double heading();
    void resetHeading();

};
#endif