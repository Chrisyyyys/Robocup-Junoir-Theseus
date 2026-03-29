#ifndef gyro_h
#define gyro_h
// timer class
class gyro{
  public:
    
    gyro();
    void init_Gyro();
    double heading();
    double yaw_heading();
    int inverse(int,bool);
    int modulus(int);
    int headingToCardinal(double);
    double get_acceleration();

};
#endif