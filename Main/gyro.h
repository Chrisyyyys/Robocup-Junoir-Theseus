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
    double get_filtered_acceleration();       // EMA filtered
    void reset_accel_filter();
    bool detect_forward_motion_sample();
    private:
      bool accelFilterInitialized = false;
      double accelFiltered = 0.0;

};
#endif
