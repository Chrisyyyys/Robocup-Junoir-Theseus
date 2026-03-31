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
    double update_forward_velocity();
    double get_forward_velocity();
    void reset_velocity_estimator();
    private:
      bool accelFilterInitialized = false;
      double accelFiltered = 0.0;
      double forwardVelocity = 0.0;
      unsigned long lastVelocityUpdateMs = 0;
      bool velocityEstimatorInitialized = false;

};
#endif
