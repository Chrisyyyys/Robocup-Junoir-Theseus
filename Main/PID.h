
#ifndef PID_h
#define PID_h
class PID{
  public:
    double kp, ki, kd; // define public variables kp, ki, kd
    double error, prevError, delta, cumError; // define error, previous error , deltaerror and cumulative error
    double currentTime, previousTime; // timer
    PID(double , double , double ); // ?
    getPID(double);
  

};
#endif
