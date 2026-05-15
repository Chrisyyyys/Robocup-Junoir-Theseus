
#ifndef PID_h
#define PID_h
class PID{
  public:
    double kp, ki, kd; // define public variables kp, ki, kd
    double error, prevError, delta, cumError; // define error, previous error , deltaerror and cumulative error
    double currentTime, previousTime; // timer
    double start = 0; double end = 0; // pausing timestamp
    PID(double , double , double ); // PID inputs
    double getPID(double);
    void pausePID(int);
  

};
#endif
