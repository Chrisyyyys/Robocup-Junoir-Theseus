#ifndef timer_h
#define timer_h
// timer class
class timer{
  public:
    double startTime; double currentTime;
    double startTimeStamp = 0; double endTimeStamp = 0;
    timer();
    double getTime();
    double delta_time();
    double reset_delta_time();
    void pause(int);
  private:
    double last;
    double current;
    double delta;
};
#endif