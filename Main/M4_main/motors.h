#ifndef motors_h
#define motors_h
class motors {
  public:
    motors(int, int, int, int);
    void init_drive();
    void encoder_update_A();
    void encoder_update_B();
    void fullstop();
    void drive(int, int, int, int);
    void reset_encoderCount(bool, bool);
    void set_encoderCountA(int);
    void set_encoderCountB(int);
    void set_interrupt(bool, bool); 
    void fw(int);
    void backward(int);
    void turnleft(int);
    void turnright(int);
    volatile int encoderCountA;
    volatile int encoderCountB;
  private:
    int _encoderPin_A_A;
    int _encoderPin_A_B;
    int _encoderPin_B_A;
    int _encoderPin_B_B;
};
#endif