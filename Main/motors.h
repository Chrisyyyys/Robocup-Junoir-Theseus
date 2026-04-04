#ifndef motors_h
#define motors_h
class motors {
  public:
    motors(int, int);
    encoder_update_A();
    encoder_update_B();
    fullstop();
    fw(int);
    backward(int);
    turnleft(int);
    turnright(int);
    int encoderCountA;
    int encoderCountB;
  private:
    int _encoderPin_A_B;
    int _encoderPin_B_A;

}
#endif