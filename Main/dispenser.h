#ifndef dispenser_h
#define dispenser_h
class dispenser{
  public:
    int leftDispensed; int rightDispensed;
    int incr, offset, steps_per_rev;
    dispenser(int, int, int);
    void rotate(int);
    void dispenseLeft(char);
    void dispenseRight(char);
};
#endif 