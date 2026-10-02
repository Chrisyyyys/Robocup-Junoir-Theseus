// Simulator version of the BNO055 helper maths (only imu::Vector is needed).
#pragma once
#include <math.h>
#include <stdint.h>

namespace imu {
template <uint8_t N>
class Vector {
 public:
  Vector() { for (int i = 0; i < N; i++) p_[i] = 0; }
  Vector(double a, double b, double c) { p_[0] = a; if (N > 1) p_[1] = b; if (N > 2) p_[2] = c; }
  double& operator[](int i) { return p_[i]; }
  double operator[](int i) const { return p_[i]; }
  double& x() { return p_[0]; }
  double& y() { return p_[1]; }
  double& z() { return p_[2]; }
  double magnitude() const { double s = 0; for (int i = 0; i < N; i++) s += p_[i] * p_[i]; return sqrt(s); }

 private:
  double p_[N];
};
}  // namespace imu
