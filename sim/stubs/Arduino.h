// Simulator version of Arduino.h (Arduino GIGA R1 / mbed core flavour).
// Only what the robot code needs. Every call that would take time on the real
// board (I2C, delays, serial, ...) advances the simulator's virtual clock.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <algorithm>

using std::abs;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3

#define CHANGE 2
#define FALLING 3
#define RISING 4

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bit(b) (1UL << (b))
#define F(x) (x)

typedef uint8_t byte;
typedef bool boolean;
typedef uint16_t word;

template <class T, class L>
auto min(const T& a, const L& b) -> decltype((b < a) ? b : a) { return (b < a) ? b : a; }
template <class T, class L>
auto max(const T& a, const L& b) -> decltype((b < a) ? b : a) { return (a < b) ? b : a; }

// ---- time ----
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void yield();

// ---- pins ----
void pinMode(int pin, int mode);
void digitalWrite(int pin, int val);
int digitalRead(int pin);
int analogRead(int pin);
void analogWrite(int pin, int val);
int digitalPinToInterrupt(int pin);
void attachInterrupt(int interruptNum, void (*isr)(), int mode);
void detachInterrupt(int interruptNum);
void noInterrupts();
void interrupts();

// ---- misc ----
long random(long howbig);
long random(long howsmall, long howbig);
void randomSeed(unsigned long seed);
long map(long x, long in_min, long in_max, long out_min, long out_max);

// ---- String ----
class String {
 public:
  String() {}
  String(const char* s) : s_(s ? s : "") {}
  String(const std::string& s) : s_(s) {}
  String(char c) : s_(1, c) {}
  String(unsigned char v, unsigned char base = DEC);
  String(int v, unsigned char base = DEC);
  String(unsigned int v, unsigned char base = DEC);
  String(long v, unsigned char base = DEC);
  String(unsigned long v, unsigned char base = DEC);
  String(float v, unsigned char decimals = 2);
  String(double v, unsigned char decimals = 2);
  const char* c_str() const { return s_.c_str(); }
  unsigned int length() const { return (unsigned int)s_.size(); }
  String& operator+=(const String& o) { s_ += o.s_; return *this; }
  String& operator+=(const char* o) { s_ += o; return *this; }
  String& operator+=(char c) { s_ += c; return *this; }
  bool operator==(const String& o) const { return s_ == o.s_; }
  bool operator==(const char* o) const { return s_ == o; }
  char operator[](unsigned int i) const { return i < s_.size() ? s_[i] : 0; }
  int toInt() const { return atoi(s_.c_str()); }
  float toFloat() const { return (float)atof(s_.c_str()); }
  const std::string& str() const { return s_; }

 private:
  std::string s_;
};
inline String operator+(const String& a, const String& b) { String r(a); r += b; return r; }
inline String operator+(const String& a, const char* b) { String r(a); r += b; return r; }
inline String operator+(const char* a, const String& b) { String r(a); r += b; return r; }
inline String operator+(const String& a, char b) { String r(a); r += b; return r; }
inline String operator+(const String& a, int b) { return a + String(b); }
inline String operator+(const String& a, long b) { return a + String(b); }
inline String operator+(const String& a, unsigned int b) { return a + String(b); }
inline String operator+(const String& a, unsigned long b) { return a + String(b); }
inline String operator+(const String& a, double b) { return a + String(b); }

// ---- Print / Serial ----
class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* buf, size_t n) {
    size_t k = 0;
    while (n--) k += write(*buf++);
    return k;
  }
  size_t write(const char* s) { return s ? write((const uint8_t*)s, strlen(s)) : 0; }

  size_t print(const char* s) { return write(s); }
  size_t print(const String& s) { return write(s.c_str()); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned char v, int base = DEC) { return printNumber((unsigned long)v, base); }
  size_t print(int v, int base = DEC) { return printSigned((long)v, base); }
  size_t print(unsigned int v, int base = DEC) { return printNumber((unsigned long)v, base); }
  size_t print(long v, int base = DEC) { return printSigned(v, base); }
  size_t print(unsigned long v, int base = DEC) { return printNumber(v, base); }
  size_t print(long long v, int base = DEC) { return printSigned((long)v, base); }
  size_t print(unsigned long long v, int base = DEC) { return printNumber((unsigned long)v, base); }
  size_t print(double v, int digits = 2) { return printFloat(v, digits); }

  size_t println() { return write("\r\n"); }
  template <class T>
  size_t println(const T& v) { size_t n = print(v); return n + println(); }
  template <class T>
  size_t println(const T& v, int fmt) { size_t n = print(v, fmt); return n + println(); }

 private:
  size_t printSigned(long v, int base);
  size_t printNumber(unsigned long v, int base);
  size_t printFloat(double v, int digits);
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
};

// Serial ports. Serial = USB log, Serial3/Serial4 = camera UARTs on the robot.
class SimSerial : public Stream {
 public:
  explicit SimSerial(int index) : index_(index) {}
  void begin(unsigned long baud) { baud_ = baud; }
  void begin(unsigned long baud, int) { baud_ = baud; }
  void end() {}
  int available() override;
  int read() override;
  int peek() override;
  void flush() {}
  size_t write(uint8_t c) override;
  using Print::write;
  explicit operator bool() const { return true; }

 private:
  int index_;
  unsigned long baud_ = 0;
};

extern SimSerial Serial;
extern SimSerial Serial1;
extern SimSerial Serial2;
extern SimSerial Serial3;
extern SimSerial Serial4;
