// Implementations of the Arduino API and the libraries the robot uses.
// Each call costs the virtual time it costs on the real robot (I2C at 100 kHz,
// delays inside the libraries, sensor measurement times, ...).
#include <Arduino.h>
#include <Wire.h>
#include <rtos.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MotorShield.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Stepper.h>
#include <LiquidCrystal.h>

#include <deque>

#include "sim.h"

using namespace sim;

namespace sim {
bool g_in_isr = false;
std::string g_lcd_line[4];
}  // namespace sim

static int64_t i2c_us(int bytes) { return (int64_t)bytes * 100; }  // 100 kHz bus, ~10 bit times per byte

// ------------------------------------------------------------------ time
unsigned long micros() {
  if (!sched_started()) return 0;
  sched_busy(1);
  return (unsigned long)sched_now();
}
unsigned long millis() {
  if (!sched_started()) return 0;
  sched_busy(1);
  return (unsigned long)(sched_now() / 1000);
}
void delay(unsigned long ms) { sched_sleep((int64_t)ms * 1000); }
void delayMicroseconds(unsigned int us) { sched_busy(us); }
void yield() { sched_sleep(0); }

// ------------------------------------------------------------------ pins
static int g_level[256];
static int g_mode[256];
static void (*g_isr[256])() = {nullptr};

namespace sim {
int pin_level(int pin) { return (pin >= 0 && pin < 256) ? g_level[pin] : 0; }
void pin_set_input(int pin, int level) {
  if (pin >= 0 && pin < 256) g_level[pin] = level ? 1 : 0;
}
void isr_fire(int pin) {
  if (pin < 0 || pin >= 256 || !g_isr[pin]) return;
  g_in_isr = true;
  g_isr[pin]();
  g_in_isr = false;
}
bool isr_attached(int pin) { return pin >= 0 && pin < 256 && g_isr[pin]; }
}  // namespace sim

void pinMode(int pin, int mode) {
  if (pin >= 0 && pin < 256) g_mode[pin] = mode;
}
void digitalWrite(int pin, int val) {
  sched_busy(1);
  if (pin < 0 || pin >= 256) return;
  int v = val ? 1 : 0;
  if (g_level[pin] != v) {
    g_level[pin] = v;
    rec_led(pin, v);
  }
}
int digitalRead(int pin) {
  sched_busy(1);
  return pin_level(pin);
}
int analogRead(int pin) {
  sched_busy(20);
  (void)pin;
  return 0;
}
void analogWrite(int pin, int val) { digitalWrite(pin, val > 127); }
int digitalPinToInterrupt(int pin) { return pin; }
void attachInterrupt(int n, void (*isr)(), int mode) {
  (void)mode;
  if (n >= 0 && n < 256) g_isr[n] = isr;
}
void detachInterrupt(int n) {
  if (n >= 0 && n < 256) g_isr[n] = nullptr;
}
void noInterrupts() {}
void interrupts() {}

// ------------------------------------------------------------------ misc
static Rng g_user_rng(99);
long random(long howbig) { return howbig <= 0 ? 0 : (long)(g_user_rng.next() % (uint64_t)howbig); }
long random(long a, long b) { return b <= a ? a : a + random(b - a); }
void randomSeed(unsigned long seed) { g_user_rng = Rng(seed); }
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ------------------------------------------------------------------ String
static std::string num_to_str(unsigned long v, int base) {
  if (base < 2) base = 10;
  if (v == 0) return "0";
  std::string s;
  while (v) {
    int d = (int)(v % base);
    s += (char)(d < 10 ? '0' + d : 'A' + d - 10);
    v /= base;
  }
  return std::string(s.rbegin(), s.rend());
}
static std::string float_to_str(double v, int digits) {
  if (std::isnan(v)) return "nan";
  if (std::isinf(v)) return "inf";
  if (v > 4294967040.0 || v < -4294967040.0) return "ovf";
  char buf[64];
  snprintf(buf, sizeof buf, "%.*f", digits, v);
  return buf;
}
String::String(unsigned char v, unsigned char base) : s_(num_to_str(v, base)) {}
String::String(int v, unsigned char base)
    : s_(v < 0 && base == 10 ? "-" + num_to_str((unsigned long)(-(long)v), 10) : num_to_str((unsigned int)v, base)) {}
String::String(unsigned int v, unsigned char base) : s_(num_to_str(v, base)) {}
String::String(long v, unsigned char base)
    : s_(v < 0 && base == 10 ? "-" + num_to_str((unsigned long)(-v), 10) : num_to_str((unsigned long)v, base)) {}
String::String(unsigned long v, unsigned char base) : s_(num_to_str(v, base)) {}
String::String(float v, unsigned char d) : s_(float_to_str(v, d)) {}
String::String(double v, unsigned char d) : s_(float_to_str(v, d)) {}

// ------------------------------------------------------------------ Print / Serial
size_t Print::printSigned(long v, int base) {
  if (base == 10 && v < 0) return write("-") + printNumber((unsigned long)(-v), 10);
  return printNumber((unsigned long)v, base);
}
size_t Print::printNumber(unsigned long v, int base) { return write(num_to_str(v, base).c_str()); }
size_t Print::printFloat(double v, int digits) { return write(float_to_str(v, digits).c_str()); }

SimSerial Serial(0), Serial1(1), Serial2(2), Serial3(3), Serial4(4);
static std::deque<uint8_t> g_rx[5];

namespace sim {
void serial_rx_push(int i, uint8_t b) {
  if (i < 0 || i > 4) return;
  g_rx[i].push_back(b);
  while (g_rx[i].size() > 256) g_rx[i].pop_front();  // UART buffer overflow drops old bytes
}
int serial_rx_size(int i) { return (i >= 0 && i <= 4) ? (int)g_rx[i].size() : 0; }
}  // namespace sim

int SimSerial::available() {
  sched_busy(2);
  return (int)g_rx[index_].size();
}
int SimSerial::read() {
  sched_busy(2);
  if (g_rx[index_].empty()) return -1;
  int b = g_rx[index_].front();
  g_rx[index_].pop_front();
  return b;
}
int SimSerial::peek() {
  sched_busy(1);
  return g_rx[index_].empty() ? -1 : g_rx[index_].front();
}
size_t SimSerial::write(uint8_t c) {
  if (index_ == 0) rec_serial_char((char)c);
  return 1;
}

// ------------------------------------------------------------------ Wire (I2C)
TwoWire Wire, Wire1;
void TwoWire::begin() {}
void TwoWire::begin(uint8_t) {}
void TwoWire::setClock(uint32_t) {}
void TwoWire::beginTransmission(uint8_t address) {
  tx_addr_ = address;
  tx_len_ = 0;
}
size_t TwoWire::write(uint8_t data) {
  if (tx_len_ < 32) tx_buf_[tx_len_++] = data;
  return 1;
}
size_t TwoWire::write(const uint8_t* data, size_t n) {
  for (size_t i = 0; i < n; i++) write(data[i]);
  return n;
}
uint8_t TwoWire::endTransmission(bool) {
  sched_busy(i2c_us(1 + tx_len_));
  if (tx_addr_ == 0x70 && tx_len_ >= 1) robot.mux_mask = tx_buf_[0];
  return robot.i2c_present(tx_addr_) ? 0 : 2;
}
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, bool) {
  sched_busy(i2c_us(1 + quantity));
  rx_len_ = rx_pos_ = 0;
  if (!robot.i2c_present(address)) return 0;
  for (int i = 0; i < quantity && i < 32; i++) rx_buf_[rx_len_++] = (address == 0x60) ? 0x01 : 0x00;
  return quantity;
}
int TwoWire::available() { return rx_len_ - rx_pos_; }
int TwoWire::read() { return rx_pos_ < rx_len_ ? rx_buf_[rx_pos_++] : -1; }
int TwoWire::peek() { return rx_pos_ < rx_len_ ? rx_buf_[rx_pos_] : -1; }

// ------------------------------------------------------------------ TCA9548A mux
bool QWIICMUX::begin(uint8_t a, TwoWire&) {
  addr_ = a;
  return isConnected();
}
bool QWIICMUX::isConnected() {
  sched_busy(i2c_us(1));
  return robot.mux_present;
}
bool QWIICMUX::setPort(uint8_t p) {
  if (p > 7) return false;
  return setPortState((uint8_t)(1 << p));
}
bool QWIICMUX::setPortState(uint8_t bits) {
  sched_busy(i2c_us(2));
  if (!robot.mux_present) return false;
  robot.mux_mask = bits;
  return true;
}
uint8_t QWIICMUX::getPort() {
  sched_busy(i2c_us(2));
  for (int i = 0; i < 8; i++)
    if (robot.mux_mask & (1 << i)) return (uint8_t)i;
  return 255;
}
uint8_t QWIICMUX::getPortState() {
  sched_busy(i2c_us(2));
  return (uint8_t)robot.mux_mask;
}
bool QWIICMUX::enablePort(uint8_t p) {
  if (p > 7) return false;
  return setPortState((uint8_t)(robot.mux_mask | (1 << p)));
}
bool QWIICMUX::disablePort(uint8_t p) {
  if (p > 7) return false;
  return setPortState((uint8_t)(robot.mux_mask & ~(1 << p)));
}

// ------------------------------------------------------------------ VL53L0X
static std::string hex2(int v) {
  char b[8];
  snprintf(b, sizeof b, "0x%02X", v & 0xFF);
  return b;
}
static std::string mux_desc() {
  std::string s;
  for (int i = 0; i < 8; i++)
    if (robot.mux_mask & (1 << i)) s += (s.empty() ? "" : ",") + std::to_string(i);
  return s.empty() ? "none" : s;
}

VL53L0X::VL53L0X() {}

void VL53L0X::setAddress(uint8_t new_addr) {
  sched_busy(i2c_us(3));
  TofDevice* d = robot.tof_on_bus(address_);
  if (d) d->address = new_addr & 0x7F;
  address_ = new_addr & 0x7F;
}

bool VL53L0X::init(bool) {
  TofDevice* d = robot.tof_on_bus(address_);
  if (!d) {
    sched_busy(i2c_us(3));
    last_status = 2;
    return false;
  }
  sched_busy(45000);  // ~300 register accesses plus two reference calibrations in the Pololu init()
  d->initialized = true;
  d->ranging = false;
  d->unread = false;
  d->period = (int64_t)budget_us_ + 600;
  last_status = 0;
  return true;
}

bool VL53L0X::setMeasurementTimingBudget(uint32_t budget_us) {
  if (budget_us < 20000) return false;
  sched_busy(i2c_us(20));
  budget_us_ = budget_us;
  TofDevice* d = robot.tof_on_bus(address_);
  if (d) d->period = (int64_t)budget_us + 600;
  return true;
}

void VL53L0X::startContinuous(uint32_t period_ms) {
  sched_busy(i2c_us(15));
  TofDevice* d = robot.tof_on_bus(address_);
  if (!d) return;
  d->ranging = true;
  d->period = std::max<int64_t>((int64_t)period_ms * 1000, (int64_t)budget_us_ + 600);
  d->next_ready = sched_now() + d->period;
  d->unread = false;
}

void VL53L0X::stopContinuous() {
  sched_busy(i2c_us(10));
  TofDevice* d = robot.tof_on_bus(address_);
  if (d) d->ranging = false;
}

uint16_t VL53L0X::readRangeContinuousMillimeters() {
  sched_sync();
  TofDevice* d = robot.tof_on_bus(address_);
  if (!d) {
    // Nobody answers at this address: Wire.read() returns 0xFF, so the library's
    // "data ready" check passes at once and the range reads as 0xFFFF.
    sched_busy(i2c_us(4 + 5 + 3));
    rec_warning("tof_noack_" + hex2(address_),
                "VL53L0X read at address " + hex2(address_) + " with mux port(s) " + mux_desc() +
                    " but no sensor answered there, so readRangeContinuousMillimeters() returned 65535.");
    return 65535;
  }
  int64_t start = sched_now();
  if (!d->ranging) {
    rec_warning("tof_not_started_" + std::to_string(d->port),
                "VL53L0X on mux port " + std::to_string(d->port) + " (sensor " + std::to_string(d->logical) +
                    ") was read before init()/startContinuous(). The library waits for a measurement "
                    "that never comes" +
                    (io_timeout_ == 0 ? " and setTimeout() was never called, so the call blocks forever "
                                        "(this is what happens after a cold power-on)."
                                      : "; it gives up after the timeout and returns 65535."));
    if (io_timeout_ == 0) {
      rec_event("hang", "waiting forever for VL53L0X on mux port " + std::to_string(d->port));
      for (;;) sched_busy(i2c_us(4));
    }
    sched_busy((int64_t)io_timeout_ * 1000);
    did_timeout_ = true;
    return 65535;
  }
  robot.update_tofs(sched_now());
  while (!d->unread) {
    int64_t wait = d->next_ready - sched_now();
    if (wait < 400) wait = 400;  // one status-register poll over I2C
    if (io_timeout_ > 0 && sched_now() + wait - start > (int64_t)io_timeout_ * 1000) {
      sched_busy((int64_t)io_timeout_ * 1000 - (sched_now() - start));
      did_timeout_ = true;
      return 65535;
    }
    sched_busy(wait);
    sched_sync();
    robot.update_tofs(sched_now());
  }
  sched_busy(i2c_us(4 + 5 + 3));  // status poll, 16-bit range read, interrupt clear
  d->unread = false;
  d->reads++;
  return (uint16_t)d->latest;
}

uint16_t VL53L0X::readRangeSingleMillimeters() {
  TofDevice* d = robot.tof_on_bus(address_);
  if (!d) {
    sched_busy(i2c_us(10));
    return 65535;
  }
  sched_busy(budget_us_ + 2000);
  sched_sync();
  return (uint16_t)robot.tof_measure(*d);
}

// ------------------------------------------------------------------ TCS34725 colour sensor
Adafruit_TCS34725::Adafruit_TCS34725(uint8_t it, tcs34725Gain_t gain) : it_(it), gain_(gain) {}

bool Adafruit_TCS34725::begin(uint8_t addr, TwoWire*) {
  sched_busy(i2c_us(4));
  robot.tcs_address = addr;
  if (!(robot.mux_mask & (1 << robot.tcs_port))) return false;
  initialised_ = true;
  sched_busy(i2c_us(6));
  enable();
  return true;
}

void Adafruit_TCS34725::enable() {
  sched_busy(i2c_us(3));
  delay(3);
  sched_busy(i2c_us(3));
  delay((256 - it_) * 12 / 5 + 1);
}
void Adafruit_TCS34725::disable() { sched_busy(i2c_us(6)); }
void Adafruit_TCS34725::setInterrupt(bool) { sched_busy(i2c_us(6)); }
void Adafruit_TCS34725::clearInterrupt() { sched_busy(i2c_us(2)); }

void Adafruit_TCS34725::getRawData(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
  if (!initialised_) begin();
  sched_busy(i2c_us(16));
  if (!(robot.mux_mask & (1 << robot.tcs_port))) {
    rec_warning("tcs_port", "TCS34725 read while its mux port (" + std::to_string(robot.tcs_port) +
                                ") was not selected; the reading is garbage.");
    *r = *g = *b = *c = 0;
  } else {
    sched_sync();
    robot.color_raw(r, g, b, c, it_, (int)gain_);
  }
  delay((256 - it_) * 12 / 5 + 1);  // the Adafruit library waits one integration time after reading
}

void Adafruit_TCS34725::getRGB(float* r, float* g, float* b) {
  uint16_t rr, gg, bb, cc;
  getRawData(&rr, &gg, &bb, &cc);
  if (cc == 0) {
    *r = *g = *b = 0;
    return;
  }
  *r = (float)rr / cc * 255.0f;
  *g = (float)gg / cc * 255.0f;
  *b = (float)bb / cc * 255.0f;
}
uint16_t Adafruit_TCS34725::calculateColorTemperature(uint16_t, uint16_t, uint16_t) { return 5000; }
uint16_t Adafruit_TCS34725::calculateColorTemperature_dn40(uint16_t, uint16_t, uint16_t, uint16_t) { return 5000; }
uint16_t Adafruit_TCS34725::calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  return (uint16_t)std::max(0.0, -0.32466 * r + 1.57837 * g - 0.73191 * b);
}

// ------------------------------------------------------------------ BNO055 IMU
Adafruit_BNO055::Adafruit_BNO055(int32_t id, uint8_t address, TwoWire*) : id_(id), addr_(address) {}

bool Adafruit_BNO055::begin(adafruit_bno055_opmode_t mode) {
  sched_busy(i2c_us(10));
  delay(700);  // reset + boot of the BNO055 inside the Adafruit begin()
  mode_ = mode;
  return true;
}

bool Adafruit_BNO055::getEvent(sensors_event_t* e) {
  sched_busy(i2c_us(9));
  sched_sync();
  memset(e, 0, sizeof *e);
  e->sensor_id = id_;
  e->timestamp = (int32_t)(sched_now() / 1000);
  e->orientation.x = (float)robot.gyro_heading_deg(sched_now());
  e->orientation.y = 0;
  e->orientation.z = (float)robot.gyro_pitch_deg();
  return true;
}

bool Adafruit_BNO055::getEvent(sensors_event_t* e, adafruit_vector_type_t type) {
  if (type == VECTOR_EULER) return getEvent(e);
  sched_busy(i2c_us(9));
  memset(e, 0, sizeof *e);
  return true;
}

imu::Vector<3> Adafruit_BNO055::getVector(adafruit_vector_type_t type) {
  sched_busy(i2c_us(9));
  sched_sync();
  if (type == VECTOR_EULER) return imu::Vector<3>(robot.gyro_heading_deg(sched_now()), 0, robot.gyro_pitch_deg());
  return imu::Vector<3>();
}

void Adafruit_BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
  sched_busy(i2c_us(3));
  if (sys) *sys = 3;
  if (gyro) *gyro = 3;
  if (accel) *accel = 3;
  if (mag) *mag = 3;
}

// ------------------------------------------------------------------ motor shield
Adafruit_MotorShield::Adafruit_MotorShield(uint8_t addr) : addr_(addr) {
  for (int i = 0; i < 4; i++) {
    motors_[i].sim_index = i;
    motors_[i].sim_shield = this;
  }
}

bool Adafruit_MotorShield::begin(uint16_t, TwoWire*) {
  sched_busy(i2c_us(30));
  delay(10);
  robot.shield_begun = true;
  for (int i = 0; i < 4; i++) {
    robot.mdir[i] = RELEASE;
    robot.mspeed[i] = 0;
  }
  return true;
}

Adafruit_DCMotor* Adafruit_MotorShield::getMotor(uint8_t n) {
  if (n < 1 || n > 4) return nullptr;
  return &motors_[n - 1];
}

void Adafruit_DCMotor::run(uint8_t cmd) {
  sched_busy(2 * i2c_us(6));  // two PCA9685 pin writes
  robot.mdir[sim_index] = cmd;
}

void Adafruit_DCMotor::setSpeed(uint8_t speed) {
  sched_busy(i2c_us(6));
  robot.mspeed[sim_index] = speed;
}

void Adafruit_DCMotor::setSpeed(int speed) {
  if (speed < 0 || speed > 255) {
    robot.speed_wraps++;
    rec_warning("speed_wrap", "Adafruit_DCMotor::setSpeed() takes a uint8_t, but the code passed values outside 0..255 "
                              "(first one: " + std::to_string(speed) + " became " +
                              std::to_string((uint8_t)speed) + "). Negative speeds turn into fast forward speeds.");
  }
  setSpeed((uint8_t)speed);
}

// ------------------------------------------------------------------ stepper (rescue kit dispenser)
Stepper::Stepper(int n, int, int) : steps_per_rev_(n) {}
Stepper::Stepper(int n, int, int, int, int) : steps_per_rev_(n) {}
void Stepper::setSpeed(long rpm) {
  if (rpm <= 0) rpm = 1;
  step_delay_us_ = (unsigned long)(60L * 1000L * 1000L / steps_per_rev_ / rpm);
}
void Stepper::step(int n) {
  rec_stepper(n);
  sched_busy((int64_t)std::abs(n) * (int64_t)step_delay_us_);
  sched_sync();
}

// ------------------------------------------------------------------ LCD
LiquidCrystal::LiquidCrystal(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t) {
  memset(text_, ' ', sizeof text_);
  for (auto& row : text_) row[40] = 0;
}
LiquidCrystal::LiquidCrystal(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t) {
  memset(text_, ' ', sizeof text_);
  for (auto& row : text_) row[40] = 0;
}
void LiquidCrystal::begin(uint8_t cols, uint8_t rows, uint8_t) {
  cols_ = cols;
  rows_ = rows > 4 ? 4 : rows;
  delay(50);
  clear();
}
void LiquidCrystal::clear() {
  sched_busy(2000);
  memset(text_, ' ', sizeof text_);
  for (int r = 0; r < 4; r++) {
    text_[r][40] = 0;
    g_lcd_line[r] = "";
  }
  col_ = row_ = 0;
}
void LiquidCrystal::home() {
  sched_busy(2000);
  col_ = row_ = 0;
}
void LiquidCrystal::setCursor(uint8_t c, uint8_t r) {
  sched_busy(50);
  col_ = c;
  row_ = r < rows_ ? r : rows_ - 1;
}
size_t LiquidCrystal::write(uint8_t c) {
  sched_busy(50);
  if (col_ < 40) {
    text_[row_][col_] = (char)c;
    col_++;
  } else if (row_ + 1 < rows_) {
    row_++;
    col_ = 0;
    text_[row_][col_++] = (char)c;
  }
  std::string s(text_[row_], cols_ < 40 ? cols_ : 40);
  size_t e = s.find_last_not_of(' ');
  g_lcd_line[row_] = e == std::string::npos ? "" : s.substr(0, e + 1);
  return 1;
}

// ------------------------------------------------------------------ mbed RTOS
namespace rtos {
Mutex::Mutex() : id_(sched_mutex_create("mutex")) {}
Mutex::Mutex(const char* name) : id_(sched_mutex_create(name)) {}
void Mutex::lock() { sched_mutex_lock(id_); }
bool Mutex::trylock() { return sched_mutex_trylock(id_); }
void Mutex::unlock() { sched_mutex_unlock(id_); }

Thread::Thread(osPriority priority, uint32_t, unsigned char*, const char* name)
    : prio_(priority), name_(name) {}
const char* sim_thread_name_for(void (*fn)());
osStatus Thread::start(void (*task)()) {
  const char* n = name_ ? name_ : sim_thread_name_for(task);
  id_ = sched_spawn(task, (int)prio_, n);
  return osOK;
}
osStatus Thread::join() {
  for (;;) sched_sleep(1000000);
}
osStatus Thread::terminate() { return osOK; }
osStatus Thread::set_priority(osPriority p) {
  prio_ = p;
  if (id_ >= 0) sched_set_priority(id_, (int)p);
  return osOK;
}
osPriority Thread::get_priority() const { return prio_; }

namespace ThisThread {
void sim_sleep_us(int64_t us) { sched_sleep(us); }
void yield() { sched_sleep(0); }
}  // namespace ThisThread
}  // namespace rtos
