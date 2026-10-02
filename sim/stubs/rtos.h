// Simulator version of mbed's rtos API (Thread, Mutex, ThisThread).
// Threads are real OS threads, but the simulator runs exactly one at a time and
// switches between them in virtual time, like the RTOS on the GIGA does on one core.
#pragma once
#include <chrono>
#include <stdint.h>

typedef enum {
  osPriorityNone = 0,
  osPriorityIdle = 1,
  osPriorityLow = 8,
  osPriorityBelowNormal = 16,
  osPriorityNormal = 24,
  osPriorityAboveNormal = 32,
  osPriorityHigh = 40,
  osPriorityRealtime = 48,
  osPriorityISR = 56,
  osPriorityError = -1,
} osPriority_t;
typedef osPriority_t osPriority;
typedef int32_t osStatus;
#define osOK 0

namespace rtos {

class Mutex {
 public:
  Mutex();
  explicit Mutex(const char* name);
  void lock();
  bool trylock();
  void unlock();
  int sim_id() const { return id_; }

 private:
  int id_;
};

class Thread {
 public:
  explicit Thread(osPriority priority = osPriorityNormal, uint32_t stack_size = 4096,
                  unsigned char* stack_mem = nullptr, const char* name = nullptr);
  osStatus start(void (*task)());
  osStatus join();
  osStatus terminate();
  osStatus set_priority(osPriority priority);
  osPriority get_priority() const;
  int sim_id() const { return id_; }

 private:
  osPriority prio_;
  const char* name_;
  int id_ = -1;
};

namespace ThisThread {
void sim_sleep_us(int64_t us);
template <class Rep, class Period>
void sleep_for(std::chrono::duration<Rep, Period> d) {
  sim_sleep_us((int64_t)std::chrono::duration_cast<std::chrono::microseconds>(d).count());
}
inline void sleep_for(uint32_t ms) { sim_sleep_us((int64_t)ms * 1000); }
void yield();
}  // namespace ThisThread

}  // namespace rtos

using namespace std::chrono_literals;
