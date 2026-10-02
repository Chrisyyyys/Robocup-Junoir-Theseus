// Deterministic scheduler for the robot code's RTOS threads, driven by virtual time.
//
// Every robot thread (loop(), the camera thread, the pause thread) is a real OS
// thread, but only one of them runs at any moment, exactly like on the GIGA's
// single core. Time only moves when the running thread does something that takes
// time on the robot (delay(), an I2C transfer, waiting for a sensor, ...). When a
// sleeping thread with equal or higher priority should wake up during that time,
// the scheduler switches to it at that exact virtual moment. The result does not
// depend on how fast the computer is, so a run with a given seed always repeats.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

#include "sim.h"

namespace sim {
namespace {

enum class St { Runnable, Busy, Sleeping, Blocked, Dead };

struct SThread {
  int id = 0;
  std::string name;
  int prio = 24;
  St st = St::Runnable;
  int64_t wake = 0;
  int64_t busy_left = 0;
  uint64_t ready_seq = 0;
  std::condition_variable cv;
  void (*fn)() = nullptr;
  int blocked_on = -1;
};

struct SMutex {
  std::string name;
  int owner = -1;
  int count = 0;
  std::vector<int> waiters;
};

std::mutex g_m;
std::vector<SThread*> g_threads;
int g_running = -1;
std::atomic<int64_t> g_now{0};
int64_t g_pending = 0;
int64_t g_next_event = 0;
uint64_t g_seq = 0;
bool g_started = false;
bool g_hook_active = false;
thread_local int t_self = -1;
void (*g_hook)(int64_t) = nullptr;

std::vector<SMutex>& mutexes() {
  static std::vector<SMutex> v;  // created on first use: rtos::Mutex globals register before main()
  return v;
}

const char* st_name(St s) {
  switch (s) {
    case St::Runnable: return "runnable";
    case St::Busy: return "busy";
    case St::Sleeping: return "sleeping";
    case St::Blocked: return "blocked";
    case St::Dead: return "finished";
  }
  return "?";
}

void recompute_next_event() {
  int64_t now = g_now.load();
  int64_t ne = (now / 1000 + 1) * 1000;  // next physics tick
  for (auto* t : g_threads)
    if (t->st == St::Sleeping && t->wake < ne) ne = t->wake;
  g_next_event = ne;
}

void advance_to(int64_t t) {
  if (t <= g_now.load()) return;
  g_hook_active = true;
  if (g_hook) g_hook(t);
  g_hook_active = false;
  g_now.store(t);
}

[[noreturn]] void deadlock() {
  finish("deadlock", "every robot thread is waiting for a mutex: " + sched_describe());
}

void switch_to(std::unique_lock<std::mutex>& lk, SThread* me, SThread* to) {
  g_running = to->id;
  to->cv.notify_one();
  if (me->st == St::Dead) return;
  me->cv.wait(lk, [&] { return g_running == me->id; });
  recompute_next_event();
}

// Run the scheduler until `me` may continue. Called with g_m held.
void schedule(std::unique_lock<std::mutex>& lk, SThread* me) {
  for (;;) {
    int64_t now = g_now.load();
    for (auto* t : g_threads) {
      if (t->st == St::Sleeping && t->wake <= now) {
        t->st = St::Runnable;
        t->ready_seq = ++g_seq;
      }
    }
    SThread* cand = nullptr;
    for (auto* t : g_threads) {
      if (t->st != St::Runnable && t->st != St::Busy) continue;
      if (!cand || t->prio > cand->prio) {
        cand = t;
        continue;
      }
      if (t->prio < cand->prio) continue;
      bool tr = t->st == St::Runnable, cr = cand->st == St::Runnable;
      if (tr != cr) {
        if (tr) cand = t;  // a thread that just woke up runs before a busy one of equal priority
      } else if (tr) {
        if (t->ready_seq < cand->ready_seq) cand = t;  // FIFO among ready threads
      } else if (t == me) {
        cand = t;  // the thread that was running keeps the CPU
      }
    }
    if (!cand) {
      int64_t nw = INT64_MAX;
      for (auto* t : g_threads)
        if (t->st == St::Sleeping && t->wake < nw) nw = t->wake;
      if (nw == INT64_MAX) deadlock();
      advance_to(nw);
      continue;
    }
    if (cand->st == St::Busy) {
      if (cand->busy_left <= 0) {
        cand->st = St::Runnable;
        cand->ready_seq = ++g_seq;
        continue;
      }
      int64_t until = now + cand->busy_left;
      for (auto* t : g_threads)
        if (t != cand && t->st == St::Sleeping && t->prio >= cand->prio && t->wake < until) until = t->wake;
      if (until <= now) until = now + 1;
      cand->busy_left -= (until - now);
      advance_to(until);
      continue;
    }
    if (cand == me) {
      recompute_next_event();
      return;
    }
    recompute_next_event();
    switch_to(lk, me, cand);
    return;
  }
}

void commit_pending(std::unique_lock<std::mutex>& lk) {
  if (g_pending <= 0) return;
  SThread* me = g_threads[t_self];
  me->st = St::Busy;
  me->busy_left = g_pending;
  g_pending = 0;
  schedule(lk, me);
}

void thread_main(SThread* t) {
  {
    std::unique_lock<std::mutex> lk(g_m);
    t->cv.wait(lk, [&] { return g_running == t->id; });
    t_self = t->id;
    recompute_next_event();
  }
  t->fn();
  std::unique_lock<std::mutex> lk(g_m);
  g_pending = 0;
  t->st = St::Dead;
  schedule(lk, t);
}

}  // namespace

void sched_init_main(const char* name) {
  std::unique_lock<std::mutex> lk(g_m);
  SThread* t = new SThread;
  t->id = 0;
  t->name = name;
  t->prio = 24;
  g_threads.push_back(t);
  g_running = 0;
  t_self = 0;
  g_started = true;
  recompute_next_event();
}

bool sched_started() { return g_started; }
bool sched_in_hook() { return g_hook_active; }
int sched_current() { return t_self; }
const char* sched_thread_name(int id) {
  return (id >= 0 && id < (int)g_threads.size()) ? g_threads[id]->name.c_str() : "?";
}
void sched_set_world_hook(void (*advance)(int64_t)) { g_hook = advance; }

int64_t sched_now() {
  if (g_hook_active || t_self < 0) return g_now.load();
  return g_now.load() + g_pending;
}
int64_t sched_committed_now() { return g_now.load(); }

void sched_busy(int64_t us) {
  if (!g_started || g_hook_active || t_self < 0 || us <= 0) return;
  g_pending += us;
  if (g_now.load() + g_pending < g_next_event) return;
  std::unique_lock<std::mutex> lk(g_m);
  commit_pending(lk);
}

void sched_sync() {
  if (!g_started || g_hook_active || t_self < 0 || g_pending <= 0) return;
  std::unique_lock<std::mutex> lk(g_m);
  commit_pending(lk);
}

void sched_sleep(int64_t us) {
  if (!g_started || g_hook_active || t_self < 0) return;
  std::unique_lock<std::mutex> lk(g_m);
  commit_pending(lk);
  SThread* me = g_threads[t_self];
  if (us <= 0) {
    me->st = St::Runnable;
    me->ready_seq = ++g_seq;
  } else {
    me->st = St::Sleeping;
    me->wake = g_now.load() + us;
  }
  schedule(lk, me);
}

int sched_spawn(void (*fn)(), int prio, const char* name) {
  std::unique_lock<std::mutex> lk(g_m);
  commit_pending(lk);
  SThread* t = new SThread;
  t->id = (int)g_threads.size();
  t->name = name ? name : "thread";
  t->prio = prio;
  t->fn = fn;
  t->st = St::Runnable;
  t->ready_seq = ++g_seq;
  g_threads.push_back(t);
  std::thread(thread_main, t).detach();
  SThread* me = g_threads[t_self];
  if (prio > me->prio) {
    me->st = St::Runnable;
    me->ready_seq = ++g_seq;
    schedule(lk, me);
  }
  return t->id;
}

void sched_set_priority(int id, int prio) {
  if (id < 0) return;
  std::unique_lock<std::mutex> lk(g_m);
  commit_pending(lk);
  g_threads[id]->prio = prio;
  SThread* me = g_threads[t_self];
  me->st = St::Runnable;
  schedule(lk, me);
}

int sched_get_priority(int id) { return (id >= 0 && id < (int)g_threads.size()) ? g_threads[id]->prio : 0; }

int sched_mutex_create(const char* name) {
  auto& m = mutexes();
  m.push_back(SMutex());
  m.back().name = name ? name : "mutex";
  return (int)m.size() - 1;
}

void sched_mutex_lock(int id) {
  if (!g_started || t_self < 0 || g_hook_active) return;
  {
    SMutex& m = mutexes()[id];
    if (m.owner == -1 || m.owner == t_self) {
      m.owner = t_self;
      m.count++;
      return;
    }
  }
  std::unique_lock<std::mutex> lk(g_m);
  commit_pending(lk);
  SMutex& m = mutexes()[id];
  if (m.owner == -1 || m.owner == t_self) {
    m.owner = t_self;
    m.count++;
    return;
  }
  SThread* me = g_threads[t_self];
  me->st = St::Blocked;
  me->blocked_on = id;
  m.waiters.push_back(t_self);
  schedule(lk, me);  // unlock() hands the mutex over before waking us
}

bool sched_mutex_trylock(int id) {
  if (!g_started || t_self < 0 || g_hook_active) return true;
  SMutex& m = mutexes()[id];
  if (m.owner == -1 || m.owner == t_self) {
    m.owner = t_self;
    m.count++;
    return true;
  }
  return false;
}

void sched_mutex_unlock(int id) {
  if (!g_started || t_self < 0 || g_hook_active) return;
  {
    SMutex& m = mutexes()[id];
    if (m.owner != t_self) return;
    if (--m.count > 0) return;
    if (m.waiters.empty()) {
      m.owner = -1;
      return;
    }
  }
  std::unique_lock<std::mutex> lk(g_m);
  commit_pending(lk);  // we keep the mutex while our pending CPU time passes
  SMutex& m = mutexes()[id];
  int best = -1, bi = -1;
  for (int i = 0; i < (int)m.waiters.size(); i++) {
    int w = m.waiters[i];
    if (best < 0 || g_threads[w]->prio > g_threads[best]->prio) {
      best = w;
      bi = i;
    }
  }
  m.waiters.erase(m.waiters.begin() + bi);
  m.owner = best;
  m.count = 1;
  SThread* w = g_threads[best];
  w->st = St::Runnable;
  w->blocked_on = -1;
  w->ready_seq = ++g_seq;
  SThread* me = g_threads[t_self];
  if (w->prio > me->prio) {
    me->st = St::Runnable;
    me->ready_seq = ++g_seq;
    schedule(lk, me);
  }
}

std::string sched_describe() {
  std::string s;
  for (auto* t : g_threads) {
    if (!s.empty()) s += ", ";
    s += t->name + "=" + st_name(t->st);
    if (t->st == St::Blocked && t->blocked_on >= 0) s += "(on " + mutexes()[t->blocked_on].name + ")";
  }
  return s;
}

void sched_start_watchdog(double wall_seconds) {
  std::thread([wall_seconds] {
    int64_t last = -1;
    auto last_change = std::chrono::steady_clock::now();
    for (;;) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      int64_t n = g_now.load();
      auto now = std::chrono::steady_clock::now();
      if (n != last) {
        last = n;
        last_change = now;
        continue;
      }
      double idle = std::chrono::duration<double>(now - last_change).count();
      if (idle > wall_seconds) {
        std::string who = (g_running >= 0 && g_running < (int)g_threads.size()) ? g_threads[g_running]->name : "?";
        finish("hang", "the robot code in thread '" + who +
                           "' is stuck in a loop that never waits or reads hardware, so time cannot advance");
      }
    }
  }).detach();
}

}  // namespace sim
