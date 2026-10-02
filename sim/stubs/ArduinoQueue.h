// Simulator version of the ArduinoQueue library (FIFO queue).
#pragma once
#include <deque>

template <typename T>
class ArduinoQueue {
 public:
  ArduinoQueue(unsigned int maxitems = 100, unsigned int maxMemory = 0) : max_(maxitems) { (void)maxMemory; }
  bool enqueue(const T& item) { if (q_.size() >= max_) return false; q_.push_back(item); return true; }
  T dequeue() { T v = q_.front(); q_.pop_front(); return v; }
  T getHead() { return q_.front(); }
  T getTail() { return q_.back(); }
  unsigned int itemCount() { return (unsigned int)q_.size(); }
  bool isEmpty() { return q_.empty(); }
  bool isFull() { return q_.size() >= max_; }
  unsigned int maxQueueSize() { return max_; }

 private:
  std::deque<T> q_;
  unsigned int max_;
};
