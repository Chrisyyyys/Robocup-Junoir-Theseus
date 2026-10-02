// Simulator version of the Arduino "Vector" library (fixed-capacity vector).
#pragma once
#include <vector>
#include <stddef.h>

template <typename T>
class Vector {
 public:
  Vector() {}
  template <size_t MAX_SIZE>
  Vector(T (&values)[MAX_SIZE], size_t size = 0) { setStorage(values, size); }
  template <size_t MAX_SIZE>
  void setStorage(T (&values)[MAX_SIZE], size_t size = 0) { cap_ = MAX_SIZE; v_.assign(values, values + size); }
  void setStorage(T* values, size_t max_size, size_t size) { cap_ = max_size; v_.assign(values, values + size); }
  T& operator[](size_t i) { return v_[i]; }
  const T& operator[](size_t i) const { return v_[i]; }
  T& at(size_t i) { return v_.at(i); }
  T& front() { return v_.front(); }
  T& back() { return v_.back(); }
  void clear() { v_.clear(); }
  void push_back(const T& value) { if (v_.size() < cap_) v_.push_back(value); }
  void pop_back() { v_.pop_back(); }
  void remove(size_t i) { v_.erase(v_.begin() + i); }
  size_t size() const { return v_.size(); }
  size_t max_size() const { return cap_; }
  bool empty() const { return v_.empty(); }
  bool full() const { return v_.size() >= cap_; }
  T* data() { return v_.data(); }
  typename std::vector<T>::iterator begin() { return v_.begin(); }
  typename std::vector<T>::iterator end() { return v_.end(); }

 private:
  std::vector<T> v_;
  size_t cap_ = (size_t)-1;
};
