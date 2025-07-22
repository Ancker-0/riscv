#ifndef __TYPE_HPP__
#define __TYPE_HPP__

#include "config.hpp"

#include <cstdint>
#include <vector>
#include <functional>

typedef uint8_t byte;
typedef uint32_t uint32;
typedef uint32_t word;

template <typename T>
class Proxy {
  using thunk = std::function<void(void)>;
  T val;
  std::vector<thunk> ths;

  void trigger() {
    for (auto &f : ths)
      f();
  }

public:
  Proxy() = default;
  Proxy(const T &v) : val(v) {}
  // void Set(const T &v) { val = v; trigger(); }
  T Set(const T &v) { val = v; trigger(); }
  Proxy &operator=(const T &v) { val = v; return *this; }
  Proxy &operator=(const Proxy &v) { val = v.val; return *this; }
  void Register(thunk th) { ths.push_back(th); }
  operator T() {
    return val;
  }
};

#if RISCV_ENABLE64
typedef uint32_t reg_t;
#else
typedef uint64_t reg_t;
#endif

#endif
