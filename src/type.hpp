#ifndef __TYPE_HPP__
#define __TYPE_HPP__

#include "config.hpp"

#include <cstdint>
#include <vector>
#include <functional>

typedef uint8_t byte;
typedef int8_t sbyte;
typedef uint32_t uint32;
typedef uint32_t word;

template <typename T>
class Proxy {
  using thunk = std::function<void(void)>;
  T val;
  bool changed;
  std::vector<thunk> ths;

  void trigger() {
    for (auto &f : ths)
      f();
  }

public:
  Proxy() = default;
  Proxy(const T &v) : val(v) {}
  // void Set(const T &v) { val = v; trigger(); }
  Proxy &Set(const T &v) { val = v; changed = true; trigger(); return *this; }
  T Get() { return val; }
  operator T() { return val; }
  Proxy &operator=(const T &v) { return Set(v); }
  Proxy &operator=(Proxy &v) { if (v.changed) { Set(v.val); v.changed = false; }; return *this; }
  void Register(thunk th) { ths.push_back(th); }
};

template <typename T>
struct Proto {
  bool valid = false;
  bool ready = true;
  T data;
};
template <typename T> using PProto = Proto<Proxy<T>>;

#if RISCV_ENABLE64
typedef uint64_t reg_t;
typedef int64_t sreg_t;
#else
typedef uint32_t reg_t;
typedef int32_t sreg_t;
#endif

#endif
