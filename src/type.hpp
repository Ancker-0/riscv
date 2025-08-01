#ifndef __TYPE_HPP__
#define __TYPE_HPP__

#include "config.hpp"

#include <cstdint>
#include <cstring>
#include <vector>
#include <functional>

typedef uint8_t byte;
typedef int8_t sbyte;
typedef uint32_t uint32;
typedef uint32_t word;
class CPU;

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
  // void set(const T &v) { val = v; trigger(); }
  Proxy &set(const T &v) { val = v; changed = true; trigger(); return *this; }
  Proxy &set() { changed = true; trigger(); return *this; }
  void reset(const T &v) { changed = false; }
  void give(Proxy &o) { if (changed) o.val = val, o.changed = true, changed = false; }
  T &get() { return val; }
  const T &get() const { return val; }
  operator T() { return val; }
  // Proxy &operator=(const T &v) { return set(v); }
  // Proxy &operator=(Proxy &v) { if (v.changed) { set(v.val); v.changed = false; }; return *this; }
  void Register(thunk th) { ths.push_back(th); }
};

template <class T>
struct Dual {
  Proxy<T> pre, now;
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

template <class T, int N_>
class RQueue {
  static constexpr int N = N_ + 1;
  static int nxt(int x) { return (x + 1) % N; }
public:
  T q[N];
  int hd = 0, tl = 0;
  RQueue &operator=(const RQueue &o) { memcpy(q, o.q, sizeof(T) * N); hd = o.hd; tl = o.tl; return *this; }
  // byte push(const T &x) { q[tl] = x; byte ret = tl; tl = nxt(tl); return ret; }
  const T &front() { return q[hd]; }
  void pop() { hd = nxt(hd); }
  bool empty() { return tl == hd; }
  bool full() { return nxt(tl) == hd; }
};

#endif
