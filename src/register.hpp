#ifndef __REGISTER_HPP__
#define __REGISTER_HPP__

#include "config.hpp"
#include "type.hpp"
#include <cassert>

class Reg;

class RegIdx {
  word p;
  friend Reg;

public:
  explicit RegIdx(const word &p_ = 0) : p(p_) {
    assert(p_ < 32);
  }
};

class Reg {
  reg_t reg[32];
public:
  reg_t &operator[](const RegIdx &k) {
    if (k.p == 0)
      return reg[0] = 0;
    return reg[k.p];
  }

  void Run() {
  }
};

#endif
