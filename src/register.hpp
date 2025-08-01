#ifndef __REGISTER_HPP__
#define __REGISTER_HPP__

#include "config.hpp"
#include "type.hpp"

#include <array>
#include <cassert>

class Reg;
class RF;

class RegIdx {
  friend Reg;
  friend RF;

public:
  word p;
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

class RF {
public:
  struct RFc {
    reg_t val;
    bool busy;
    int dependRoB;
  };
  typedef std::array<Proxy<RFc>, 32> RFcs;
  RFcs pre, now;
  void Upd(RegIdx x, int robId) {
    now[x.p].set(RFc{ (reg_t)-1, true, robId });
  }
  void UpdVal(RegIdx x, reg_t val) {
    now[x.p].set(RFc{ val, false, -1 });
  }
};

#endif
