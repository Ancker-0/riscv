#ifndef __ALU_HPP__
#define __ALU_HPP__

#include "type.hpp"

const int ALU_size = 8;

struct ALU {
  enum Type {
    ADD
  };
  struct Unit {
    bool busy;
    int robid;
    Type type;
    word a, b;
  };
  std::array<Dual<Unit>, ALU_size> as;

  int validId(int start) {
    for (int i = start; i < ALU_size; ++i)
      if (!as[i].pre.get().busy)
        return i;
    return -1;
  }
};

#endif