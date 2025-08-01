#ifndef __LSB_HPP__
#define __LSB_HPP__

#include "type.hpp"

class CPU;

class LSB {
  static const int LSB_DELAY = 3;
  friend CPU;
  int counter;

public:
  enum Type {
    Load,
    Store
  };

  struct Data {
    Type type;
    bool busy;
    int robid, rsid;
    reg_t addr;
    byte store_length;  // 0 for byte, 1 for half word, 2 for word
    bool sign;
  };
  Dual<Data> s;
};

#endif