#ifndef __LSB_HPP__
#define __LSB_HPP__

#include "type.hpp"

class CPU;

class LSB {
  static const int LSB_DELAY = 3;
  static const int LSB_size = 8;
  friend CPU;
  int counter;

public:
  enum Type {
    Load,
    Store
  };

  enum State {
    Prepare,
    Ready,
    Executing,
  };

  struct Data {
    Type type;
    bool busy;
    int robid;
    reg_t addr;
    byte store_length;  // 0 for byte, 1 for half word, 2 for word
    bool sign;
    State state;
  };
  RQueue<Proxy<Data>, LSB_size> qold, q;
};

#endif