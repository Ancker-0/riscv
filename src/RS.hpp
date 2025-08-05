#ifndef __RS_HPP__
#define __RS_HPP__

#include "type.hpp"
#include "register.hpp"
#include <array>

constexpr int RS_size = 8;
const int RoB_size = 16;

struct RS {
  enum RSState {
    rsprepare,
    rsready,
    rsexecuting,
    rsdone,
  };
  static const byte PCID = 32, FREEID = 255, INVALID_ID = 254;
  struct RS_shot {
    bool busy;
    int opcode, fn3, fn7;
    int robid;  // can also be lsbid
    word result;
    word Vj, Vk;
    byte Qj, Qk;
    reg_t A;
    RSState state;
  };

  int emptyslot() {
    for (int i = 0; i < RS_size; ++i)
      if (not qold[i].get().busy)
        return i;
    return -1;
  }
  std::array<Proxy<RS_shot>, RS_size> q, qold;
};

struct RoB {
  enum RoBType{
    robbranch,
    robmem,
    robreg,
    robend,
  };
  enum RoBState {
    robbusy,
    robready,
    robexecuting,
    robfinish,
  };
  struct RoBc {
    RoBType type;
    RegIdx dest;
    reg_t memdest;
    byte store_length;  // 0 for byte, 1 for half word, 2 for word
    word val;
    reg_t PC;
    RoBState state;
  };
  // std::array<Proxy<RoBc>, RoB_size> qold;
  RQueue<Proxy<RoBc>, RoB_size> qold;
  RQueue<Proxy<RoBc>, RoB_size> q;
};

#endif
