#ifndef __LSB_HPP__
#define __LSB_HPP__

struct LSB {
  struct Data {
    bool busy;
    int robid;
    reg_t addr;
    byte store_length;  // 0 for byte, 1 for half word, 2 for word
  };
  Dual<Data> s;
};

#endif