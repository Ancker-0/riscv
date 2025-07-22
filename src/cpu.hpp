#ifndef __CPU_HPP__
#define __CPU_HPP__

#include "type.hpp"
#include "memory.hpp"
#include "log.hpp"
#include <vector>

class CPU {
  // Mem &mem;
  reg_t pc;

  struct connection {
    size_t sz;
    void *dest, *src;
  };
  std::vector<connection> conn;

public:

  template <typename T>
  void Connect(T &dest, T &src) {  // chronic connect
    conn.push_back({sizeof(T), (void*)&dest, (void*)&src});
  }

  // It remains a technical problem to logically connect the stuffs.

  // explicit CPU(Mem *mem_) : mem(*mem_) {
  //   pc = 0x00;
  // }

  void Step() {
    // byte identifier = mem[pc];
    // if ((identifier & 0x3) != 0x3) {
    //   log.Warn("C extension is not supported");
    //   pc += 2;
    //   return;
    // }

    // word cmd = mem[pc] | ((word)mem[pc + 1] << 8) | ((word)mem[pc + 2] << 16) | ((word)mem[pc + 3] << 24);
  }
};

#endif
