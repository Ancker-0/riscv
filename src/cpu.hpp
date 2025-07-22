#ifndef __CPU_HPP__
#define __CPU_HPP__

#include "type.hpp"
#include "memory.hpp"
#include "log.hpp"
#include <vector>
#include <memory>

class CPU {
  Mem &mem;
  reg_t pc;

  struct copy_pipe {
    virtual void copy();
  };
  template<typename T>
  struct copy_pipe_impl : public copy_pipe {
    T *dest, *src;
    void copy() override { *dest = *src; }
  };
  std::vector<std::unique_ptr<copy_pipe>> conn;

public:

  template <typename T>
  void Connect(T &dest, T &src) {  // chronic connect
    conn.push_back({&dest, &src});
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

    for (auto &p : conn)
      p->copy();
  }
};

#endif
