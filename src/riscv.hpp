#ifndef __RISCV_HPP__
#define __RISCV_HPP__

#include "cpu.hpp"
#include "memory.hpp"
#include "register.hpp"

namespace RV {
  CPU cpu;
  Mem mem;
  Reg reg;

  void Init(const Mem &mem_) {
  }
}

#endif
