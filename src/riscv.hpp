#ifndef __RISCV_HPP__
#define __RISCV_HPP__

#include "decoder.hpp"
#include "cpu.hpp"
#include "memory.hpp"
#include "register.hpp"
#include <vector>

namespace RV {
  // struct copy_pipe {
  //   virtual void copy() = 0;
  //   virtual ~copy_pipe() = default;
  // };

  // template<typename T>
  // struct copy_pipe_impl : public copy_pipe {
  //   T *dest, *src;
  //   copy_pipe_impl(T *dest_, T *src_) : dest(dest_), src(src_) {}
  //   void copy() override { *dest = *src; }
  // };

  // std::vector<std::unique_ptr<copy_pipe>> conn;

  // template <typename T>
  // void Connect(T &dest, T &src) {  // chronic connect
  //   conn.push_back(std::make_unique<copy_pipe_impl<T>>(&dest, &src));
  // }

  // It remains a technical problem to logically connect the stuffs.

  std::unique_ptr<Mem> mem;
  // Reg reg;
  Decoder dec;
  RS rs;
  RF rf;
  RoB rob;
  ALU alu;
  LSB lsb;
  NaivePredictor pred;
  void Run(Mem &&mem_) {
    mem = std::make_unique<Mem>(mem_);
    CPU cpu(mem.get(), &dec, &rs, &rf, &rob, &alu, &lsb, &pred);
    cpu.Run();
  }
}

#endif
