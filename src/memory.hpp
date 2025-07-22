#ifndef __MEMORY_HPP__
#define __MEMORY_HPP__

#include "type.hpp"
#include <unordered_map>

class Mem : public std::unordered_map<reg_t, byte> {
  using faT = std::unordered_map<reg_t, byte>;
public:
  struct {
    Proxy<reg_t> get1;
  } I;
  struct {
    Proxy<word> res1;
  } O;

  byte &operator[](const reg_t &p) {
    return faT::operator[](p);
  }

  void Run() {
  }
};

#endif
