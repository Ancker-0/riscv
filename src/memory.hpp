#ifndef __MEMORY_HPP__
#define __MEMORY_HPP__

#include "type.hpp"
#include <unordered_map>

class Mem : public std::unordered_map<reg_t, byte> {
  using faT = std::unordered_map<reg_t, byte>;
public:
  struct {
    Proxy<reg_t> get1;
    Proxy<std::pair<reg_t, word>> put2;
  } I;
  struct {
    Proxy<word> res1;
    Proxy<bool> done2;
  } O;

  byte &operator[](const reg_t &p) {
    return faT::operator[](p);
  }

  void Run(CPU *cpu) {
  }
};

class IFetcher {
public:
  struct {
  } I;
  struct {
  } O;
  void Run() {
  }
};

#endif
