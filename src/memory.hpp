#ifndef __MEMORY_HPP__
#define __MEMORY_HPP__

#include "type.hpp"
#include <unordered_map>

class Mem : public std::unordered_map<reg_t, byte> {
  using faT = std::unordered_map<reg_t, byte>;
public:
  struct {
    Proxy<bool> rst;
  } I;
  struct {
  } O;

  byte &operator[](const reg_t &p) {
    return faT::operator[](p);
  }
};

#endif
