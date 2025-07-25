#ifndef __DECODER_HPP__
#define __DECODER_HPP__

class Decoder {
  reg_t PC;

  enum {
    Idle,
    Waiting
  } state = Idle;

public:
  struct {
    bool rst;
    Proxy<word> res1;
  } I;
  struct {
    Proxy<reg_t> get1;
    byte eop;
  } O;

  void Run() {
    if (!I.rst) {
      state = Idle;
      return;
    }
  }
};

class IFecher {
public:
  struct {
  } I;
  struct {
  } O;
  void Run() {
  }
};

#endif
