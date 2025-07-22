#ifndef __DECODER_HPP__
#define __DECODER_HPP__

class Decoder {
public:
  struct {
    Proxy<word> res1;
  } I;
  struct {
    Proxy<reg_t> get1;
    byte eop;
  } O;

  void Run() {
  }
};

#endif
