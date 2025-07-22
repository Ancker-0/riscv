#ifndef __INSTRUCTION_HPP__
#define __INSTRUCTION_HPP__

#include "register.hpp"
#include "type.hpp"

struct RV32_Instruction {
  virtual void Parse(word cmd);
};

static constexpr inline word Mask(byte x) { return (1 << x) - 1; }
static constexpr inline word Mask(byte l, byte r) { return Mask(r - l + 1) << l; }
static constexpr inline word SignFrom(word c, byte pos) { return (int32_t)c << (31 - pos) >> (31 - pos); }

struct RV32_U : public RV32_Instruction {
  RegIdx rd;
  word imm;
  void Parse(word cmd) override {
    imm = cmd & ~Mask(12);
    rd = RegIdx(cmd >> 7 & Mask(5));
  }
};

struct RV32_J : public RV32_Instruction {
  RegIdx rd;
  word imm;
  void Parse(word cmd) override {
    imm = (cmd & Mask(8) << 12) | (cmd >> 9 & 1 << 11) | (cmd >> 20 & Mask(1, 10)) | ((int32_t)cmd >> 11 & 1 << 20);  // signed
    rd = RegIdx(cmd >> 7 & Mask(5));
  }
};

struct RV32_I : public RV32_Instruction {
  RegIdx rd, rs1;
  word imm;
  void Parse(word cmd) override {
    imm = cmd >> 25;
    rd = RegIdx(cmd >> 7 & Mask(5));
    rs1 = RegIdx(cmd >> 15 & Mask(5));
  }
  void Sign() {
    imm = SignFrom(imm, 11);
  }
};

struct RV32_B : public RV32_Instruction {
  RegIdx rd, rs1, rs2;
  word imm;
  void Parse(word cmd) override {
    imm = (cmd >> 7 & Mask(1, 4)) | (cmd << 4 & 1 << 11) | (cmd >> 20 & Mask(5, 10)) | (cmd >> 19 & 1 << 12);
    rd = RegIdx(cmd >> 7 & Mask(5));
    rs1 = RegIdx(cmd >> 15 & Mask(5));
    rs2 = RegIdx(cmd >> 20 & Mask(5));
  }
  void Sign() {
    imm = SignFrom(imm, 12);
  }
};

struct RV32_S : public RV32_Instruction {
  RegIdx rd, rs1, rs2;
  word imm;
  void Parse(word cmd) override {
    imm = (cmd >> 7 & Mask(0, 4)) | (cmd >> 20 & Mask(5, 11));
    rd = RegIdx(cmd >> 7 & Mask(5));
    rs1 = RegIdx(cmd >> 15 & Mask(5));
    rs2 = RegIdx(cmd >> 20 & Mask(5));
  }
  void Sign() {
    imm = SignFrom(imm, 12);
  }
};

struct RV32_R : public RV32_Instruction {
  RegIdx rd, rs1, rs2;
  void Parse(word cmd) override {
    rd = RegIdx(cmd >> 7 & Mask(5));
    rs1 = RegIdx(cmd >> 15 & Mask(5));
    rs2 = RegIdx(cmd >> 20 & Mask(5));
  }
};

#endif
