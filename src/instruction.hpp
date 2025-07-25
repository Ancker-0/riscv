#ifndef __INSTRUCTION_HPP__
#define __INSTRUCTION_HPP__

#include "register.hpp"
#include "type.hpp"

struct RV32_Instruction;
struct RV32_U;
struct RV32_J;
struct RV32_I;
struct RV32_B;
struct RV32_S;
struct RV32_R;

struct Visitor {
  virtual void Visit(RV32_U *s) = 0;
  virtual void Visit(RV32_J *s) = 0;
  virtual void Visit(RV32_I *s) = 0;
  virtual void Visit(RV32_B *s) = 0;
  virtual void Visit(RV32_S *s) = 0;
  virtual void Visit(RV32_R *s) = 0;
};

struct RV32_Instruction {
  virtual void Parse(word cmd) = 0;
  virtual void Exec(Visitor &v) = 0;
  // virtual ~RV32_Instruction() = default;
};

static constexpr inline word Mask(byte x) { return (1 << x) - 1; }
static constexpr inline word Mask(byte l, byte r) { return Mask(r - l + 1) << l; }
static constexpr inline word SignFrom(word c, byte pos) { return (int32_t)(c << (31 - pos)) >> (31 - pos); }

struct RV32_U : public RV32_Instruction {
  byte opcode;
  RegIdx rd;
  word imm;
  void Parse(word cmd) override {
    opcode = cmd & Mask(7);
    imm = cmd & ~Mask(12);
    rd = RegIdx(cmd >> 7 & Mask(5));
  }
  void Exec(Visitor &v) override { v.Visit(this); }
};

struct RV32_J : public RV32_Instruction {
  byte opcode;
  RegIdx rd;
  word imm;
  void Parse(word cmd) override {
    opcode = cmd & Mask(7);
    // imm = (cmd & Mask(12, 19)) | (cmd >> 9 & 1 << 11) | (cmd >> 20 & Mask(1, 10)) | ((int32_t)cmd >> 11 & Mask(20, 31));  // signed
    imm = ((cmd >> 31) & 0x1) << 20 | ((cmd >> 21) & 0x3FF) << 1 | ((cmd >> 20) & 0x1) << 11 | ((cmd >> 12) & 0xFF) << 12;
    imm = SignFrom(imm, 20);

    rd = RegIdx(cmd >> 7 & Mask(5));
  }
  void Exec(Visitor &v) override { v.Visit(this); }
};

struct RV32_I : public RV32_Instruction {
  byte opcode, fn3;
  RegIdx rd, rs1;
  word imm;
  void Parse(word cmd) override {
    opcode = cmd & Mask(7);
    fn3 = cmd >> 12 & Mask(3);
    imm = cmd >> 20;
    rd = RegIdx(cmd >> 7 & Mask(5));
    rs1 = RegIdx(cmd >> 15 & Mask(5));
  }
  void Sign() {
    imm = SignFrom(imm, 11);
  }
  void Exec(Visitor &v) override { v.Visit(this); }
};

struct RV32_B : public RV32_Instruction {
  byte opcode, fn3;
  RegIdx rd, rs1, rs2;
  word imm;
  void Parse(word cmd) override {
    opcode = cmd & Mask(7);
    fn3 = cmd >> 12 & Mask(3);
    // imm = (cmd >> 7 & Mask(1, 4)) | (cmd << 4 & 1 << 11) | (cmd >> 20 & Mask(5, 10)) | (cmd >> 19 & 1 << 12);
    imm = ((cmd >> 31) & 0x1) << 12 | ((cmd >> 7) & 0x1E) | ((cmd >> 25) & 0x3F) << 5 | ((cmd >> 7) & 0x1) << 11;
    // imm = SignFrom(imm, 12);


    rd = RegIdx(cmd >> 7 & Mask(5));
    rs1 = RegIdx(cmd >> 15 & Mask(5));
    rs2 = RegIdx(cmd >> 20 & Mask(5));
  }
  void Sign() {
    imm = SignFrom(imm, 12);
  }
  void Exec(Visitor &v) override { v.Visit(this); }
};

struct RV32_S : public RV32_Instruction {
  byte opcode, fn3;
  RegIdx rd, rs1, rs2;
  word imm;
  void Parse(word cmd) override {
    opcode = cmd & Mask(7);
    fn3 = cmd >> 12 & Mask(3);
    imm = (cmd >> 7 & Mask(0, 4)) | (cmd >> 20 & Mask(5, 11));
    rd = RegIdx(cmd >> 7 & Mask(5));
    rs1 = RegIdx(cmd >> 15 & Mask(5));
    rs2 = RegIdx(cmd >> 20 & Mask(5));
  }
  void Sign() {
    imm = SignFrom(imm, 11);
  }
  void Exec(Visitor &v) override { v.Visit(this); }
};

struct RV32_R : public RV32_Instruction {
  byte opcode, fn3, fn7;
  RegIdx rd, rs1, rs2;
  void Parse(word cmd) override {
    opcode = cmd & Mask(7);
    fn3 = cmd >> 12 & Mask(3);
    fn7 = cmd >> 25 & Mask(7);
    rd = RegIdx(cmd >> 7 & Mask(5));
    rs1 = RegIdx(cmd >> 15 & Mask(5));
    rs2 = RegIdx(cmd >> 20 & Mask(5));
  }
  void Exec(Visitor &v) override { v.Visit(this); }
};

#endif
