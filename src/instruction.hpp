#ifndef __INSTRUCTION_HPP__
#define __INSTRUCTION_HPP__

#include "register.hpp"
#include "type.hpp"
#include "log.hpp"

#include <memory>
#include <format>

struct RV32_Instruction;
struct RV32_U;
struct RV32_J;
struct RV32_I;
struct RV32_B;
struct RV32_S;
struct RV32_R;
struct RV32_ECALL;

struct Visitor {
  virtual void Visit(RV32_U *s) = 0;
  virtual void Visit(RV32_J *s) = 0;
  virtual void Visit(RV32_I *s) = 0;
  virtual void Visit(RV32_B *s) = 0;
  virtual void Visit(RV32_S *s) = 0;
  virtual void Visit(RV32_R *s) = 0;
  virtual void Visit(RV32_ECALL *s) {};
};

struct RV32_Instruction {
  virtual void Parse(word cmd) = 0;
  virtual void Exec(Visitor &v) = 0;
  virtual ~RV32_Instruction() = default;
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
  ~RV32_U() override = default;
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
  ~RV32_J() override = default;
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
  ~RV32_I() override = default;
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
  ~RV32_B() override = default;
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
  ~RV32_S() override = default;
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
  ~RV32_R() override = default;
};

struct RV32_ECALL : public RV32_Instruction {
  int type;
  void Parse(word cmd) override {
    if (cmd == 0x73)
      type = 0;
    else if (cmd == 0x100073)
      type = 1;
    else
      type = -1;
  }
  void Exec(Visitor &v) override { v.Visit(this); }
  ~RV32_ECALL() override = default;
};

std::unique_ptr<RV32_Instruction> dispatch(word cmd) {
  std::unique_ptr<RV32_Instruction> ins;
  byte opcode = cmd & Mask(7);
  if (opcode == 0b0110111 || opcode == 0b0010111)
    ins = std::unique_ptr<RV32_Instruction>(new RV32_U);
  else if (opcode == 0b1101111)
    ins = std::unique_ptr<RV32_Instruction>(new RV32_J);
  else if (opcode == 0b1100111 or opcode == 0b0000011 or opcode == 0b0010011)
    ins = std::unique_ptr<RV32_Instruction>(new RV32_I);
  else if (opcode == 0b1100011)
    ins = std::unique_ptr<RV32_Instruction>(new RV32_B);
  else if (opcode == 0b0100011)
    ins = std::unique_ptr<RV32_Instruction>(new RV32_S);
  else if (opcode == 0b0110011)
    ins = std::unique_ptr<RV32_Instruction>(new RV32_R);
  else if (cmd == 0x73 or cmd == 0x100073)
    ins = std::unique_ptr<RV32_Instruction>(new RV32_ECALL);
  else {
    log.Error(std::format("Unparseable opcode {} cmd {:X}", opcode, cmd));
    return nullptr;
  }
  return ins;
}

#endif
