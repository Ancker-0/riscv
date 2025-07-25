#ifndef __TOY_HPP__
#define __TOY_HPP__

#include "type.hpp"
#include "instruction.hpp"
#include "log.hpp"
#include "register.hpp"
#include "memory.hpp"

#include <cassert>
#include <format>
#include <memory>

namespace RV_toy {
  const byte lui_opc   = 0b0110111;
  const byte auipc_opc = 0b0010111;
  const byte jal_opc   = 0b1101111;
  const byte jalr_opc  = 0b1100111;

  struct Visitor : ::Visitor {
    std::unique_ptr<Mem> mem;
    reg_t PC, NPC;
    Reg regs;

    void Visit(RV32_U *s) override {
      if (s->opcode == lui_opc)
        regs[s->rd] = s->imm;
      else if (s->opcode == auipc_opc)
        regs[s->rd] = PC + s->imm;
      else
        log.Error(std::format("Unrecognized U code {}", s->opcode));
    };

    void Visit(RV32_J *s) override {
      if (s->opcode == jal_opc) {
        regs[s->rd] = PC + 4;
        NPC = PC + s->imm;
      } else
        log.Error(std::format("Unrecognized J code {}", s->opcode));
    };


    void Visit(RV32_I *s) override {
      if (s->opcode == jalr_opc and s->fn3 == 0b000) {
        s->Sign();
        reg_t tmp = PC + 4;
        NPC = (regs[s->rs1] + s->imm) & ~1;
        regs[s->rd] = tmp;
      } else if (s->opcode == 0b0000011) {
        s->Sign();
        if (s->fn3 == 0b000)
          regs[s->rd] = (sbyte)(*mem)[regs[s->rs1] + s->imm];
        else if (s->fn3 == 0b001)
          regs[s->rd] = (*mem)[regs[s->rs1] + s->imm] | (sreg_t)(sbyte)(*mem)[regs[s->rs1] + s->imm + 1] << 8;
        else if (s->fn3 == 0b010) {
          regs[s->rd] = (*mem)[regs[s->rs1] + s->imm] | (reg_t)(*mem)[regs[s->rs1] + s->imm + 1] << 8 | (reg_t)(*mem)[regs[s->rs1] + s->imm + 2] << 16 | (sreg_t)(sbyte)(*mem)[regs[s->rs1] + s->imm + 3] << 24;
          // log.Debug(std::format("Load {:02X} {:02X} {:02X} {:02X} as {}", (*mem)[regs[s->rs1] + s->imm], (*mem)[regs[s->rs1] + s->imm + 1], (*mem)[regs[s->rs1] + s->imm + 2], (*mem)[regs[s->rs1] + s->imm + 3], regs[s->rd]));
        } else if (s->fn3 == 0b100)
          regs[s->rd] = (*mem)[regs[s->rs1] + s->imm];
        else if (s->fn3 == 0b101)
          regs[s->rd] = (*mem)[regs[s->rs1] + s->imm] | (reg_t)(*mem)[regs[s->rs1] + s->imm + 1] << 8;
        else
          log.Error(std::format("Unrecognized I code {} fn3 {}", s->opcode, s->fn3));
        // log.Debug(std::format("loading {:X} with imm={:X}, val={}", regs[s->rs1] + s->imm, s->imm, regs[s->rd]));
      } else if (s->opcode == 0b0010011) {
        if (s->fn3 == 0b000)
          s->Sign(), regs[s->rd] = regs[s->rs1] + s->imm;
        else if (s->fn3 == 0b101 and (s->imm >> 6) == 0b000000) {
          regs[s->rd] = regs[s->rs1] >> (s->imm & Mask(6));
#ifdef RV32I
          if (s->imm >> 5 & 1)
            log.Error(std::format("Cannot shift more than 31 pos, opcode = {}", s->opcode));
#endif
        } else if (s->fn3 == 0b101 and (s->imm >> 6) == 0b010000) {
          regs[s->rd] = (sreg_t)regs[s->rs1] >> (s->imm & Mask(6));
#ifdef RV32I
          if (s->imm >> 5 & 1)
            log.Error(std::format("Cannot shift more than 31 pos, opcode = {}", s->opcode));
#endif
        } else if (s->fn3 == 0b001 and (s->imm >> 6) == 0b000000) {
          regs[s->rd] = regs[s->rs1] << (s->imm & Mask(6));
#ifdef RV32I
          if (s->imm >> 5 & 1)
            log.Error(std::format("Cannot shift more than 31 pos, opcode = {}", s->opcode));
#endif
        } else if (s->fn3 == 0b100)
          s->Sign(), regs[s->rd] = regs[s->rs1] ^ s->imm;
        else if (s->fn3 == 0b111)
          s->Sign(), regs[s->rd] = regs[s->rs1] & s->imm;
        else if (s->fn3 == 0b010) {
          s->Sign(), regs[s->rd] = (regs[s->rs1] < s->imm) ? 1 : 0;
        } else
          log.Error(std::format("Unrecognized I code {} fn3 {}", s->opcode, s->fn3));
      } else
        log.Error(std::format("Unrecognized I code {} fn3 {}", s->opcode, s->fn3));
    };

    void Visit(RV32_B *s) override {
      if (s->opcode == 0b1100011) {
        bool jump = false;
        reg_t a = regs[s->rs1], b = regs[s->rs2];
        if (s->fn3 == 0b000)
          jump = a == b;
        else if (s->fn3 == 0b001)
          jump = a != b;
        else if (s->fn3 == 0b100)
          jump = (sreg_t)a < (sreg_t) b;
        else if (s->fn3 == 0b101)
          jump = (sreg_t)a >= (sreg_t) b;
        else if (s->fn3 == 0b110)
          jump = a < b;
        else if (s->fn3 == 0b111)
          jump = a >= b;
        else
          log.Error(std::format("Unrecognized B code {} fn3 {}", s->opcode, s->fn3));
        s->Sign();
        if (jump)
          NPC = PC + s->imm;
      } else
        log.Error(std::format("Unrecognized B code {}", s->opcode));
    };

    void Visit(RV32_S *s) override {
      if (s->opcode == 0b0100011) {
        if (s->fn3 == 0b000)  // sb
          s->Sign(), (*mem)[regs[s->rs1] + s->imm] = regs[s->rs2] & Mask(8);
        else if (s->fn3 == 0b001)
          s->Sign(), (*mem)[regs[s->rs1] + s->imm] = regs[s->rs2] & Mask(8), (*mem)[regs[s->rs1] + s->imm + 1] = regs[s->rs2] >> 8 & Mask(8);
        else if (s->fn3 == 0b010) {
          s->Sign();
          (*mem)[regs[s->rs1] + s->imm] = regs[s->rs2] & Mask(8);
          (*mem)[regs[s->rs1] + s->imm + 1] = regs[s->rs2] >> 8 & Mask(8);
          (*mem)[regs[s->rs1] + s->imm + 2] = regs[s->rs2] >> 16 & Mask(8);
          (*mem)[regs[s->rs1] + s->imm + 3] = regs[s->rs2] >> 24 & Mask(8);
          if (regs[s->rs1] + s->imm == 0xA0000000)
            log.Debug(std::format("Output dec={} hex={:X}", regs[s->rs2], regs[s->rs2]));
          // log.Debug(std::format("storing {:X} with imm={:X} val={}", regs[s->rs1] + s->imm, s->imm, regs[s->rs2]));
        } else
          log.Error(std::format("Unrecognized S code {} fn3 {}", s->opcode, s->fn3));
      } else
        log.Error(std::format("Unrecognized S code {}", s->opcode));
    };

    void Visit(RV32_R *s) override {
      if (s->opcode == 0b0110011) {
        if (s->fn3 == 0b100)
          regs[s->rd] = regs[s->rs1] ^ regs[s->rs2];
        else if (s->fn3 == 0b001 and s->fn7 == 0b0000000) {
#ifdef RV32I
          regs[s->rd] = regs[s->rs1] << (regs[s->rs2] & Mask(5));
#elif defined(RV64I)
          regs[s->rd] = regs[s->rs1] << (regs[s->rs2] & Mask(6));
#endif
        } else if (s->fn3 == 0b000) {
          if (s->fn7 == 0b0000000)
            regs[s->rd] = regs[s->rs1] + regs[s->rs2];
          else if (s->fn7 == 0b0100000)
            regs[s->rd] = regs[s->rs1] - regs[s->rs2];
          else
            log.Error(std::format("Unrecognized R code {} fn3 {} fn7", s->opcode, s->fn3, s->fn7));
        } else if (s->fn3 == 0b110) {
          if (s->fn7 == 0b0000000)
            regs[s->rd] = regs[s->rs1] | regs[s->rs2];
          else
            log.Error(std::format("Unrecognized R code {} fn3 {} fn7", s->opcode, s->fn3, s->fn7));
        } else
          log.Error(std::format("Unrecognized R code {} fn3 {}", s->opcode, s->fn3));
      } else
        log.Error(std::format("Unrecognized R code {}", s->opcode));
    };

    void operator()(Mem &&mem_) {
      mem = std::make_unique<Mem>(mem_);
      PC = 0;

      while (true) {
        word cmd = (*mem)[PC] | (*mem)[PC + 1] << 8 | (*mem)[PC + 2] << 16 | (*mem)[PC + 3] << 24;
        // printf("PC = %X, cmd = %X\n", PC, cmd);
        if (cmd == 0x0ff00513) {
          log.Info("Returning");
          printf("%d\n", (int)regs[RegIdx(10)] & 0xFF);
          break;
        }
        byte opcode = cmd & Mask(7);
        if ((opcode & 0x3) != 0x3)
          log.Error(std::format("Bad opcode {} with cmd {}", opcode, cmd));
        assert((opcode & 0x3) == 0x3);
        std::unique_ptr<RV32_Instruction> ins;
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
        else {
          log.Error(std::format("Unparseable opcode {}", opcode));
          return;
        }
        NPC = PC + 4;
        ins->Parse(cmd);
        ins->Exec(*this);
        PC = NPC;
      }
    }
  } Run;
}

#endif
