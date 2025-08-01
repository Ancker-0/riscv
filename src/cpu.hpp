#ifndef __CPU_HPP__
#define __CPU_HPP__

#include "type.hpp"
#include "RS.hpp"
#include "alu.hpp"
#include "memory.hpp"
#include "log.hpp"
#include "decoder.hpp"
#include "register.hpp"
#include "lsb.hpp"
#include "instruction.hpp"
#include <vector>
#include <memory>

class CPU : public Visitor {
  const byte lui_opc   = 0b0110111;
  const byte auipc_opc = 0b0010111;
  const byte load_series_opc = 0b0000011;
  const byte jal_opc   = 0b1101111;
  const byte jalr_opc  = 0b1100111;

  struct copy_pipe {
    virtual void copy() = 0;
    virtual ~copy_pipe() = default;
  };

  class DecoderVisitor : Visitor {
  };

  template<typename T>
  struct copy_pipe_impl : public copy_pipe {
    Proxy<T> *dest, *src;
    copy_pipe_impl(Proxy<T> *dest_, Proxy<T> *src_) : dest(dest_), src(src_) {}
    void copy() override {
      src->give(*dest);
    }
  };

  template<typename T>
  struct copy_primitive : public copy_pipe {
    T *dest, *src;
    copy_primitive(T *dest_, T *src_) : dest(dest_), src(src_) {}
    void copy() override {
      *dest = *src;
    }
  };

  std::vector<std::unique_ptr<copy_pipe>> conn;
  bool finish;

  void RSExec(RS::RS_shot &now, int rsid, int &start);
  void ALUExec(const ALU::Unit & now);

  void RunALU();
  void RunRS();
  void RunRoB();
  void RunDecoder();

public:
  Mem *mem;
  // Reg *reg;
  Decoder *dec;
  RS *rs;
  RF *rf;
  RoB *rob;
  ALU *alu;
  LSB *lsb;

  reg_t PC, NPC;
  int clk;

  byte validRS;

  template <typename T>
  void Connect(Proxy<T> &dest, Proxy<T> &src) {  // chronic connect
    conn.push_back(std::make_unique<copy_pipe_impl<T>>(&dest, &src));
  }
  void Connect(int &dest, int &src) {  // chronic connect
    conn.push_back(std::make_unique<copy_primitive<int>>(&dest, &src));
  }

  CPU(Mem *mem_, Decoder *dec_, RS *rs_, RF *rf_, RoB *rob_, ALU *alu_, LSB *lsb_) : mem(mem_), dec(dec_), rs(rs_), rf(rf_), rob(rob_), alu(alu_), lsb(lsb_) {
    finish = false;
    PC = 0;
    clk = 0;
    for (int i = 0; i < RS_size; ++i)
      Connect(rs->qold[i], rs->q[i]);

    Connect(rob->qold.tl, rob->q.tl);
    Connect(rob->qold.hd, rob->q.hd);
    for (int i = 0; i < RoB_size; ++i)
      Connect(rob->qold.q[i], rob->q.q[i]);

    for (int i = 0; i < 32; ++i)
      Connect(rf->pre[i], rf->now[i]);
    for (int i = 0; i < ALU_size; ++i)
      Connect(alu->as[i].pre, alu->as[i].now);
  }

  // It remains a technical problem to logically connect the stuffs.

  void Run() {
    printf("Hello CPU\n");
    while (not finish)
      Step();
  }

  void Step() {
    ++clk;
    // byte identifier = mem[pc];
    // if ((identifier & 0x3) != 0x3) {
    //   log.Warn("C extension is not supported");
    //   pc += 2;
    //   return;
    // }
    // word cmd = mem[pc] | ((word)mem[pc + 1] << 8) | ((word)mem[pc + 2] << 16) | ((word)mem[pc + 3] << 24);

    NPC = PC + 4;

    RunRS();
    RunRoB();
    RunDecoder();
    RunALU();

    PC = NPC;

    for (auto &p : conn)
      p->copy();
  }

  virtual void Visit(RV32_U *s) override;
  virtual void Visit(RV32_J *s) override {};
  virtual void Visit(RV32_I *s) override;
  virtual void Visit(RV32_B *s) override {};
  virtual void Visit(RV32_S *s) override {};
  virtual void Visit(RV32_R *s) override {};
  virtual void Visit(RV32_ECALL *s) override;
};

inline void CPU::ALUExec(const ALU::Unit &now) {
  if (now.type == ALU::ADD) {
    int result = now.a + now.b;
    rs->q[now.rsid].get().state = RS::rsdone;
    rs->q[now.rsid].set();
    int robid = rs->qold[now.rsid].get().robid;
    rob->q.q[robid].get().val = result;
    rob->q.q[robid].get().state = RoB::robfinish;
    rob->q.q[robid].set();
    log.Debug(std::format("ALU ADD: {:x} + {:x} = {:x}", now.a, now.b, result));
  } else
    log.Debug(std::format("ALU: unrecognized type {}", (int)now.type));
}

inline void CPU::RunALU() {
  for (int i = 0; i < ALU_size; ++i) {
    auto &now = alu->as[i].pre.get();
    if (now.busy) {
      alu->as[i].now.get().busy = false;
      alu->as[i].now.set();
      ALUExec(now);
    }
  }
}

void CPU::RSExec(RS::RS_shot &now, int rsid, int &start) {
  if (now.opcode == auipc_opc) {
    int aluid = alu->validId(start);
    if (aluid == -1)
      return;
    start = aluid + 1;
    auto &unit = alu->as[aluid].now;
    unit.get().busy = true;
    unit.set();
    log.Debug(std::format("Issuing ALU::ADD {:x} {:x} by alu {}", now.Vj, now.Vk, aluid));
    unit.set(ALU::Unit{ true, rsid, ALU::ADD, now.Vj, now.Vk });
    now.state = RS::rsexecuting;
    rs->q[rsid].set(now);
  } else if (now.opcode == load_series_opc) {
    if (lsb->s.now.get().busy)
      return;
    int len = now.fn3 == 0b000 or now.fn3 == 0b100 ? 1 : now.fn3 == 0b001 or now.fn3 == 0b101 ? 2 : now.fn3 == 0b010 ? 4 : -1;
    if (len == -1)
      log.Error("Unexpected error: Eisi6");
    lsb->s.now.set(LSB::Data{ true, now.robid, now.Vj + now.Vk, });
  }
}

void CPU::RunRS() {
  int start = 0;
  for (int i = 0; i < RS_size; ++i)
    if (auto &now = rs->qold[i].get(); now.busy and now.state == RS::rsready)
      RSExec(now, i, start);
}

void CPU::RunRoB() {
  if (rob->qold.empty())
    return;
  RoB::RoBc now = rob->qold.front().get();
  log.Debug(std::format("Checking PC {:X}", now.PC));
  if (now.state != RoB::robfinish)
    return;
  int robid = rob->qold.hd;
  log.Debug(std::format("Commiting PC {:X} robid {}", now.PC, robid));
  rob->q.hd++;
  if (now.type == RoB::robreg) {
    if (auto rfi = rf->pre[now.dest.p].get(); rfi.dependRoB == robid and rfi.busy == true) {
      rf->UpdVal(now.dest, now.val);
    } else {
      printf("%d %d %d\n", rfi.dependRoB, robid, rfi.busy);
    }
  }
}

void CPU::RunDecoder() {
  validRS = rs->emptyslot();
  if (validRS == -1 or rob->q.full()) {
    NPC = PC;
    return;
  }
  word cmd = (*mem)[PC] | (*mem)[PC + 1] << 8 | (*mem)[PC + 2] << 16 | (*mem)[PC + 3] << 24;
  if (cmd == 0x0ff00513) {
    finish = true;
    return;
  }
  std::unique_ptr<RV32_Instruction> ins = dispatch(cmd);
  if (ins == nullptr) {
    log.Error(std::format("Fail to dispatch cmd {:X}", cmd));
    return;
  }
  ins->Parse(cmd);
  ins->Exec(*this);
}

void CPU::Visit(RV32_U *s) {
  if (rob->q.full()) {
    NPC = PC;
    return;
  }
  if (s->opcode == lui_opc) {
    // (*reg)[s->rd] = s->imm;
    int robid = rob->q.tl++;
    rob->q.q[robid].set(RoB::RoBc{
      RoB::robreg,
      s->rd,
      (reg_t)-1, (byte)-1,
      s->imm,
      PC,
      RoB::robfinish,
    });
    rf->Upd(s->rd, robid);
  } else if (s->opcode == auipc_opc) {
    // (*reg)[s->rd] = PC + s->imm;
    int robid = rob->q.tl++;
    rob->q.q[robid].set(RoB::RoBc{
      RoB::robreg,
      s->rd,
      (reg_t)-1, (byte)-1,
      0,
      PC,
      RoB::robexecuting
    });
    rs->q[validRS].set(RS::RS_shot{
      true,
      s->opcode, -1, -1,
      robid, 0,
      PC, s->imm,
      RS::FREEID, RS::FREEID,
      0,
      RS::rsready
    });
    rf->Upd(s->rd, robid);
  } else
    log.Error(std::format("Unrecognized U code {}", s->opcode));
}

inline void CPU::Visit(RV32_I *s) {
  if (rob->q.full()) {
    NPC = PC;
    return;
  }
  if (s->opcode == jalr_opc and s->fn3 == 0b000) {
    // s->Sign();
    // reg_t tmp = PC + 4;
    // NPC = (regs[s->rs1] + s->imm) & ~1;
    // regs[s->rd] = tmp;
  } else if (s->opcode == 0b0000011) {
    s->Sign();
    if (s->fn3 == 0b000) {
      int robid = rob->q.tl++;
      rob->q.q[robid].set(RoB::RoBc{
        RoB::robreg,
        s->rd,
        (reg_t) -1, (byte) -1,
        0,
        PC,
        RoB::robexecuting
      });
      rs->q[validRS].set(RS::RS_shot{
        true,
        s->opcode, s->fn3, -1,
        robid, 0,
        0, s->imm,
        RS::INVALID_ID, RS::FREEID,
        0,
        RS::rsprepare
      });
      if (auto &rfc = rf->pre[s->rs1.p].get(); rfc.busy)
        rs->q[validRS].get().Qj = rf->pre[s->rs1.p].get().dependRoB;
      else {
        rs->q[validRS].get().state = RS::rsready;
        rs->q[validRS].get().Qj = RS::FREEID;
        rs->q[validRS].get().Vj = rfc.val;
      }
      rf->Upd(s->rd, robid);
    }
    // if (s->fn3 == 0b000)
    //   regs[s->rd] = (sbyte) (*mem)[regs[s->rs1] + s->imm];
    // else if (s->fn3 == 0b001)
    //   regs[s->rd] = (*mem)[regs[s->rs1] + s->imm] | (sreg_t) (sbyte) (*mem)[regs[s->rs1] + s->imm + 1] << 8;
    // else if (s->fn3 == 0b010) {
    //   regs[s->rd] = (*mem)[regs[s->rs1] + s->imm] | (reg_t) (*mem)[regs[s->rs1] + s->imm + 1] << 8 | (reg_t) (*mem)[
    //                   regs[s->rs1] + s->imm + 2] << 16 | (sreg_t) (sbyte) (*mem)[regs[s->rs1] + s->imm + 3] << 24;
    //   // log.Debug(std::format("Load {:02X} {:02X} {:02X} {:02X} as {}", (*mem)[regs[s->rs1] + s->imm], (*mem)[regs[s->rs1] + s->imm + 1], (*mem)[regs[s->rs1] + s->imm + 2], (*mem)[regs[s->rs1] + s->imm + 3], regs[s->rd]));
    // } else if (s->fn3 == 0b100)
    //   regs[s->rd] = (*mem)[regs[s->rs1] + s->imm];
    // else if (s->fn3 == 0b101)
    //   regs[s->rd] = (*mem)[regs[s->rs1] + s->imm] | (reg_t) (*mem)[regs[s->rs1] + s->imm + 1] << 8;
    // else
    //   log.Error(std::format("Unrecognized I code {} fn3 {}", s->opcode, s->fn3));
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
      regs[s->rd] = (sreg_t) regs[s->rs1] >> (s->imm & Mask(6));
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
}

void CPU::Visit(RV32_ECALL *s) {
  if (s->type == 1) {
    printf("ebreak... args: ");
    for (int i = 0; i < 32; ++i)
      if (!rf->now[i].get().busy)
        printf("%X ", (*rf).now[i].get().val);
      else
        printf("[%d] ", (*rf).now[i].get().dependRoB);
    puts("");
  }
};

#endif
