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
  const byte store_series_opc = 0b0100011;
  const byte arithmetic_imm_series_opc = 0b0010011;
  const byte arithmetic_series_opc = 0b0110011;
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
  void UpdRoB(int robid, reg_t val);

  void RunALU();
  void RunRS();
  void RunRoB();
  void RunDecoder();
  void RunLSB();

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

  CPU(Mem *mem_, Decoder *dec_, RS *rs_, RF *rf_, RoB *rob_, ALU *alu_, LSB *lsb_);

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
    log.Info(std::format("Step to {:X}", PC));

    RunRS();
    RunRoB();
    RunDecoder();
    RunALU();
    RunLSB();

    PC = NPC;

    for (auto &p : conn)
      p->copy();
  }

  virtual void Visit(RV32_U *s) override;

  void FillRSj(int rsid, RegIdx r);

  void FillRSk(int rsid, RegIdx r);

  virtual void Visit(RV32_J *s) override;
  virtual void Visit(RV32_I *s) override;
  virtual void Visit(RV32_B *s) override {};
  virtual void Visit(RV32_S *s) override;
  virtual void Visit(RV32_R *s) override;
  virtual void Visit(RV32_ECALL *s) override;
};


CPU::CPU(Mem *mem_, Decoder *dec_, RS *rs_, RF *rf_, RoB *rob_, ALU *alu_, LSB *lsb_) : mem(mem_), dec(dec_), rs(rs_),
  rf(rf_), rob(rob_), alu(alu_), lsb(lsb_) {
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

  Connect(lsb->qold.tl, lsb->q.tl);
  Connect(lsb->qold.hd, lsb->q.hd);
  for (int i = 0; i < LSB::LSB_size; ++i)
    Connect(lsb->qold.q[i], lsb->q.q[i]);

  static int always_zero = 0;
  Connect(rs->lsb_used, always_zero);
}

inline void CPU::UpdRoB(int robid, reg_t val) {
  auto &q = rob->q.q[robid];
  assert(q.get().type == RoB::robreg);
  q.get().val = val;
  q.get().state = RoB::robfinish;
  q.set();
  log.Debug(std::format("rob {} released", robid));

  for (int i = 0; i < RS_size; ++i)
    if (auto &rsi = rs->q[i].get(), &rso = rs->qold[i].get(); rso.busy and rso.state == RS::rsprepare) {
      if (rso.Qj == robid)
        rsi.Qj = RS::FREEID, rsi.Vj = val;
      if (rso.Qk == robid)
        rsi.Qk = RS::FREEID, rsi.Vk = val;
      log.Debug(std::format("checking RS {} with Qj = {}, Qk = {}", i, rso.Qj, rso.Qk));
      if (rsi.Qj == RS::FREEID and rsi.Qk == RS::FREEID) {
        assert(rsi.state == RS::rsprepare or rsi.state == RS::rsready);
        log.Debug(std::format("Gotcha RS {} val {:X}", i, val));
        rsi.state = RS::rsready;
      }
      rs->q[i].set();
    }
}

inline void CPU::RunLSB() {
  if (not lsb->qold.empty() and lsb->qold.front().get().state == LSB::Ready) {
    auto &now = lsb->q.front().get();
    now = lsb->qold.front().get();
    if (now.type == LSB::Load) {
      reg_t val = 0;
      if (now.store_length == 0) {
        if (now.sign)
          val = (sbyte) (*mem)[now.addr];
        else
          val = (*mem)[now.addr];
      } else if (now.store_length == 1) {
        if (now.sign)
          val = (*mem)[now.addr] | (sreg_t)(sbyte)(*mem)[now.addr + 1] << 8;
        else
          val = (*mem)[now.addr] | (reg_t)(*mem)[now.addr + 1] << 8;
      } else if (now.store_length == 2) {
        val = (*mem)[now.addr] | (reg_t) (*mem)[now.addr + 1] << 8 | (reg_t) (*mem)[now.addr + 2] << 16 | (reg_t) (*mem)[now.addr + 3] << 24;
      } else
        log.Error("Unexpected error: ooL2o");
      // lsb->q.front().set();
      UpdRoB(now.robid, val);
      // lsb->q.hd++;
      lsb->q.inc(lsb->q.hd);
    } else if (now.type == LSB::Store) {
      rob->q.q[now.robid].get().state = RoB::robfinish;
      rob->q.q[now.robid].set();
      now.state = LSB::Executing;
      lsb->q.front().set();
    }
  } else if (not lsb->qold.empty() and lsb->qold.front().get().state == LSB::Done) {
    // lsb->q.hd++;
    lsb->q.inc(lsb->q.hd);
  }
}

// TODO: Let ALU directly know robid
inline void CPU::ALUExec(const ALU::Unit &now) {
  if (now.type == ALU::ADD) {
    reg_t result = now.a + now.b;
    UpdRoB(now.robid, result);
  } else if (now.type == ALU::XOR) {
    reg_t result = now.a ^ now.b;
    UpdRoB(now.robid, result);
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
    log.Debug(std::format("Issuing ALU::ADD {:x} {:x} by alu {} updating rob {} (aka {})", now.Vj, now.Vk, aluid, now.robid, rs->qold[rsid].get().robid));
    unit.set(ALU::Unit{ true, now.robid, ALU::ADD, now.Vj, now.Vk });
    now.state = RS::rsexecuting;
    now.busy = false;
    rs->q[rsid].set(now);
  } else if (now.opcode == load_series_opc) {
    int lsbid = now.robid;
    log.Debug(std::format("RS ready for load {}", lsbid));
    lsb->q.q[lsbid].get().addr = now.Vj + now.Vk;
    lsb->q.q[lsbid].get().state = LSB::Ready;
    lsb->q.q[lsbid].set();
    lsb->counter = LSB::LSB_DELAY;
    now.state = RS::rsexecuting;
    now.busy = false;
    rs->q[rsid].set(now);
  } else if (now.opcode == store_series_opc) {
    int lsbid = now.robid;
    reg_t addr = now.Vj + now.A;
    now.busy = false;
    lsb->q.q[lsbid].get().addr = addr;
    lsb->q.q[lsbid].get().store_value = now.Vk;
    lsb->q.q[lsbid].get().state = LSB::Ready;
    lsb->q.q[lsbid].set();
  } else if (now.opcode == arithmetic_imm_series_opc) {
    int aluid = alu->validId(start);
    if (aluid == -1)
      return;
    start = aluid + 1;
    auto &unit = alu->as[aluid].now;
    unit.get().busy = true;
    unit.set();
    if (now.fn3 == 0b000) {
      unit.set(ALU::Unit{ true, now.robid, ALU::ADD, now.Vj, now.Vk });
      now.busy = false;
    } else
      log.Error("Sorry, not implemented! ieVo8");
  } else if (now.opcode == arithmetic_series_opc) {
    int aluid = alu->validId(start);
    if (aluid == -1)
      return;
    start = aluid + 1;
    auto &unit = alu->as[aluid].now;
    unit.get().busy = true;
    unit.set();
    if (now.fn3 == 0b000) {
      if (now.fn7 == 0b0000000)
        unit.set(ALU::Unit{ true, now.robid, ALU::ADD, now.Vj, now.Vk });
      else
        unit.set(ALU::Unit{ true, now.robid, ALU::ADD, now.Vj, -now.Vk });
      now.busy = false;
    } else if (now.fn3 == 0b100) {
      unit.set(ALU::Unit{ true, now.robid, ALU::XOR, now.Vj, now.Vk });
      now.busy = false;
    } else
      log.Error("Sorry, not implemented! Tie8D");
  } else
    log.Error("Sorry, not implemented!");
}

void CPU::RunRS() {
  for (int i = 0; i < RS_size; ++i)
    if (auto &rsi = rs->q[i].get(), &rso = rs->qold[i].get(); rso.busy and rso.state == RS::rsprepare) {
      if (rso.Qj != RS::FREEID and rso.Qj != RS::INVALID_ID and rob->qold.q[rso.Qj].get().state == RoB::robfinish) {
        rsi.Vj = rob->qold.q[rso.Qj].get().val;
        rsi.Qj = RS::FREEID;
      }
      if (rso.Qk != RS::FREEID and rso.Qk != RS::INVALID_ID and rob->qold.q[rso.Qk].get().state == RoB::robfinish) {
        rsi.Vk = rob->qold.q[rso.Qk].get().val;
        rsi.Qk = RS::FREEID;
      }
      if (rsi.Qj == RS::FREEID and rsi.Qk == RS::FREEID) {
        assert(rsi.state == RS::rsprepare or rsi.state == RS::rsready);
        log.Debug(std::format("Gotcha (rs) RS {} value {:x} {:x}", i, rsi.Vj, rsi.Vk));
        rsi.state = RS::rsready;
      }
      rs->q[i].set();
    }

  int start = 0;
  for (int i = 0; i < RS_size; ++i)
    if (auto &now = rs->qold[i].get(); now.busy and now.state == RS::rsready)
      RSExec(now, i, start);
}

void CPU::RunRoB() {
  if (rob->qold.empty())
    return;
  RoB::RoBc now = rob->qold.front().get();
  if (now.state != RoB::robfinish)
    return;
  int robid = rob->qold.hd;
  log.Debug(std::format("Commiting PC {:X} robid {}", now.PC, robid));
  if (now.type == RoB::robreg) {
    // rob->q.hd++;
    rob->q.inc(rob->q.hd);
    if (auto rfi = rf->pre[now.dest.p].get(); rfi.dependRoB == robid and rfi.busy == true) {
      rf->UpdVal(now.dest, now.val);
    } else {
      // printf("%d %d %d\n", rfi.dependRoB, robid, rfi.busy);
    }
  } else if (now.type == RoB::robmem) {
    int lsbid = now.val;
    auto &ls = lsb->qold.q[lsbid].get();
    if (ls.state != LSB::Executing)
      return;
    // rob->q.hd++;
    rob->q.inc(rob->q.hd);
    log.Debug(std::format("Commit store lsbid {}", lsbid));
    (*mem)[ls.addr] = ls.store_value;
    if (ls.store_length >= 1)
      (*mem)[ls.addr + 1] = ls.store_value >> 8;
    if (ls.store_length >= 2) {
      (*mem)[ls.addr + 2] = ls.store_value >> 16;
      (*mem)[ls.addr + 3] = ls.store_value >> 24;
    }
    lsb->q.q[lsbid].get().state = LSB::Done;
    lsb->q.q[lsbid].set();
  } else if (now.type == RoB::robend) {
    finish = true;
    printf("%d\n", rf->now[10].get().val & 0xFF);
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
    int robid = rob->q.tl; rob->q.inc(rob->q.tl);
    rob->q.q[robid].set(RoB::RoBc{
      RoB::robend,
      RegIdx(0), (reg_t)-1, (byte)-1, (word)-1,
      PC, RoB::robfinish,
    });
    NPC = PC;
    // finish = true;
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
    int robid = rob->q.tl; rob->q.inc(rob->q.tl);
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
    int robid = rob->q.tl; rob->q.inc(rob->q.tl);
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

void CPU::FillRSj(int rsid, RegIdx r) {
  if (rf->pre[r.p].get().busy)
    rs->q[rsid].get().Qj = rf->pre[r.p].get().dependRoB;
  else {
    rs->q[rsid].get().Qj = RS::FREEID;
    rs->q[rsid].get().Vj = rf->pre[r.p].get().val;
  }
  rs->q[rsid].set();
}

void CPU::FillRSk(int rsid, RegIdx r) {
  if (rf->pre[r.p].get().busy)
    rs->q[rsid].get().Qk = rf->pre[r.p].get().dependRoB;
  else {
    rs->q[rsid].get().Qk = RS::FREEID;
    rs->q[rsid].get().Vk = rf->pre[r.p].get().val;
  }
  rs->q[rsid].set();
}

inline void CPU::Visit(RV32_J *s) {
  if (s->opcode == jal_opc) {
    if (rob->qold.full()) {
      NPC = PC;
      return;
    }
    int robid = rob->q.tl; rob->q.inc(rob->q.tl);
    rob->q.q[robid].set(RoB::RoBc{
      RoB::robreg,
      s->rd, (reg_t)-1,
      (byte)-1, (word)(PC + 4),
      PC,
      RoB::robfinish,
    });
    rf->Upd(s->rd, robid);
    NPC = PC + s->imm;
  } else
    log.Error("Sorry, not implemented! Eey8r");
}

inline void CPU::Visit(RV32_I *s) {
  if (rob->q.full()) {
    NPC = PC;
    return;
  }
  if (s->opcode == jalr_opc and s->fn3 == 0b000) {
    if (rf->pre[s->rs1.p].get().busy or rob->qold.full()) {
      NPC = PC;
      return;
    }
    NPC = (rf->pre[s->rs1.p].get().val + s->imm) & ~1;
    int robid = rob->q.tl;
    rob->q.q[robid].set(RoB::RoBc{
      RoB::robreg,
      s->rd, (reg_t)-1,
      (byte)-1, (word)(PC + 4),
      PC,
      RoB::robfinish
    });
    rf->Upd(s->rd, robid);
  } else if (s->opcode == 0b0000011) {
    if (lsb->qold.full()) {
      NPC = PC;
      return;
    }
    int lsbid = lsb->q.tl; lsb->q.inc(lsb->q.tl);
    s->Sign();
    int robid = rob->q.tl; rob->q.inc(rob->q.tl);
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
      lsbid, 0,
      0, s->imm,
      RS::INVALID_ID, RS::FREEID,
      0,
      RS::rsprepare
    });
    if (auto &rfc = rf->pre[s->rs1.p].get(); rfc.busy) {
      rs->q[validRS].get().Qj = rf->pre[s->rs1.p].get().dependRoB;
      log.Debug(std::format("PC {:X} Waiting for {}", PC, rf->pre[s->rs1.p].get().dependRoB));
    } else {
      log.Debug("Already ready!");
      rs->q[validRS].get().state = RS::rsready;
      rs->q[validRS].get().Qj = RS::FREEID;
      rs->q[validRS].get().Vj = rfc.val;
    }
    int len = s->fn3 & 0b011;
    bool sign = !(s->fn3 & 0b100);
    lsb->q.q[lsbid].set(LSB::Data{
      LSB::Load,
      true,
      robid,
      (reg_t)-1,
      (byte)len, sign,
      LSB::Prepare,
      (reg_t)-1,
    });
    rf->Upd(s->rd, robid);
  } else if (s->opcode == 0b0010011) {
    int robid = rob->q.tl; rob->q.inc(rob->q.tl);
    rob->q.q[robid].set(RoB::RoBc{
      RoB::robreg,
      s->rd, (reg_t)-1,
      (byte)-1,
      (word)-1, PC,
      RoB::robexecuting
    });
    rs->q[validRS].set(RS::RS_shot{
      true, s->opcode, s->fn3, -1,
      robid, (word)-1,
      (word)-1, s->imm,
      RS::INVALID_ID, RS::FREEID,
      (reg_t)-1,
      RS::rsprepare
    });
    FillRSj(validRS, s->rs1);
    rf->Upd(s->rd, robid);

//     if (s->fn3 == 0b000)
//       s->Sign(), regs[s->rd] = regs[s->rs1] + s->imm;
//     else if (s->fn3 == 0b101 and (s->imm >> 6) == 0b000000) {
//       regs[s->rd] = regs[s->rs1] >> (s->imm & Mask(6));
// #ifdef RV32I
//       if (s->imm >> 5 & 1)
//         log.Error(std::format("Cannot shift more than 31 pos, opcode = {}", s->opcode));
// #endif
//     } else if (s->fn3 == 0b101 and (s->imm >> 6) == 0b010000) {
//       regs[s->rd] = (sreg_t) regs[s->rs1] >> (s->imm & Mask(6));
// #ifdef RV32I
//       if (s->imm >> 5 & 1)
//         log.Error(std::format("Cannot shift more than 31 pos, opcode = {}", s->opcode));
// #endif
//     } else if (s->fn3 == 0b001 and (s->imm >> 6) == 0b000000) {
//       regs[s->rd] = regs[s->rs1] << (s->imm & Mask(6));
// #ifdef RV32I
//       if (s->imm >> 5 & 1)
//         log.Error(std::format("Cannot shift more than 31 pos, opcode = {}", s->opcode));
// #endif
//     } else if (s->fn3 == 0b100)
//       s->Sign(), regs[s->rd] = regs[s->rs1] ^ s->imm;
//     else if (s->fn3 == 0b111)
//       s->Sign(), regs[s->rd] = regs[s->rs1] & s->imm;
//     else if (s->fn3 == 0b010) {
//       s->Sign(), regs[s->rd] = (regs[s->rs1] < s->imm) ? 1 : 0;
//     } else
//       log.Error(std::format("Unrecognized I code {} fn3 {}", s->opcode, s->fn3));
  } else
    log.Error(std::format("Unrecognized I code {} fn3 {}", s->opcode, s->fn3));
}

inline void CPU::Visit(RV32_S *s) {
  if (s->opcode == store_series_opc) {
    if (lsb->qold.full()) {
      NPC = PC;
      return;
    }
    int lsbid = lsb->q.tl; lsb->q.inc(lsb->q.tl);
    s->Sign();
    int robid = rob->q.tl; rob->q.inc(rob->q.tl);
    int len = s->fn3;
    assert(s->fn3 <= 2);
    rob->q.q[robid].set(RoB::RoBc{
      RoB::robmem,
      RegIdx(0), (reg_t)-1,
      (byte)len, (reg_t)lsbid,
      PC,
      RoB::robexecuting
    });
    rs->q[validRS].set(RS::RS_shot{
      true,
      s->opcode, s->fn3, -1,
      lsbid, 0,
      0, 0,
      RS::INVALID_ID, RS::INVALID_ID,
      s->imm,
      RS::rsprepare,
    });
    if (auto &rfc = rf->pre[s->rs1.p].get(); rfc.busy) {
      rs->q[validRS].get().Qj = rf->pre[s->rs1.p].get().dependRoB;
    } else {
      rs->q[validRS].get().Qj = RS::FREEID;
      rs->q[validRS].get().Vj = rfc.val;
    }
    if (auto &rfc = rf->pre[s->rs2.p].get(); rfc.busy) {
      rs->q[validRS].get().Qk = rf->pre[s->rs2.p].get().dependRoB;
    } else {
      rs->q[validRS].get().Qk = RS::FREEID;
      rs->q[validRS].get().Vk = rfc.val;
    }
    lsb->q.q[lsbid].set(LSB::Data{
      LSB::Store,
      true,
      robid,
      (reg_t)-1,
      (byte)len, false,
      LSB::Prepare,
      (reg_t)-1,
    });
  } else
    assert(false);
}

inline void CPU::Visit(RV32_R *s) {
  if (s->opcode == arithmetic_series_opc) {
    if (rob->qold.full()) {
      NPC = PC;
      return;
    }
    int robid = rob->q.tl; rob->q.inc(rob->q.tl);
    rob->q.q[robid].set(RoB::RoBc{
      RoB::robreg,
      s->rd, (reg_t)-1,
      (byte)-1, (reg_t)-1,
      PC,
      RoB::robexecuting
    });
    rs->q[validRS].set(RS::RS_shot{
      true, s->opcode, s->fn3, s->fn7,
      robid,
      (word)-1, (word)-1, (word)-1,
      RS::INVALID_ID, RS::INVALID_ID,
      (reg_t)-1, RS::rsprepare,
    });
    FillRSj(validRS, s->rs1);
    FillRSk(validRS, s->rs2);
    rf->Upd(s->rd, robid);
  } else
    assert(false);
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
