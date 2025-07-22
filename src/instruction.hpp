#ifndef __INSTRUCTION_HPP__
#define __INSTRUCTION_HPP__

class RV32_Instruction {
  virtual void Execute(word cmd);
};

class RV32_U : public RV32_Instruction {
  RegIdx rd;
  word imm;

public:
};

#endif
