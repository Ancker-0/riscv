#include "config.hpp"
#include "type.hpp"
#include "cpu.hpp"
#include "memory.hpp"
#include "riscv.hpp"
#include "instruction.hpp"
#include "toy.hpp"

#include <cstring>
#include <format>
#include <iostream>
#include <memory>
#include <cstdio>
// #include <bits/stdc++.h>

Mem Load(FILE *f) {
  static byte map[128];
  for (int i = 0; i < 10; ++i)
    map['0' + i] = (byte)i;
  for (int i = 0; i < 6; ++i)
    map['A' + i] = (byte)(10 + i);

  reg_t where = 0;
  Mem res;
  int ch;
  while ((ch = getc(f)) != EOF) {
    if (ch == '@') {
      reg_t pos = 0;
#ifndef RV32I
 #error "unimplemented"
#else
      for (int i = 0; i < 8; ++i)
        pos = (pos << 4) | map[getc(f)];
#endif
      log.Info(std::format("Jump to {:X}", pos));
      where = pos;
    } else if (('0' <= ch and ch <= '9') or ('A' <= ch and ch <= 'F')) {
      int nxt = getc(f);
      // log.Debug(std::format("Reading {:X} as {:#X}", where, map[ch] << 4 | map[nxt]));
      res[where++] = map[ch] << 4 | map[nxt];
    }
  }
  return res;
}

int main(int argc, char *argv[]) {
  // printf("%d %d\n", (sreg_t)(sbyte)0xFF, (sreg_t)(byte)0xFF);
  char *filename;
  std::unique_ptr<char[]> fnptr;
  FILE *input;
  if (argc == 1) {
    // fnptr = std::unique_ptr<char[]>(new char[1024]);
    // filename = fnptr.get();
    // scanf("%s", filename);
    // filename = "../asm/many.data";
    // input = fopen(filename, "r");
    // strcpy(filename, "/dev/stdin");
    input = stdin;
    // input = fopen("../asm/simple.data", "r");
  } else if (argc == 2) {
    filename = argv[1];
    input = fopen(filename, "r");
  } else {
    fprintf(stderr, "Unexpected argc!\n");
    return EXIT_FAILURE;
  }

  // RV_toy::Run(std::move(Load(input)));
  RV::Run(std::move(Load(input)));
  fclose(input);

  std::cerr << "Hello world!" << std::endl;
  return 0;
}
