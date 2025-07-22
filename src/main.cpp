#include "config.hpp"
#include "type.hpp"
#include "cpu.hpp"
#include "memory.hpp"
#include "riscv.hpp"

#include <format>
#include <iostream>
#include <memory>
#include <cstdio>
// #include <bits/stdc++.h>

Mem Load(char *filename) {
  static byte map[128];
  for (int i = 0; i < 10; ++i)
    map['0' + i] = (byte)i;
  for (int i = 0; i < 6; ++i)
    map['A' + i] = (byte)(10 + i);

  FILE *f = fopen(filename, "r");
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
      log.Info(std::format("Jump to {}", pos));
    } else if (('0' <= ch and ch <= '9') or ('A' <= ch and ch <= 'F')) {
      int nxt = getc(f);
      res[where++] = map[ch] << 4 | map[nxt];
    }
  }
  return res;
}

int main(int argc, char *argv[]) {
  char *filename;
  std::unique_ptr<char[]> fnptr;
  if (argc == 1) {
    fnptr = std::unique_ptr<char[]>(new char[1024]);
    filename = fnptr.get();
    scanf("%s", filename);
  } else if (argc == 2) {
    filename = argv[1];
  } else {
    fprintf(stderr, "Unexpected argc!\n");
    return EXIT_FAILURE;
  }

  RV::Init(Load(filename));

  std::cout << "Hello world!" << std::endl;
  return 0;
}
