#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

#define RISCV_ENABLE64 0

#if RISCV_ENABLE64
 #define RV64I
#else
 #define RV32I
#endif

#endif
