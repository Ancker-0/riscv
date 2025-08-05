li a1, 0x10
li a2, 0x0
ecall
ecall
ebreak
addi a1, a1, 1
bne a1, a2, -16
