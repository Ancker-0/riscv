li t0, 0          # 计数器
li t1, 3          # 目标值
li a0, 0          # 累加器
loop:
addi a0, a0, 10   # a0 += 10
addi t0, t0, 1    # 计数器++
#ebreak
blt t0, t1, loop  # 如果计数器 < 3，继续循环
# 循环3次：a0 = 10, 20, 30
li a0, 0xFF
