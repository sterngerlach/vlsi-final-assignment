
# test.asm

main:
    lw   $1, 0($0)   # 17
    lw   $2, 4($0)   # 18
    lw   $3, 8($0)   # 19
    lw   $4, 12($0)  # 20
    add  $5, $1, $2  # 35
    sub  $6, $3, $1  # 2
    and  $7, $1, $2  # 16
    or   $8, $1, $2  # 19
    slt  $9, $2, $4  # 1
    addi $10, $1, 30 # 47
    sw   $5, 16($0)
    sw   $6, 20($0)
    sw   $7, 24($0)
    sw   $8, 28($0)
    sw   $9, 32($0)
    sw   $10, 36($0)

