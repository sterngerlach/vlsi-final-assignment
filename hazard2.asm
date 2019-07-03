
# hazard2.asm

main:
    addi $3, $0, 63
    addi $4, $0, 28
    addi $5, $0, 42
    addi $6, $0, 63
    addi $7, $0, 112
    addi $1, $0, 35
    addi $2, $0, 35
    addi $3, $3, -1
    beq $1, $2, hoge
    add $3, $4, $5   # This must not be executed (must be discarded)
    or $4, $5, $6    # This must not be executed
    sub $5, $6, $7   # This must not be executed
hoge:
    slt $8, $3, $6   # 1 ($3 must contain the value 62)
    
