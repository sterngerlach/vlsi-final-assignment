
# test1.asm

main:
    lw $1, 0($0)
    lw $2, 4($0)
    lw $3, 8($0)
    lw $4, 12($0)
    lw $5, 16($0)
    addi $1, $1, 1
    beq $1, $2, end
    add $0, $0, $0
end:
    sw $5, 255($0)

