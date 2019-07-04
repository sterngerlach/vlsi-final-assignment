
# fib.asm

main:
    addi $3, $0, 8
    addi $4, $0, 1
    addi $5, $0, -1
loop:
    beq $3, $0, end
    add $4, $4, $5
    sub $5, $4, $5
    addi $3, $3, -1
    j loop
end:
    sw $4, 4($0)

