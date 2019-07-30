
# find.asm

main:
    addi $1, $0, 99
    addi $2, $0, 100
    add  $3, $0, $0
loop:
    beq  $3, $2, end
    add  $5, $3, $3
    add  $5, $5, $5
    lw   $4, 0($5)
    beq  $4, $1, end
    addi $3, $3, 1
    j loop
end:
    sw   $5, 128($0)
    addi $7, $0, 127
    sw   $7, 192($0)

