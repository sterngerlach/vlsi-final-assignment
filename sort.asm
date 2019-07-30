
# sort.asm

main:
    addi $1, $0, 0
    addi $2, $0, 0
    addi $3, $0, 9
loop0:
    slt  $4, $1, $3
    beq  $4, $0, end
    add  $2, $3, $0
loop1:
    slt  $4, $1, $2
    beq  $4, $0, end0
    add  $5, $2, $2
    add  $5, $5, $5
    lw   $6, 0($5)
    addi $5, $5, -4
    lw   $7, 0($5)
    slt  $4, $6, $7
    beq  $4, $0, end1
    add  $4, $7, $0
    add  $7, $6, $0
    add  $6, $4, $0
    sw   $7, 0($5)
    addi $5, $5, 4
    sw   $6, 0($5)
end1:
    addi $2, $2, -1
    j loop1
end0:
    addi $1, $1, 1
    j loop0
end:
    addi $7, $0, 127
    sw   $7, 192($0)

