
# sum.asm

main:
	addi $3, $0, 0    # s = 0
	addi $4, $0, 240  # i = 240
loop:
	beq  $4, $0, end  # done with loop i = 0
	add  $3, $3, $4	  # s += i
    addi $4, $4, -1   # i--
    sub  $3, $3, $4   # s -= i
	addi $4, $4, -1   # i--
	j loop            # repeat until done
end:
	sw   $3, 128($0)  # store
    addi $7, $0, 127
    sw   $7, 192($0)

