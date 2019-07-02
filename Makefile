
# Makefile

run: test.v mips.v imem.dat dmem.dat
	ncverilog +access+r test.v mips.v

test0: test0.asm
	./asm.pl -o imem.dat test0.asm

test1: test1.asm
	./asm.pl -o imem.dat test1.asm

