
# Makefile

run: test.v mips.v imem.dat dmem.dat
	ncverilog +access+r test.v mips.v

test0: test0.asm
	./asm.pl -o imem.dat test0.asm

test1: test1.asm
	./asm.pl -o imem.dat test1.asm

hazard0: hazard0.asm
	./asm.pl -o imem.dat hazard0.asm

hazard1: hazard1.asm
	./asm.pl -o imem.dat hazard1.asm

hazard2: hazard2.asm
	./asm.pl -o imem.dat hazard2.asm

fib: fib.asm
	./asm.pl -o imem.dat fib.asm

