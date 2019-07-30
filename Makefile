
# Makefile

# Step 1: RTL Simulation
sim: test.v mips.v imem.dat dmem.dat
	ncverilog +access+r test.v mips.v | tee sim.log

test0: test0.asm
	./asm32.pl -o imem.dat test0.asm

test1: test1.asm
	./asm32.pl -o imem.dat test1.asm

hazard0: hazard0.asm
	./asm32.pl -o imem.dat hazard0.asm

hazard1: hazard1.asm
	./asm32.pl -o imem.dat hazard1.asm

hazard2: hazard2.asm
	./asm32.pl -o imem.dat hazard2.asm

fib: fib.asm
	./asm32.pl -o fib.dat fib.asm

sum: sum.asm
	./asm32.pl -o sum.dat sum.asm

find: find.asm
	./asm32.pl -o find.dat find.asm

sort: sort.asm
	./asm32.pl -o sort.dat sort.asm

# Step 2: Synthesis
syn:
	dc_shell-xg-t -f scripts/syn.tcl | tee syn.log

# Step 3: Place-and-route
par:
	innovus -win -init scripts/par.tcl | tee par.log

# Step 4: Static Timing Analysis
sta:
	dc_shell-xg-t -f scripts/sta.tcl | tee sta.log

# Step 5: Delay Simulation
dsim:
	ncverilog +access+r +define+__POST_PR__ test.v mips_final.vnet -v ~matutani/lib/cells.v | tee dsim.log

# Step 6: Power Estimation
power:
	vcd2saif -input dump.vcd -output mips.saif
	dc_shell-xg-t -f scripts/power.tcl | tee power.log

.PHONY: clean
.PHONY: allclean

# Remove unnecessary files
clean:
	rm -rf INCA_libs ncverilog.log dump.trn dump.dsn sdf.log mips.saif
	rm -rf command.log default.svf WORK *.enc.dat *.enc *.old *.err scheduling_file.cts.*
	rm -rf innovus.cmd* innovus.log* *.rpt timingReports Default.view

allclean:
	make clean
	rm -f sim.log syn.log par.log sta.log power.log dsim.log dump.vcd
	rm -f mips.vnet mips_final.vnet mips.sdc mips.sdf mips.spef mips.sdf.X
	rm -rf .simvision .cadence .*.rs.fp .*.rs.fp.spr

