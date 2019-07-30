
/* test.v */

`timescale 1ns/100ps

`define DISABLE     0
`define ENABLE      1

module top #(parameter WIDTH = 32, REGBITS = 5)();
    
    localparam STEP = 7.7;

    reg              clk;
    reg              rst;
    reg  [WIDTH-1:0] count;
    reg  [WIDTH-1:0] stall;
    reg  [WIDTH-1:0] nop;
    wire [WIDTH-1:0] instr;
    wire [WIDTH-1:0] pc;
    wire [WIDTH-1:0] aluout;
    wire [WIDTH-1:0] writedata;
    wire             memwrite;
    wire [WIDTH-1:0] readdata;

    mips dut(clk, rst, pc, instr,
             aluout, writedata, memwrite, readdata);

    imem ex_imem(pc[15:2], instr);
    dmem ex_dmem(clk, memwrite, aluout[15:2], readdata, writedata);

    initial begin
        `ifdef __POST_PR__
            $sdf_annotate("mips.sdf", top.dut, , "sdf.log", "MAXIMUM");
        `endif

        $dumpfile("dump.vcd");
        $dumpvars(0, top);

        clk <= `DISABLE; rst <= `ENABLE;
        
        #(STEP * 3 / 4);
        #(STEP * 10);

        rst <= `DISABLE; count <= 0; stall <= 0; nop <= 0;

        #(STEP * 10000);

        $finish;
    end

    always #(STEP / 2) begin
        clk <= ~clk;
    end
    
    always @(posedge clk) begin
        count <= count + 1;

        if (dut.hazard_unit.stall_f)
            stall <= stall + 1;

        if (dut.nop_d)
            nop <= nop + 1;

        if (memwrite) begin
            $display("Data %h is stored in address %h", writedata, aluout[15:0]);
            $display("Clock: %d, stall: %d, nop: %d", count, stall, nop);
        end
        
        // if (dut.memwrite_m & dut.aluout_m == 192 & dut.writedata_m == 127) begin
        if (ex_dmem.mem[48] == 127) begin
            $display("Finished: clock: %d, stall: %d, nop: %d", count, stall, nop);
            $display("Dmem : %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d",
                     ex_dmem.mem[0], ex_dmem.mem[1], ex_dmem.mem[2],
                     ex_dmem.mem[3], ex_dmem.mem[4], ex_dmem.mem[5],
                     ex_dmem.mem[6], ex_dmem.mem[7], ex_dmem.mem[8],
                     ex_dmem.mem[9]);
            $finish;
        end
    end

endmodule

