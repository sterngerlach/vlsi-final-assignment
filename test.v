
/* test.v */

`timescale 1ns/100ps

`define DISABLE     0
`define ENABLE      1

module top #(parameter WIDTH = 32, REGBITS = 5)();
    
    localparam STEP = 10.0;

    reg              clk;
    reg              rst;
    reg  [WIDTH-1:0] count;
    reg  [WIDTH-1:0] stall;
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
        $dumpfile("dump.vcd");
        $dumpvars(0, top);

        clk <= `DISABLE; rst <= `ENABLE;
        
        #(STEP * 3 / 4);
        #(STEP * 10);

        rst <= `DISABLE; count <= 0; stall <= 0;

        #(STEP * 400);

        $finish;
    end

    always #(STEP / 2) begin
        clk <= ~clk;
    end
    
    always @(posedge clk) begin
        count <= count + 1;

        if (dut.hazard_unit.stall_f)
            stall <= stall + 1;

        if (memwrite) begin
            $display("Data %h is stored in address %h", writedata, aluout[15:0]);
            $display("Clock: %d, stall: %d", count, stall);
        end
    end

endmodule

