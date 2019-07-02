
/* test.v */

`timescale 10ns/100ps

`define DISABLE     0
`define ENABLE      1

module top #(parameter WIDTH = 32, REGBITS = 5)();
    
    localparam STEP = 10.0;

    reg              clk;
    reg              rst;
    wire [WIDTH-1:0] instr;
    wire [WIDTH-1:0] pc;
    wire [WIDTH-1:0] aluout;
    wire [WIDTH-1:0] writedata;
    wire             memwrite;
    wire [WIDTH-1:0] readdata;

    mips dut(clk, rst, pc, instr,
             aluout, writedata, memwrite, readdata);

    imem ex_imem(pc[15:0], instr);
    dmem ex_dmem(clk, memwrite, aluout[15:0], readdata, writedata);

    initial begin
        clk <= `DISABLE; rst <= `ENABLE; #STEP; rst <= `DISABLE;
        $dumpfile("dump.vcd");
        $dumpvars(0, top);

        #(STEP * 100);

        $finish;
    end

    always #(STEP / 2) begin
        clk <= ~clk;
    end
    
    always @(negedge clk) begin
        if (memwrite)
            $display("Data %h is stored in address %h", writedata, aluout[15:0]);
    end

endmodule

