
/* mips.v */

`define ALU_ADD     3'b010
`define ALU_SUB     3'b110
`define ALU_AND     3'b000
`define ALU_OR      3'b001
`define ALU_SLT     3'b111
`define ALU_NOP     3'b101

module mips #(parameter WIDTH = 32, REGBITS = 5)
             (input              clk,
              input              rst,
              output [WIDTH-1:0] pc,
              input  [WIDTH-1:0] instr,
              output [WIDTH-1:0] aluout,
              output [WIDTH-1:0] writedata,
              output             memwrite,
              input  [WIDTH-1:0] readdata);
    
    localparam OPCODEW   = 6;
    localparam SHAMTW    = 5;
    localparam FUNCTW    = 6;
    localparam IMMW      = 16;
    localparam JADDRW    = 26;

    localparam OP_RTYPE  = 6'b000000;
    localparam OP_LB     = 6'b100000;
    localparam OP_SB     = 6'b101000;
    localparam OP_LW     = 6'b100011;
    localparam OP_SW     = 6'b101011;
    localparam OP_BEQ    = 6'b000100;
    localparam OP_ADDI   = 6'b001000;
    localparam OP_J      = 6'b000010;
    
    /*
     * F stage: Instruction Fetch stage
     * D stage: Instruction Decode stage
     * E stage: Execution stage
     * M stage: Memory stage
     * W stage: Write back stage
     */

    /*
     * F stage: Instruction fetch stage
     */
    wire [WIDTH-1:0] pcbranch_d;
    wire [WIDTH-1:0] pcbranch_f;
    wire             pcsrc_d;
    wire [WIDTH-1:0] pcjump_d;
    wire             jump_d;

    wire [WIDTH-1:0] pcplus4_f;
    wire [WIDTH-1:0] pcnext;
    wire             stall_f;
    wire             stall_d;

    wire [WIDTH-1:0] instr_d;
    wire [WIDTH-1:0] pcplus4_d;
    wire             rst_d;
    
    assign pcplus4_f = pc + 4;

    mux2    #(WIDTH) pcbranch_mux(pcplus4_f, pcbranch_d, pcsrc_d, pcbranch_f);
    mux2    #(WIDTH) pcnext_mux(pcbranch_f, pcjump_d, jump_d, pcnext);
    flopenr #(WIDTH) pc_flop(clk, rst, !stall_f, pcnext, pc);

    /* Transfer to D stage */
    assign rst_d = rst | pcsrc_d | jump_d;

    flopenr #(WIDTH) instr_d_flop(clk, rst_d, !stall_d, instr, instr_d);
    flopenr #(WIDTH) pcplus4_d_flop(clk, rst_d, !stall_d, pcplus4_f, pcplus4_d);

    /*
     * D stage: Instruction Decode stage
     */
    wire [OPCODEW-1:0] opcode_d;
    wire [REGBITS-1:0] rs_d;
    wire [REGBITS-1:0] rt_d;
    wire [REGBITS-1:0] rd_d;
    wire [SHAMTW-1:0]  shamt_d;
    wire [FUNCTW-1:0]  funct_d;
    wire [IMMW-1:0]    imm_d;
    wire [WIDTH-1:0]   signimm_d;
    
    /* Decompose instruction */
    assign { opcode_d, rs_d, rt_d, rd_d, shamt_d, funct_d } = instr_d;
    assign imm_d      = instr_d[IMMW-1:0];
    assign signimm_d  = {{(WIDTH-IMMW){imm_d[IMMW-1]}}, imm_d};
    assign pcbranch_d = {signimm_d[WIDTH-3:0],2'b00} + pcplus4_d;
    
    wire [JADDRW-1:0]  addr_d;
    
    assign addr_d     = instr_d[JADDRW-1:0];
    assign pcjump_d   = {pcplus4_d[WIDTH-1:WIDTH-4],addr_d,2'b00};

    /* Register file output */
    wire               regwrite_w;
    wire [REGBITS-1:0] writereg_w;
    wire [WIDTH-1:0]   result_w;

    wire [WIDTH-1:0]   rd1_d;
    wire [WIDTH-1:0]   rd2_d;
    
    regfile #(WIDTH, REGBITS) rfile(clk, regwrite_w, rs_d, rt_d,
                                    writereg_w, result_w,
                                    rd1_d, rd2_d);

    /* Transfer to E stage */
    wire [WIDTH-1:0]   rd1_e;
    wire [WIDTH-1:0]   rd2_e;
    wire [REGBITS-1:0] rs_e;
    wire [REGBITS-1:0] rt_e;
    wire [REGBITS-1:0] rd_e;
    wire [WIDTH-1:0]   signimm_e;
    wire               flush_e;
    wire               rst_e;
    
    wire [WIDTH-1:0] forwarda_d;
    wire [WIDTH-1:0] forwardb_d;
    wire             forwarda_sel_d;
    wire             forwardb_sel_d;

    wire [WIDTH-1:0] aluout_m;

    mux2 #(WIDTH) forwarda_d_mux(rd1_d, aluout_m, forwarda_sel_d, forwarda_d);
    mux2 #(WIDTH) forwardb_d_mux(rd2_d, aluout_m, forwardb_sel_d, forwardb_d);
    
    assign rst_e = rst | flush_e;

    flopr #(WIDTH)   rd1_e_flop(clk, rst_e, forwarda_d, rd1_e);
    flopr #(WIDTH)   rd2_e_flop(clk, rst_e, forwardb_d, rd2_e);
    flopr #(REGBITS) rs_e_flop(clk, rst_e, rs_d, rs_e);
    flopr #(REGBITS) rt_e_flop(clk, rst_e, rt_d, rt_e);
    flopr #(REGBITS) rd_e_flop(clk, rst_e, rd_d, rd_e);
    flopr #(WIDTH)   signimm_e_flop(clk, rst_e, signimm_d, signimm_e);

    /* Controller output */
    assign op_rtype_d = (opcode_d == OP_RTYPE);
    assign op_lb_d    = (opcode_d == OP_LB);
    assign op_sb_d    = (opcode_d == OP_SB);
    assign op_lw_d    = (opcode_d == OP_LW);
    assign op_sw_d    = (opcode_d == OP_SW);
    assign op_beq_d   = (opcode_d == OP_BEQ);
    assign op_addi_d  = (opcode_d == OP_ADDI);
    assign op_j_d     = (opcode_d == OP_J);

    wire [1:0] aluop_d;
    wire [2:0] alucontrol_d;

    assign regwrite_d   = op_rtype_d | op_lw_d | op_lb_d | op_addi_d;
    assign memtoreg_d   = op_lw_d | op_lb_d;
    assign memwrite_d   = op_sw_d | op_sb_d;
    assign branch_d     = op_beq_d;
    assign alusrc_d     = op_lw_d | op_sw_d | op_lb_d | op_sb_d | op_addi_d;
    assign regdst_d     = op_rtype_d;
    assign jump_d       = op_j_d;

    assign aluop_d      = (op_lw_d | op_sw_d | op_lb_d | op_sb_d | op_addi_d) ? 2'b00 :
                          (op_beq_d) ? 2'b01 :
                          (op_rtype_d) ? 2'b10 : 2'b11;

    alucontrol alucont(aluop_d, funct_d, alucontrol_d);

    assign equal_d      = forwarda_d == forwardb_d;
    assign pcsrc_d      = equal_d & branch_d;

    /* Transfer to E stage */
    wire       regwrite_e;
    wire       memtoreg_e;
    wire       memwrite_e;
    wire       branch_e;
    wire       alusrc_e;
    wire       regdst_e;
    wire [2:0] alucontrol_e;

    flopr #(1) regwrite_e_flop(clk, rst_e, regwrite_d, regwrite_e);
    flopr #(1) memtoreg_e_flop(clk, rst_e, memtoreg_d, memtoreg_e);
    flopr #(1) memwrite_e_flop(clk, rst_e, memwrite_d, memwrite_e);
    flopr #(1) branch_e_flop(clk, rst_e, branch_d, branch_e);
    flopr #(1) alusrc_e_flop(clk, rst_e, alusrc_d, alusrc_e);
    flopr #(1) regdst_e_flop(clk, rst_e, regdst_d, regdst_e);
    flopr #(3) alucontrol_e_flop(clk, rst_e, alucontrol_d, alucontrol_e);

    /*
     * E stage: Execution stage
     */
    wire [WIDTH-1:0]   srca_e;
    wire [WIDTH-1:0]   srcb_e;
    wire [1:0]         forwarda_sel_e;
    wire [1:0]         forwardb_sel_e;
    wire [WIDTH-1:0]   forwarda_e;
    wire [WIDTH-1:0]   forwardb_e;
    wire [REGBITS-1:0] writereg_e;
    wire [WIDTH-1:0]   aluout_e;
    wire               zero_e;
    wire [WIDTH-1:0]   writedata_e;
    
    mux4 #(WIDTH) forwarda_e_mux(rd1_e, result_w, aluout_m, 0, forwarda_sel_e, forwarda_e);
    mux4 #(WIDTH) forwardb_e_mux(rd2_e, result_w, aluout_m, 0, forwardb_sel_e, forwardb_e);
    mux2 #(WIDTH) srcb_e_mux(forwardb_e, signimm_e, alusrc_e, srcb_e);

    assign srca_e      = forwarda_e;
    assign writedata_e = forwardb_e;

    mux2 #(REGBITS) writereg_e_mux(rt_e, rd_e, regdst_e, writereg_e);
    alu  #(WIDTH)   alu0(srca_e, srcb_e, alucontrol_e, aluout_e, zero_e);

    /* Transfer to M stage */
    wire               regwrite_m;
    wire               memtoreg_m;
    wire               memwrite_m;
    wire               branch_m;
    wire               zero_m;
    wire [WIDTH-1:0]   writedata_m;
    wire [REGBITS-1:0] writereg_m;
    
    flopr #(1)       regwrite_m_flop(clk, rst, regwrite_e, regwrite_m);
    flopr #(1)       memtoreg_m_flop(clk, rst, memtoreg_e, memtoreg_m);
    flopr #(1)       memwrite_m_flop(clk, rst, memwrite_e, memwrite_m);
    flopr #(1)       branch_m_flop(clk, rst, branch_e, branch_m);
    flopr #(1)       zero_m_flop(clk, rst, zero_e, zero_m);
    flopr #(WIDTH)   aluout_m_flop(clk, rst, aluout_e, aluout_m);
    flopr #(WIDTH)   writedata_m_flop(clk, rst, writedata_e, writedata_m);
    flopr #(REGBITS) writereg_m_flop(clk, rst, writereg_e, writereg_m);

    /*
     * M stage: Memory stage
     */

    /* Data memory */
    wire [WIDTH-1:0] readdata_m;

    assign aluout     = aluout_m;
    assign writedata  = writedata_m;
    assign memwrite   = memwrite_m;
    assign readdata_m = readdata;

    /* Transfer to W stage */
    wire             memtoreg_w;
    wire [WIDTH-1:0] aluout_w;
    wire [WIDTH-1:0] readdata_w;

    flopr #(1)       regwrite_w_flop(clk, rst, regwrite_m, regwrite_w);
    flopr #(1)       memtoreg_w_flop(clk, rst, memtoreg_m, memtoreg_w);
    flopr #(WIDTH)   aluout_w_flop(clk, rst, aluout_m, aluout_w);
    flopr #(WIDTH)   readdata_w_flop(clk, rst, readdata_m, readdata_w);
    flopr #(REGBITS) writereg_w_flop(clk, rst, writereg_m, writereg_w);

    /*
     * W stage: Write back stage
     */
    mux2 #(WIDTH) result_w_mux(aluout_w, readdata_w, memtoreg_w, result_w);

    /*
     * Hazard unit
     */
    hazard_detect #(WIDTH, REGBITS) hazard_unit(
        rs_d, rt_d, rs_e, rt_e,
        branch_d,
        memtoreg_e, memtoreg_m,
        regwrite_e, regwrite_m, regwrite_w,
        writereg_e, writereg_m, writereg_w,
        stall_f, stall_d, flush_e,
        forwarda_sel_d, forwardb_sel_d,
        forwarda_sel_e, forwardb_sel_e);

endmodule

module alucontrol #(parameter FUNCTW = 6)
                   (input  [1:0]        aluop,
                    input  [FUNCTW-1:0] funct,
                    output [2:0]        alucont);

    localparam FUNCT_ADD = 6'b100000;
    localparam FUNCT_SUB = 6'b100010;
    localparam FUNCT_AND = 6'b100100;
    localparam FUNCT_OR  = 6'b100101;
    localparam FUNCT_SLT = 6'b101010;

    assign funct_add = (funct == FUNCT_ADD);
    assign funct_sub = (funct == FUNCT_SUB);
    assign funct_and = (funct == FUNCT_AND);
    assign funct_or  = (funct == FUNCT_OR);
    assign funct_slt = (funct == FUNCT_SLT);
    
    assign alucont = (aluop == 2'b00) ? `ALU_ADD :
                     (aluop == 2'b01) ? `ALU_SUB :
                     (aluop == 2'b10 && funct_add) ? `ALU_ADD :
                     (aluop == 2'b10 && funct_sub) ? `ALU_SUB :
                     (aluop == 2'b10 && funct_and) ? `ALU_AND :
                     (aluop == 2'b10 && funct_or)  ? `ALU_OR  :
                     (aluop == 2'b10 && funct_slt) ? `ALU_SLT : `ALU_NOP;

endmodule

module hazard_detect #(parameter WIDTH = 32, REGBITS = 5)
                      (input  [REGBITS-1:0] rs_d,
                       input  [REGBITS-1:0] rt_d,
                       input  [REGBITS-1:0] rs_e,
                       input  [REGBITS-1:0] rt_e,
                       input                branch_d,
                       input                memtoreg_e,
                       input                memtoreg_m,
                       input                regwrite_e,
                       input                regwrite_m,
                       input                regwrite_w,
                       input  [REGBITS-1:0] writereg_e,
                       input  [REGBITS-1:0] writereg_m,
                       input  [REGBITS-1:0] writereg_w,
                       output               stall_f,
                       output               stall_d,
                       output               flush_e,
                       output               forwarda_sel_d,
                       output               forwardb_sel_d,
                       output [1:0]         forwarda_sel_e,
                       output [1:0]         forwardb_sel_e);
    
    /* Forwarding */
    assign forwarda_sel_d =
        (rs_d != 0) && (rs_d == writereg_m) && regwrite_m;
    assign forwardb_sel_d =
        (rt_d != 0) && (rt_d == writereg_m) && regwrite_m;

    assign forwarda_sel_e =
        ((rs_e != 0) && (rs_e == writereg_m) && regwrite_m) ? 2'b10 :
        ((rs_e != 0) && (rs_e == writereg_w) && regwrite_w) ? 2'b01 : 2'b00;
    assign forwardb_sel_e =
        ((rt_e != 0) && (rt_e == writereg_m) && regwrite_m) ? 2'b10 :
        ((rt_e != 0) && (rt_e == writereg_w) && regwrite_w) ? 2'b01 : 2'b00;
    
    /* Load word hazard */
    assign lw_stall = ((rs_d == rt_e) || (rt_d == rt_e)) && memtoreg_e;
    
    /* Branch hazard */
    assign branch_stall =
        (branch_d && regwrite_e &&
            ((writereg_e == rs_d && rs_d != 0) ||
             (writereg_e == rt_d && rt_d != 0))) ||
        (branch_d && memtoreg_m &&
            ((writereg_m == rs_d && rs_d != 0) ||
             (writereg_m == rt_d && rt_d != 0)));

    /* assign branch_stall = 
        (branch_d && regwrite_e &&
            ((writereg_e == rs_d) || (writereg_e == rt_d))) ||
        (branch_d && memtoreg_m &&
            ((writereg_m == rs_d) || (writereg_m == rt_d))); */
    
    assign stall    = lw_stall || branch_stall;
    assign stall_f  = stall;
    assign stall_d  = stall;
    assign flush_e  = stall;

endmodule

module imem #(parameter WIDTH = 32, IMEMDEPTH = 14)
             (input  [IMEMDEPTH-1:0] addr,
              output [WIDTH-1:0]     rd);

    reg [WIDTH-1:0] mem[0:(1<<IMEMDEPTH)-1];

    assign rd = mem[addr];

    initial begin
        $readmemh("imem.dat", mem);
    end

endmodule

module dmem #(parameter WIDTH = 32, DMEMDEPTH = 14)
             (input                  clk,
              input                  memwrite,
              input  [DMEMDEPTH-1:0] addr,
              output [WIDTH-1:0]     rd,
              input  [WIDTH-1:0]     wd);

    reg [WIDTH-1:0] mem[0:(1<<DMEMDEPTH)-1];

    assign rd = mem[addr];

    always @(posedge clk) begin
        if (memwrite)
            mem[addr] <= wd;
    end
    
    initial begin
        $readmemh("dmem.dat", mem);
    end

endmodule

module alu #(parameter WIDTH = 32)
            (input  [WIDTH-1:0] a,
             input  [WIDTH-1:0] b,
             input  [2:0]       s,
             output [WIDTH-1:0] y,
             output             zero);
    
    assign y = (s == `ALU_ADD) ? a + b :
               (s == `ALU_SUB) ? a - b :
               (s == `ALU_AND) ? a & b :
               (s == `ALU_OR)  ? a | b :
               (s == `ALU_SLT) ? ((a < b) ? 1 : 0) : 0;

    assign zero = (y == 0);

endmodule

module regfile #(parameter WIDTH = 32, REGBITS = 5)
                (input                clk,
                 input                regwrite,
                 input  [REGBITS-1:0] ra1,
                 input  [REGBITS-1:0] ra2,
                 input  [REGBITS-1:0] wa,
                 input  [WIDTH-1:0]   wd,
                 output [WIDTH-1:0]   rd1,
                 output [WIDTH-1:0]   rd2);
    
    reg [WIDTH-1:0] rf[0:(1<<REGBITS)-1];

    assign rd1 = (|ra1 == 0) ? 0 : rf[ra1];
    assign rd2 = (|ra2 == 0) ? 0 : rf[ra2];

    always @(negedge clk) begin
        if (regwrite)
            rf[wa] <= wd;
    end

endmodule

module flop #(parameter WIDTH = 32)
             (input                  clk,
              input      [WIDTH-1:0] d,
              output reg [WIDTH-1:0] q);

    always @(posedge clk) begin
        q <= d;
    end

endmodule

module flopr #(parameter WIDTH = 32)
              (input                  clk,
               input                  rst,
               input      [WIDTH-1:0] d,
               output reg [WIDTH-1:0] q);

    always @(posedge clk) begin
        if (rst)
            q <= 0;
        else
            q <= d;
    end

endmodule

module flopenr #(parameter WIDTH = 32)
                (input                  clk,
                 input                  rst,
                 input                  en,
                 input      [WIDTH-1:0] d,
                 output reg [WIDTH-1:0] q);

    always @(posedge clk) begin
        if (rst)
            q <= 0;
        else if (en)
            q <= d;
    end

endmodule

module mux2 #(parameter WIDTH = 32)
             (input  [WIDTH-1:0] d0,
              input  [WIDTH-1:0] d1,
              input              s,
              output [WIDTH-1:0] y);

    assign y = s ? d1 : d0;

endmodule

module mux4 #(parameter WIDTH = 32)
             (input  [WIDTH-1:0] d0,
              input  [WIDTH-1:0] d1,
              input  [WIDTH-1:0] d2,
              input  [WIDTH-1:0] d3,
              input  [1:0]       s,
              output [WIDTH-1:0] y);
    
    assign y = (s == 2'b00) ? d0 :
               (s == 2'b01) ? d1 :
               (s == 2'b10) ? d2 : d3;

endmodule

