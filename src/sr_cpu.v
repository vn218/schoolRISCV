/*
 * schoolRISCV - small RISC-V CPU 
 *
 * originally based on Sarah L. Harris MIPS CPU 
 *                   & schoolMIPS project
 * 
 * Copyright(c) 2017-2020 Stanislav Zhelnio 
 *                        Aleksandr Romanov 
 */ 

`include "sr_cpu.vh"

module sr_cpu
(
    input           clk,        // clock
    input           rst_n,      // reset
    input   [ 4:0]  regAddr,    // debug access reg address
    output  [31:0]  regData,    // debug access reg data
    output  [31:0]  imAddr,     // instruction memory address
    input   [31:0]  imData      // instruction memory data
);
    //control wires
    wire        aluZero;
    wire        pcSrc;
    wire        regWrite;
    reg         regWrite_pipe [0:1];
    wire        aluSrc;
    wire        wdSrc;
    reg         wdSrc_pipe [0:1];
    wire  [2:0] aluControl;

    //instruction decode wires
    wire [ 6:0] cmdOp;
    wire [ 4:0] rd;
    reg  [ 4:0] rd_pipe [0:1];
    wire [ 2:0] cmdF3;
    wire [ 4:0] rs1;
    wire [ 4:0] rs2;
    wire [ 6:0] cmdF7;
    wire [31:0] immI;
    wire [31:0] immB;
    wire [31:0] immU;
    reg  [31:0] immU_pipe [0:1];
    wire [1:0] bypass;
    wire [1:0] bypass_index;

    //program counter
    wire [31:0] pc;
    wire [31:0] pcBranch = pc + immB;
    wire [31:0] pcPlus4  = pc + 4;
    wire [31:0] pcNext   = pcSrc ? pcBranch : pcPlus4;
    sm_register r_pc(clk ,rst_n, pcNext, pc);

    //program memory access
    assign imAddr = pc >> 2;
    wire [31:0] instr = imData;

    //instruction decode
    sr_decode id (
        .instr      ( instr        ),
        .cmdOp      ( cmdOp        ),
        .rd         ( rd           ),
        .cmdF3      ( cmdF3        ),
        .rs1        ( rs1          ),
        .rs2        ( rs2          ),
        .cmdF7      ( cmdF7        ),
        .immI       ( immI         ),
        .immB       ( immB         ),
        .immU       ( immU         ) 
    );

    //register file
    wire [31:0] rd0;
    wire [31:0] rd1;
    wire [31:0] rd2;
    wire [31:0] wd3;

    sm_register_file rf (
        .clk        ( clk          ),
        .a0         ( regAddr      ),
        .a1         ( rs1          ),
        .a2         ( rs2          ),
        .a3         ( rd_pipe[1]   ),
        .rd0        ( rd0          ),
        .rd1        ( rd1          ),
        .rd2        ( rd2          ),
        .wd3        ( wd3          ),
        .we3        ( regWrite_pipe[1])
    );

    //debug register access
    assign regData = (regAddr != 0) ? rd0 : pc;

    //alu
    wire [31:0] srcB = aluSrc ? immI : rd2;
    wire [31:0] aluResult;

    sr_alu alu (
        .srcA       ( rd1          ),
        .srcB       ( srcB         ),
        .oper       ( aluControl   ),
        .zero       ( aluZero      ),
        .result     ( aluResult    ),
        .clk        (clk           ),
        .bypass     (bypass        ),
        .bypass_index(bypass_index ) 
    );

    always @ (posedge clk) begin
        wdSrc_pipe[0] <= wdSrc;
        wdSrc_pipe[1] <= wdSrc_pipe[0];
    end

    always @ (posedge clk) begin
        immU_pipe[0] <= immU;
        immU_pipe[1] <= immU_pipe[0];
    end

    always @ (posedge clk) begin
        regWrite_pipe[0] <= regWrite;
        regWrite_pipe[1] <= regWrite_pipe[0];
    end

    always @ (posedge clk) begin
        rd_pipe[0] <= rd;
        rd_pipe[1] <= rd_pipe[0];
    end
    assign wd3 = wdSrc_pipe[1] ? immU_pipe[1] : aluResult;

    //control
    sr_control sm_control (
        .cmdOp      ( cmdOp        ),
        .cmdF3      ( cmdF3        ),
        .cmdF7      ( cmdF7        ),
        .aluZero    ( aluZero      ),
        .pcSrc      ( pcSrc        ),
        .regWrite   ( regWrite     ),
        .aluSrc     ( aluSrc       ),
        .wdSrc      ( wdSrc        ),
        .aluControl ( aluControl   ),
        .rd         ( rd  ),
        .rs1        ( rs1  ),
        .rs2        ( rs2  ),
        .clk        ( clk  ),
        .bypass     ( bypass  ),
        .bypass_index(bypass_index  )
    );

endmodule

module sr_decode
(
    input      [31:0] instr,
    output     [ 6:0] cmdOp,
    output     [ 4:0] rd,
    output     [ 2:0] cmdF3,
    output     [ 4:0] rs1,
    output     [ 4:0] rs2,
    output     [ 6:0] cmdF7,
    output reg [31:0] immI,
    output reg [31:0] immB,
    output reg [31:0] immU 
);
    assign cmdOp = instr[ 6: 0];
    assign rd    = instr[11: 7];
    assign cmdF3 = instr[14:12];
    assign rs1   = instr[19:15];
    assign rs2   = instr[24:20];
    assign cmdF7 = instr[31:25];

    // I-immediate
    always @ (*) begin
        immI[10: 0] = instr[30:20];
        immI[31:11] = { 21 {instr[31]} };
    end

    // B-immediate
    always @ (*) begin
        immB[    0] = 1'b0;
        immB[ 4: 1] = instr[11:8];
        immB[10: 5] = instr[30:25];
        immB[   11] = instr[7];
        immB[31:12] = { 20 {instr[31]} };
    end

    // U-immediate
    always @ (*) begin
        immU[11: 0] = 12'b0;
        immU[31:12] = instr[31:12];
    end

endmodule

module sr_control
(
    input     [ 6:0] cmdOp,
    input     [ 2:0] cmdF3,
    input     [ 6:0] cmdF7,
    input            aluZero,
    input     [ 4:0] rd,
    input     [ 4:0] rs1,
    input     [ 4:0] rs2,
    input            clk,
    output     [1:0] bypass,
    output     [1:0] bypass_index,
    output           pcSrc, 
    output reg       regWrite, 
    output reg       aluSrc,
    output reg       wdSrc,
    output reg [2:0] aluControl
);
    reg          branch;
    reg          condZero;
    assign pcSrc = branch & (aluZero == condZero);
    
    reg [4:0] rd_buff [1:0];
    
    reg rs1_bypass, rs2_bypass;
    reg rs1_bypass_index, rs2_bypass_index;
    assign bypass = {rs2_bypass,rs1_bypass};
    assign bypass_index = {rs2_bypass_index,rs1_bypass_index};

    always @ (posedge clk) begin
        rd_buff[0] <= rd;
        rd_buff[1] <= rd_buff[0];
    end

    always @ (*) begin
        rs1_bypass = 0;
        rs2_bypass = 0;
        rs1_bypass_index = 0;
        rs2_bypass_index = 0;        

        if (rd_buff[0] == rs1 || rd_buff[1] == rs1) begin
            rs1_bypass = 1;
            if (rd_buff[1] == rs1 && rd_buff[0] != rs1 )
                rs1_bypass_index = 1;
        end

        if (rd_buff[0] == rs2 || rd_buff[1] == rs2) begin
            rs2_bypass = 1;
            if (rd_buff[1] == rs2 && rd_buff[0] != rs2 )
                rs2_bypass_index = 1;
        end
    end


    always @ (*) begin
        branch      = 1'b0;
        condZero    = 1'b0;
        regWrite    = 1'b0;
        aluSrc      = 1'b0;
        wdSrc       = 1'b0;
        aluControl  = `ALU_ADD;

        casez( {cmdF7, cmdF3, cmdOp} )
            { `RVF7_ADD,  `RVF3_ADD,  `RVOP_ADD  } : begin regWrite = 1'b1; aluControl = `ALU_ADD;  end
            { `RVF7_OR,   `RVF3_OR,   `RVOP_OR   } : begin regWrite = 1'b1; aluControl = `ALU_OR;   end
            { `RVF7_SRL,  `RVF3_SRL,  `RVOP_SRL  } : begin regWrite = 1'b1; aluControl = `ALU_SRL;  end
            { `RVF7_SLTU, `RVF3_SLTU, `RVOP_SLTU } : begin regWrite = 1'b1; aluControl = `ALU_SLTU; end
            { `RVF7_SUB,  `RVF3_SUB,  `RVOP_SUB  } : begin regWrite = 1'b1; aluControl = `ALU_SUB;  end
            { `RVF7_MUL,  `RVF3_MUL,  `RVOP_MUL  } : begin regWrite = 1'b1; aluControl = `ALU_MUL;  end

            { `RVF7_ANY,  `RVF3_ADDI, `RVOP_ADDI } : begin regWrite = 1'b1; aluSrc = 1'b1; aluControl = `ALU_ADD; end
            { `RVF7_ANY,  `RVF3_ANY,  `RVOP_LUI  } : begin regWrite = 1'b1; wdSrc  = 1'b1; end

            { `RVF7_ANY,  `RVF3_BEQ,  `RVOP_BEQ  } : begin branch = 1'b1; condZero = 1'b1; aluControl = `ALU_SUB; end
            { `RVF7_ANY,  `RVF3_BNE,  `RVOP_BNE  } : begin branch = 1'b1; aluControl = `ALU_SUB; end
        endcase
    end
endmodule

module sr_alu
(
    input  [31:0] srcA,
    input  [31:0] srcB,
    input  [ 2:0] oper,
    input         clk,
    input  [1:0]  bypass,
    input  [1:0]  bypass_index,
    output        zero,
    output reg [31:0] result
);
    reg [31:0] sub;
    wire [31:0] multiplier_out;
    reg [2:0] oper_pipe [0:1];
    reg [31:0] result_pipe [0:1];
    reg [31:0] result_comb;
    wire [31:0] srcA_mux, srcB_mux;

    assign srcA_mux = bypass[0]? result_pipe[bypass_index[0]] : srcA;
    assign srcB_mux = bypass[1]? result_pipe[bypass_index[1]] : srcB;

    always @ (posedge clk) begin
        oper_pipe[0] <= oper;
        oper_pipe[1] <= oper_pipe[0];
    end

    always @ (posedge clk) begin
        result_pipe[0] <= result_comb;
        result_pipe[1] <= result_pipe[0];
    end

    always @ (*) begin
        case (oper)
            default   : result_comb = srcA_mux + srcB_mux;
            `ALU_ADD  : result_comb = srcA_mux + srcB_mux;
            `ALU_OR   : result_comb = srcA_mux | srcB_mux;
            `ALU_SRL  : result_comb = srcA_mux >> srcB_mux [4:0];
            `ALU_SLTU : result_comb = (srcA_mux < srcB_mux) ? 1 : 0;
            `ALU_SUB : result_comb = srcA_mux - srcB_mux;
        endcase
    end

    always @ (*) begin
        case (oper_pipe[1])
            default   : result = result_pipe[1];
            `ALU_MUL : result = multiplier_out;
        endcase
    end

    assign zero   = (result_comb == 0);
    multiplier multiplier
    (
        .clk(clk),
        .valid(oper==`ALU_MUL),
        .srcA(srcA_mux),
        .srcB(srcB_mux),
        .result(multiplier_out)
    );
endmodule

module multiplier
(
    input clk,
    input valid,
    input signed [31:0] srcA,
    input signed [31:0] srcB,
    output [31:0] result
);
    reg [63:0] pipeline1, pipeline2;
    reg valid_pipe;
    assign result = pipeline2[31:0];

    always @ (posedge clk) begin
        valid_pipe <= valid;
        if (valid)
            pipeline1 <= srcA*srcB;
        
        if (valid_pipe)
            pipeline2 <= pipeline1;
    end 

endmodule

module sm_register_file
(
    input         clk,
    input  [ 4:0] a0,
    input  [ 4:0] a1,
    input  [ 4:0] a2,
    input  [ 4:0] a3,
    output [31:0] rd0,
    output [31:0] rd1,
    output [31:0] rd2,
    input  [31:0] wd3,
    input         we3
);
    reg [31:0] rf [31:0];

    assign rd0 = (a0 != 0) ? rf [a0] : 32'b0;
    assign rd1 = (a1 != 0) ? rf [a1] : 32'b0;
    assign rd2 = (a2 != 0) ? rf [a2] : 32'b0;

    always @ (posedge clk)
        if(we3) rf [a3] <= wd3;
endmodule
