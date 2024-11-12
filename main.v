module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );
    wire beq, blt, jal, jalr, reg_write, mem_to_reg, mem_write, alu_src;
    wire [2:0] imm_control;
    wire [2:0] alu_control;
    wire [31:0] wd3, rs1, rs2;
    wire [31:0] imm;
    wire [31:0] src_a, src_b;
    assign src_a = rs1;
    wire [31:0] alu_out;
    wire alu_zero, alu_lt;
    wire [31:0] branch_sum_data, branch_target;
    assign address_to_mem = alu_out;
    assign data_to_mem = rs2;
    assign WE = mem_write;
    wire [31:0] pc_in, pc_plus_4, jalx_mux_out;
    wire jalx;
    assign jalx = jal || jalr;
    wire branch_outcome;
    assign branch_outcome = (alu_lt && blt) || (beq && alu_zero) || jalx;
    control_unit control_unit(instruction, beq, blt, jal, jalr, reg_write, mem_to_reg, mem_write, alu_src, imm_control, alu_control);
    registers registers(reg_write, clk, reset, instruction[19:15], instruction[24:20], instruction[11:7], wd3, rs1, rs2);
    imm_decode imm_decoder (instruction [31:7], imm_control, imm );
    mux alu_mux(rs2, imm, alu_src, src_b);
    alu alu(src_a, src_b, alu_control, alu_out, alu_zero, alu_lt);
    register pc_reg(pc_in, clk, reset, PC);
    sum pc_inc(PC, 4, pc_plus_4);
    sum branch_sum(imm, PC, branch_sum_data);
    mux branch_target_mux(branch_sum_data, alu_out, jalr, branch_target);
    mux jalx_mux(alu_out, pc_plus_4, jalx, jalx_mux_out);
    mux reg_write_mux(jalx_mux_out, data_from_mem, mem_to_reg, wd3);
    mux pc_next_mux(pc_plus_4, branch_target, branch_outcome, pc_in);
endmodule

module register(input [31:0] in, inout clk, input reset, output [31:0] out);
    reg [31:0] data = 0;
    assign out = data;
    always @(posedge clk) begin
        data = in;
        if (reset) data = 0;
    end
endmodule

module sum (input [31:0] a, b, output [31:0] res);
    assign res = a + b;
endmodule

module mux (input [31:0] a, b, input select, output reg [31:0] out);
    always @(*) begin
        if (!select) out = a;
        else out = b;
    end
endmodule

module alu (input [31:0] a, b, input [2:0] alu_control, output reg [31:0] out, output reg zero, output reg lt);
    localparam [2:0] ALU_ADD = 0;
    localparam [2:0] ALU_SUB = 1;
    localparam [2:0] ALU_AND = 2;
    localparam [2:0] ALU_SRL = 3;
    localparam [2:0] ALU_LOG = 4;
    reg [7:0] exp;
    reg [22:0] m;
    always @(*) begin
        case (alu_control)
            ALU_ADD: out = a + b;
            ALU_SUB: out = a - b;
            ALU_AND: out = a & b;
            ALU_SRL: out = a >> b;
            ALU_LOG: begin
                // Float in IEEE 754
                // log2 (1.m * 2^(e-127)) = log2(1.m) + log2(2^(e-127)) = log2(1.m) + e - 127 = e - 127 (log2 of something from 1 to 2 is always 0).
                exp = a[30:23];
                out = exp - 127;
            end
        endcase
        zero = (out == 0);
        lt = out[31];
    end
endmodule

module imm_decode ( input [31:7] instruction, input [2:0] imm_control, output reg [31:0] imm );
    localparam [2:0] IMM_I = 1;
    localparam [2:0] IMM_S = 2;
    localparam [2:0] IMM_B = 3;
    localparam [2:0] IMM_U = 4;
    localparam [2:0] IMM_J = 5;
    always @(*) begin
        imm[31:0] = {32{1'b0}};
        case (imm_control)
            IMM_I: imm = {{20{instruction[31]}}, instruction[31:20]};
            IMM_S: imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            IMM_B: imm = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
            IMM_U: imm[31:12] = instruction[31:12];
            IMM_J: imm = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
        endcase
    end
endmodule

module registers ( input we3, clk, reset, input [4:0] a1, a2, a3, input [31:0] wd3, output [31:0] rd1, rd2);
    reg [31:0] data [31:0];
    assign rd1 = data[a1];
    assign rd2 = data[a2];
    integer i;
    always @(posedge clk) begin
        data[0] <= 0;
        if (we3 && a3 != 0) data[a3] = wd3;
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
               data[i] <= 0;
            end
        end
    end
endmodule

module control_unit ( input [31:0] instruction,
                      output reg beq, blt, jal, jalr, reg_write, mem_to_reg, mem_write, alu_src,
                      output reg [2:0] imm_control,
                      output reg [2:0] alu_control);
    localparam [2:0] ALU_ADD = 0;
    localparam [2:0] ALU_SUB = 1;
    localparam [2:0] ALU_AND = 2;
    localparam [2:0] ALU_SRL = 3;
    localparam [2:0] ALU_LOG = 4;
    localparam [2:0] IMM_OFF = 0;
    localparam [2:0] IMM_I = 1;
    localparam [2:0] IMM_S = 2;
    localparam [2:0] IMM_B = 3;
    localparam [2:0] IMM_U = 4;
    localparam [2:0] IMM_J = 5;
    always @(*) begin
        beq = 0;
        blt = 0;
        jal = 0;
        jalr = 0;
        reg_write = 0;
        mem_to_reg = 0;
        mem_write = 0;
        alu_src = 0;
        imm_control = IMM_OFF;
        alu_control = ALU_ADD;
        case (instruction[6:0])
            7'b0110011: begin // add, and, sub, slr
                reg_write = 1;
                if (instruction[14:12] == 3'b111) alu_control = ALU_AND;
                else if (instruction[14:12] == 3'b101) alu_control = ALU_SRL;
                else if (instruction[31:25] == 7'b0000000) alu_control = ALU_ADD;
                else if (instruction[31:25] == 7'b0100000) alu_control = ALU_SUB;
            end
            7'b0010011: begin // addi
                reg_write = 1;
                alu_src = 1;
                imm_control = IMM_I;
            end
            7'b1100011: begin // beq, blt
                alu_control = ALU_SUB;
                imm_control = IMM_B;
                if (instruction[14:12] == 3'b000) beq = 1;
                else blt = 1;
            end
            7'b0000011: begin // lw
                reg_write = 1;
                mem_to_reg = 1;
                alu_src = 1;
                imm_control = IMM_I;
            end
            7'b0100011: begin // sw
                mem_write = 1;
                alu_src = 1;
                imm_control = IMM_S;
            end
            7'b0110111: begin // lui
                reg_write = 1;
                alu_src = 1;
                imm_control = IMM_U;
            end
            7'b1101111: begin // jal
                jal = 1;
                reg_write = 1;
                // doesn't use alu
                imm_control = IMM_J;
            end
            7'b1100111: begin // jalr
                jalr = 1;
                reg_write = 1;
                alu_src = 1;
                imm_control = IMM_I;
            end
            7'b0001011: begin // floor_log
                reg_write = 1;
                alu_control = ALU_LOG;
                alu_src = 1;
                imm_control = IMM_I;
            end
        endcase
    end
endmodule