`timescale 1ns / 1ps

module ascon_core_v1_unmasked (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    input  wire [1:0]   mode,
    input  wire [127:0] key_in,
    input  wire [127:0] nonce_in,
    input  wire [63:0]  data_in,
    output wire [63:0]  data_out,
    output wire [255:0] tag_out,
    output reg          ready,
    output reg          done
);
    reg [319:0] state;
    reg [3:0]   round;
    reg [1:0]   fsm_state;
    
    localparam ST_IDLE = 2'b00, ST_INIT = 2'b01, ST_PROCESS = 2'b10, ST_FINAL = 2'b11;

    wire [63:0] x0 = state[319:256];
    wire [63:0] x1 = state[255:192];
    wire [63:0] x2 = state[191:128];
    wire [63:0] x3 = state[127:64];
    wire [63:0] x4 = state[63:0];

    // 1. Addition of Constants (pC)
    reg [7:0] cr;
    always @(*) begin
        case (round)
            4'd0: cr=8'hf0; 4'd1: cr=8'he1; 4'd2: cr=8'hd2; 4'd3: cr=8'hc3;
            4'd4: cr=8'hb4; 4'd5: cr=8'ha5; 4'd6: cr=8'h96; 4'd7: cr=8'h87;
            4'd8: cr=8'h78; 4'd9: cr=8'h69; 4'd10: cr=8'h5a; 4'd11: cr=8'h4b;
            default: cr=8'h00;
        endcase
    end
    wire [63:0] x2_pc = x2 ^ {56'h0, cr};

    // 2. Substitution Layer (pS)
    function [4:0] sbox5 (input [4:0] in);
        case (in)
            5'h00: sbox5 = 5'h04; 5'h01: sbox5 = 5'h0b; 5'h02: sbox5 = 5'h1f; 5'h03: sbox5 = 5'h14;
            5'h04: sbox5 = 5'h1a; 5'h05: sbox5 = 5'h15; 5'h06: sbox5 = 5'h09; 5'h07: sbox5 = 5'h02;
            5'h08: sbox5 = 5'h1b; 5'h09: sbox5 = 5'h05; 5'h0a: sbox5 = 5'h08; 5'h0b: sbox5 = 5'h12;
            5'h0c: sbox5 = 5'h1d; 5'h0d: sbox5 = 5'h03; 5'h0e: sbox5 = 5'h06; 5'h0f: sbox5 = 5'h1c;
            5'h10: sbox5 = 5'h1e; 5'h11: sbox5 = 5'h13; 5'h12: sbox5 = 5'h07; 5'h13: sbox5 = 5'h0e;
            5'h14: sbox5 = 5'h00; 5'h15: sbox5 = 5'h0d; 5'h16: sbox5 = 5'h11; 5'h17: sbox5 = 5'h18;
            5'h18: sbox5 = 5'h10; 5'h19: sbox5 = 5'h0c; 5'h1a: sbox5 = 5'h01; 5'h1b: sbox5 = 5'h19;
            5'h1c: sbox5 = 5'h0a; 5'h1d: sbox5 = 5'h0f; 5'h1e: sbox5 = 5'h17; 5'h1f: sbox5 = 5'h16;
            default: sbox5 = 5'h00;
        endcase
    endfunction

    wire [63:0] x0_ps, x1_ps, x2_ps, x3_ps, x4_ps;
    genvar i;
    generate
        for (i=0; i<64; i=i+1) begin : gen_sbox
            wire [4:0] sb_out = sbox5({x4[i], x3[i], x2_pc[i], x1[i], x0[i]});
            assign x0_ps[i] = sb_out[0]; assign x1_ps[i] = sb_out[1];
            assign x2_ps[i] = sb_out[2]; assign x3_ps[i] = sb_out[3];
            assign x4_ps[i] = sb_out[4];
        end
    endgenerate

    // 3. Linear Diffusion (pL)
    wire [63:0] x0_pl = x0_ps ^ {x0_ps[18:0], x0_ps[63:19]} ^ {x0_ps[27:0], x0_ps[63:28]};
    wire [63:0] x1_pl = x1_ps ^ {x1_ps[60:0], x1_ps[63:61]} ^ {x1_ps[38:0], x1_ps[63:39]};
    wire [63:0] x2_pl = x2_ps ^ {x2_ps[0], x2_ps[63:1]} ^ {x2_ps[5:0], x2_ps[63:6]};
    wire [63:0] x3_pl = x3_ps ^ {x3_ps[9:0], x3_ps[63:10]} ^ {x3_ps[16:0], x3_ps[63:17]};
    wire [63:0] x4_pl = x4_ps ^ {x4_ps[6:0], x4_ps[63:7]} ^ {x4_ps[40:0], x4_ps[63:41]};

    wire [319:0] next_state = {x0_pl, x1_pl, x2_pl, x3_pl, x4_pl};
    assign data_out = x0 ^ data_in;
    assign tag_out = {x3 ^ key_in[127:64], x4 ^ key_in[63:0], x1, x2};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state <= ST_IDLE;
            state <= 320'h0;
            round <= 4'd0;
            ready <= 1'b1;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (fsm_state)
                ST_IDLE: begin
                    if (start) begin
                        state <= {64'h8040_0c06_0000_0000, key_in, nonce_in};
                        round <= 4'd0;
                        fsm_state <= ST_INIT;
                        ready <= 1'b0;
                    end else ready <= 1'b1;
                end
                ST_INIT: begin
                    state <= next_state;
                    if (round == 4'd11) begin
                        round <= 4'd6; // pB for Absorb
                        fsm_state <= ST_PROCESS;
                        state[319:256] <= next_state[319:256] ^ data_in; // Inject Data
                    end else round <= round + 1'b1;
                end
                ST_PROCESS: begin
                    state <= next_state;
                    if (round == 4'd11) begin
                        round <= 4'd0;
                        fsm_state <= ST_FINAL;
                        state[191:64] <= next_state[191:64] ^ key_in; // Final key XOR
                    end else round <= round + 1'b1;
                end
                ST_FINAL: begin
                    state <= next_state;
                    if (round == 4'd11) begin
                        done <= 1'b1;
                        fsm_state <= ST_IDLE;
                    end else round <= round + 1'b1;
                end
            endcase
        end
    end
endmodule