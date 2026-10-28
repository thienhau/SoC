`timescale 1ns / 1ps

module ascon_core_v2_masked (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    input  wire [1:0]   mode,
    input  wire [127:0] key_in,
    input  wire [127:0] nonce_in,
    input  wire [63:0]  data_in,
    input  wire [127:0] entropy_128b,
    output wire [63:0]  data_out,
    output wire [255:0] tag_out,
    output reg          ready,
    output reg          done
);
    // 3 Shares: 320 * 3 = 960 bits
    reg [319:0] st_A, st_B, st_C;
    reg [3:0]   round;
    reg [1:0]   fsm_state;
    
    localparam ST_IDLE = 2'b00, ST_INIT = 2'b01, ST_PROCESS = 2'b10, ST_FINAL = 2'b11;

    // 1. Addition of Constants (Applied ONLY to Share A to save area)
    reg [7:0] cr;
    always @(*) begin
        case (round)
            4'd0: cr=8'hf0; 4'd1: cr=8'he1; 4'd2: cr=8'hd2; 4'd3: cr=8'hc3;
            4'd4: cr=8'hb4; 4'd5: cr=8'ha5; 4'd6: cr=8'h96; 4'd7: cr=8'h87;
            4'd8: cr=8'h78; 4'd9: cr=8'h69; 4'd10: cr=8'h5a; 4'd11: cr=8'h4b;
            default: cr=8'h00;
        endcase
    end
    wire [63:0] x2_A_pc = st_A[191:128] ^ {56'h0, cr};
    wire [63:0] x2_B_pc = st_B[191:128];
    wire [63:0] x2_C_pc = st_C[191:128];

    // 2. Substitution Layer (3-Share TI - Algebraic Normal Form)
    wire [319:0] ps_A, ps_B, ps_C;
    genvar i;
    generate
        for (i=0; i<64; i=i+1) begin : gen_masked_sbox
            // Extract bits
            wire a0=st_A[256+i], a1=st_A[192+i], a2=x2_A_pc[i], a3=st_A[64+i], a4=st_A[i];
            wire b0=st_B[256+i], b1=st_B[192+i], b2=x2_B_pc[i], b3=st_B[64+i], b4=st_B[i];
            wire c0=st_C[256+i], c1=st_C[192+i], c2=x2_C_pc[i], c3=st_C[64+i], c4=st_C[i];

            // Affine 1
            wire a0_a = a0^a4; wire a2_a = a2^a1; wire a4_a = a4^a3;
            wire b0_a = b0^b4; wire b2_a = b2^b1; wire b4_a = b4^b3;
            wire c0_a = c0^c4; wire c2_a = c2^c1; wire c4_a = c4^c3;
            wire na1 = ~a1; wire na2 = ~a2_a; wire na3 = ~a3; wire na4 = ~a4_a; wire na0 = ~a0_a;

            // Non-Linear TI
            wire ta0 = a0_a ^ (b1&b2_a) ^ (b1&c2_a) ^ (c1&b2_a);
            wire ta1 = a1   ^ (b2_a&b3) ^ (b2_a&c3) ^ (c2_a&b3);
            wire ta2 = a2_a ^ (b3&b4_a) ^ (b3&c4_a) ^ (c3&b4_a);
            wire ta3 = a3   ^ (b4_a&b0_a) ^ (b4_a&c0_a) ^ (c4_a&b0_a);
            wire ta4 = a4_a ^ (b0_a&b1) ^ (b0_a&c1) ^ (c0_a&b1);

            wire tb0 = b0_a ^ (c1&c2_a) ^ (c1&a2_a) ^ (na1&c2_a);
            wire tb1 = b1   ^ (c2_a&c3) ^ (c2_a&a3)   ^ (na2&c3);
            wire tb2 = b2_a ^ (c3&c4_a) ^ (c3&a4_a) ^ (na3&c4_a);
            wire tb3 = b3   ^ (c4_a&c0_a) ^ (c4_a&a0_a) ^ (na4&c0_a);
            wire tb4 = b4_a ^ (c0_a&c1) ^ (c0_a&a1)   ^ (na0&c1);

            wire tc0 = c0_a ^ (na1&a2_a) ^ (na1&b2_a) ^ (b1&a2_a);
            wire tc1 = c1   ^ (na2&a3)   ^ (na2&b3)   ^ (b2_a&a3);
            wire tc2 = c2_a ^ (na3&a4_a) ^ (na3&b4_a) ^ (b3&a4_a);
            wire tc3 = c3   ^ (na4&a0_a) ^ (na4&b0_a) ^ (b4_a&a0_a);
            wire tc4 = c4_a ^ (na0&a1)   ^ (na0&b1)   ^ (b0_a&a1);

            // Affine 2
            assign ps_A[256+i] = ta0^ta4; assign ps_A[192+i] = ta1^ta0; assign ps_A[128+i] = ~ta2; assign ps_A[64+i] = ta3^ta2; assign ps_A[i] = ta4;
            assign ps_B[256+i] = tb0^tb4; assign ps_B[192+i] = tb1^tb0; assign ps_B[128+i] = tb2;  assign ps_B[64+i] = tb3^tb2; assign ps_B[i] = tb4;
            assign ps_C[256+i] = tc0^tc4; assign ps_C[192+i] = tc1^tc0; assign ps_C[128+i] = tc2;  assign ps_C[64+i] = tc3^tc2; assign ps_C[i] = tc4;
        end
    endgenerate

    // 3. Linear Diffusion Layer (pL) for all 3 shares
    wire [319:0] next_A, next_B, next_C;
    
    assign next_A[319:256] = ps_A[319:256] ^ {ps_A[274:256], ps_A[319:275]} ^ {ps_A[283:256], ps_A[319:284]};
    assign next_A[255:192] = ps_A[255:192] ^ {ps_A[252:192], ps_A[255:253]} ^ {ps_A[230:192], ps_A[255:231]};
    assign next_A[191:128] = ps_A[191:128] ^ {ps_A[128], ps_A[191:129]} ^ {ps_A[133:128], ps_A[191:134]};
    assign next_A[127:64]  = ps_A[127:64]  ^ {ps_A[73:64], ps_A[127:74]} ^ {ps_A[80:64], ps_A[127:81]};
    assign next_A[63:0]    = ps_A[63:0]    ^ {ps_A[6:0], ps_A[63:7]} ^ {ps_A[40:0], ps_A[63:41]};

    assign next_B[319:256] = ps_B[319:256] ^ {ps_B[274:256], ps_B[319:275]} ^ {ps_B[283:256], ps_B[319:284]};
    assign next_B[255:192] = ps_B[255:192] ^ {ps_B[252:192], ps_B[255:253]} ^ {ps_B[230:192], ps_B[255:231]};
    assign next_B[191:128] = ps_B[191:128] ^ {ps_B[128], ps_B[191:129]} ^ {ps_B[133:128], ps_B[191:134]};
    assign next_B[127:64]  = ps_B[127:64]  ^ {ps_B[73:64], ps_B[127:74]} ^ {ps_B[80:64], ps_B[127:81]};
    assign next_B[63:0]    = ps_B[63:0]    ^ {ps_B[6:0], ps_B[63:7]} ^ {ps_B[40:0], ps_B[63:41]};

    assign next_C[319:256] = ps_C[319:256] ^ {ps_C[274:256], ps_C[319:275]} ^ {ps_C[283:256], ps_C[319:284]};
    assign next_C[255:192] = ps_C[255:192] ^ {ps_C[252:192], ps_C[255:253]} ^ {ps_C[230:192], ps_C[255:231]};
    assign next_C[191:128] = ps_C[191:128] ^ {ps_C[128], ps_C[191:129]} ^ {ps_C[133:128], ps_C[191:134]};
    assign next_C[127:64]  = ps_C[127:64]  ^ {ps_C[73:64], ps_C[127:74]} ^ {ps_C[80:64], ps_C[127:81]};
    assign next_C[63:0]    = ps_C[63:0]    ^ {ps_C[6:0], ps_C[63:7]} ^ {ps_C[40:0], ps_C[63:41]};

    // Output mapping (Unmasking required only for Data Out and Tag)
    assign data_out = (st_A[319:256] ^ st_B[319:256] ^ st_C[319:256]) ^ data_in;
    
    wire [63:0] tag0 = st_A[127:64] ^ st_B[127:64] ^ st_C[127:64] ^ key_in[127:64];
    wire [63:0] tag1 = st_A[63:0] ^ st_B[63:0] ^ st_C[63:0] ^ key_in[63:0];
    wire [63:0] tag2 = st_A[255:192] ^ st_B[255:192] ^ st_C[255:192];
    wire [63:0] tag3 = st_A[191:128] ^ st_B[191:128] ^ st_C[191:128];
    assign tag_out = {tag0, tag1, tag2, tag3};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state <= ST_IDLE;
            st_A <= 320'h0; st_B <= 320'h0; st_C <= 320'h0;
            round <= 4'd0;
            ready <= 1'b1;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (fsm_state)
                ST_IDLE: begin
                    if (start) begin
                        // Splitting Initial State into 3 Shares using TRNG Entropy
                        st_B <= {192'h0, entropy_128b};
                        st_C <= {192'h0, ~entropy_128b};
                        st_A <= {64'h8040_0c06_0000_0000, key_in, nonce_in} ^ {192'h0, entropy_128b} ^ {192'h0, ~entropy_128b};
                        round <= 4'd0;
                        fsm_state <= ST_INIT;
                        ready <= 1'b0;
                    end else ready <= 1'b1;
                end
                ST_INIT: begin
                    st_A <= next_A; st_B <= next_B; st_C <= next_C;
                    if (round == 4'd11) begin
                        round <= 4'd6; 
                        fsm_state <= ST_PROCESS;
                        st_A[319:256] <= next_A[319:256] ^ data_in; // Inject data into Share A only
                    end else round <= round + 1'b1;
                end
                ST_PROCESS: begin
                    st_A <= next_A; st_B <= next_B; st_C <= next_C;
                    if (round == 4'd11) begin
                        round <= 4'd0;
                        fsm_state <= ST_FINAL;
                        st_A[191:64] <= next_A[191:64] ^ key_in; // Key XOR to Share A
                    end else round <= round + 1'b1;
                end
                ST_FINAL: begin
                    st_A <= next_A; st_B <= next_B; st_C <= next_C;
                    if (round == 4'd11) begin
                        done <= 1'b1;
                        fsm_state <= ST_IDLE;
                    end else round <= round + 1'b1;
                end
            endcase
        end
    end
endmodule