`timescale 1ns / 1ps

module ascon_core (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    input  wire         mode,
    input  wire [127:0] key_in,
    input  wire [127:0] nonce_in,
    input  wire [63:0]  data_in,
    input  wire         data_valid, 
    input  wire [2:0]   data_cmd,   
    
    output wire [63:0]  data_out,
    output reg  [127:0] tag_out,
    output reg  [255:0] hash_out,
    output reg          ready,
    output reg          done
);

    localparam IV_AEAD = 64'h80400c0600000000; 
    localparam IV_HASH = 64'h00400c0000000100;

    localparam CMD_AD        = 3'd0;
    localparam CMD_AD_LAST   = 3'd1;
    localparam CMD_PT        = 3'd2;
    localparam CMD_PT_LAST   = 3'd3;
    localparam CMD_HASH      = 3'd4;
    localparam CMD_HASH_LAST = 3'd5;
    localparam CMD_EMPTY_AD  = 3'd6;

    localparam ST_IDLE       = 3'd0;
    localparam ST_INIT       = 3'd1;
    localparam ST_WAIT_DATA  = 3'd2;
    localparam ST_PROCESS    = 3'd3;
    localparam ST_SQUEEZE    = 3'd4;

    reg [319:0] state;
    reg [3:0]   round;
    reg [2:0]   fsm_state;
    reg [1:0]   squeeze_cnt;
    reg [2:0]   latched_cmd;

    wire [63:0] x0 = state[319:256];
    wire [63:0] x1 = state[255:192];
    wire [63:0] x2 = state[191:128];
    wire [63:0] x3 = state[127:64];
    wire [63:0] x4 = state[63:0];

    // pC
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

    // pS
    wire [63:0] x0_ps, x1_ps, x2_ps, x3_ps, x4_ps;
    genvar i;
    generate
        for (i=0; i<64; i=i+1) begin : gen_sbox
            wire t0 = x0[i] ^ x4[i]; 
            wire t1 = x1[i]; 
            wire t2 = x2_pc[i] ^ t1; 
            wire t3 = x3[i]; 
            wire t4 = x4[i] ^ t3;
            wire t0_a = t0 ^ ((~t1) & t2); 
            wire t1_a = t1 ^ ((~t2) & t3); 
            wire t2_a = t2 ^ ((~t3) & t4); 
            wire t3_a = t3 ^ ((~t4) & t0); 
            wire t4_a = t4 ^ ((~t0) & t1);
            assign x0_ps[i] = t0_a ^ t4_a; 
            assign x1_ps[i] = t1_a ^ t0_a; 
            assign x2_ps[i] = ~t2_a; 
            assign x3_ps[i] = t3_a ^ t2_a; 
            assign x4_ps[i] = t4_a;
        end
    endgenerate

    // pL
    wire [63:0] x0_pl = x0_ps ^ {x0_ps[18:0], x0_ps[63:19]} ^ {x0_ps[27:0], x0_ps[63:28]};
    wire [63:0] x1_pl = x1_ps ^ {x1_ps[60:0], x1_ps[63:61]} ^ {x1_ps[38:0], x1_ps[63:39]};
    wire [63:0] x2_pl = x2_ps ^ {x2_ps[0], x2_ps[63:1]}   ^ {x2_ps[5:0], x2_ps[63:6]};
    wire [63:0] x3_pl = x3_ps ^ {x3_ps[9:0], x3_ps[63:10]}  ^ {x3_ps[16:0], x3_ps[63:17]};
    wire [63:0] x4_pl = x4_ps ^ {x4_ps[6:0], x4_ps[63:7]}   ^ {x4_ps[40:0], x4_ps[63:41]};

    wire [319:0] next_state = {x0_pl, x1_pl, x2_pl, x3_pl, x4_pl};

    assign data_out = x0 ^ data_in; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state   <= ST_IDLE;
            state       <= 320'h0;
            round       <= 4'd0;
            squeeze_cnt <= 2'd0;
            latched_cmd <= 3'd0;
            ready       <= 1'b1;
            done        <= 1'b0;
            hash_out    <= 256'h0;
            tag_out     <= 128'h0;
        end else begin
            done <= 1'b0; 
            case (fsm_state)
                ST_IDLE: begin
                    if (start) begin
                        fsm_state <= ST_INIT;
                        ready <= 1'b0;
                        round <= 4'd0;
                        if (mode == 1'b0) state <= {IV_AEAD, key_in, nonce_in};
                        else              state <= {IV_HASH, 256'h0};
                    end else begin
                        ready <= 1'b1;
                    end
                end

                ST_INIT: begin
                    if (round == 4'd11) begin
                        if (mode == 1'b0) begin
                            state[319:128] <= next_state[319:128];
                            state[127:0]   <= next_state[127:0] ^ key_in;
                        end else begin
                            state <= next_state;
                        end
                        fsm_state <= ST_WAIT_DATA;
                        ready <= 1'b1;
                    end else begin
                        state <= next_state;
                        round <= round + 1'b1;
                    end
                end

                ST_WAIT_DATA: begin
                    if (data_valid) begin
                        if (data_cmd == CMD_EMPTY_AD) begin
                            state <= {state[319:1], state[0] ^ 1'b1};
                            ready <= 1'b1;
                        end else begin
                            ready <= 1'b0;
                            latched_cmd <= data_cmd;
                            
                            state[319:256] <= state[319:256] ^ data_in;
                            
                            if (data_cmd == CMD_PT_LAST) state[255:128] <= state[255:128] ^ key_in;
                            else                         state[255:128] <= state[255:128];

                            state[127:0] <= state[127:0];
                            
                            // VÁ LỖI: Điểm bắt đầu vòng lặp
                            if (data_cmd == CMD_PT_LAST || data_cmd == CMD_HASH || data_cmd == CMD_HASH_LAST) begin
                                 round <= 4'd0; // pa
                            end else begin
                                 round <= 4'd6; // pb
                            end
                            
                            fsm_state <= ST_PROCESS;
                        end
                    end else begin
                        ready <= 1'b1;
                    end
                end

                ST_PROCESS: begin
                    // VÁ LỖI: Điểm kết thúc CỐ ĐỊNH là vòng số 11
                    if (round == 4'd11) begin 
                        if (latched_cmd == CMD_PT_LAST) begin
                            tag_out <= {next_state[127:64] ^ key_in[127:64], next_state[63:0] ^ key_in[63:0]};
                            done <= 1'b1;
                            ready <= 1'b1;
                            fsm_state <= ST_IDLE;
                            state <= next_state;
                        end
                        else if (latched_cmd == CMD_HASH_LAST) begin
                            hash_out[255:192] <= next_state[319:256];
                            squeeze_cnt <= 2'd1;
                            round <= 4'd0;
                            state <= next_state;
                            fsm_state <= ST_SQUEEZE;
                        end
                        else begin
                            ready <= 1'b1;
                            if (latched_cmd == CMD_AD_LAST) begin
                                state <= {next_state[319:1], next_state[0] ^ 1'b1}; 
                            end else begin
                                state <= next_state;
                            end
                            fsm_state <= ST_WAIT_DATA;
                        end
                    end else begin
                        state <= next_state;
                        round <= round + 1'b1;
                    end
                end

                ST_SQUEEZE: begin
                    if (round == 4'd11) begin
                        state <= next_state;
                        if (squeeze_cnt == 2'd1) hash_out[191:128] <= next_state[319:256];
                        if (squeeze_cnt == 2'd2) hash_out[127:64]  <= next_state[319:256];
                        
                        if (squeeze_cnt == 2'd3) begin
                            hash_out[63:0] <= next_state[319:256];
                            done <= 1'b1;
                            ready <= 1'b1;
                            fsm_state <= ST_IDLE;
                        end else begin
                            squeeze_cnt <= squeeze_cnt + 1'b1;
                            round <= 4'd0;
                        end
                    end else begin
                        state <= next_state;
                        round <= round + 1'b1;
                    end
                end
            endcase
        end
    end
endmodule