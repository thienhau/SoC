`timescale 1ns / 1ps

module trng_v2_hybrid_prng (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         enable,
    output wire [127:0] rand_out,
    output reg          ready
);
    wire [127:0] pure_seed;
    wire pure_ready;
    
    trng_v1_pure_ro u_seed_gen (
        .clk(clk), .rst_n(rst_n), .enable(enable & ~pure_ready),
        .rand_out(pure_seed), .ready(pure_ready)
    );

    reg [127:0] lfsr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr <= 128'h0;
            ready <= 1'b0;
        end else if (pure_ready && !ready) begin
            lfsr <= pure_seed; 
            ready <= 1'b1;
        end else if (ready && enable) begin
            // X^128 + X^126 + X^101 + X^99 + 1 (Galois LFSR)
            lfsr <= {lfsr[126:0], 1'b0} ^ 
                    (lfsr[127] ? 128'h80000000000000000000000000000087 : 128'h0);
        end
    end
    assign rand_out = lfsr;
endmodule