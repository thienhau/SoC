`timescale 1ns / 1ps

module trng_v1_pure (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         enable,
    output reg  [127:0] rand_out,
    output reg          ready
);
    (* DONT_TOUCH = "TRUE", ALLOW_COMBINATORIAL_LOOPS = "TRUE" *) wire [3:0] ro_clk;
    
    genvar i;
    generate
        for (i=0; i<4; i=i+1) begin : ro_gen
            (* DONT_TOUCH = "TRUE" *) wire w1, w2, w3;
            assign w1 = enable ? ~w3 : 1'b0;
            assign w2 = ~w1;
            assign w3 = ~w2;
            assign ro_clk[i] = w3;
        end
    endgenerate

    reg [3:0] ro_d1, ro_d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ro_d1 <= 4'b0; ro_d2 <= 4'b0;
        end else if (enable) begin
            ro_d1 <= ro_clk;
            ro_d2 <= ro_d1;
        end
    end

    wire [1:0] pair = {ro_d2[0], ro_d1[0]};
    reg [6:0] count;
    reg valid_bit, debiased_bit;

    always @(*) begin
        case (pair)
            2'b01: begin valid_bit = 1'b1; debiased_bit = 1'b0; end
            2'b10: begin valid_bit = 1'b1; debiased_bit = 1'b1; end
            default: begin valid_bit = 1'b0; debiased_bit = 1'b0; end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= 7'd0;
            rand_out <= 128'h0;
            ready <= 1'b0;
        end else if (!enable) begin
            count <= 7'd0;
            ready <= 1'b0;
        end else if (valid_bit && !ready) begin
            rand_out <= {rand_out[126:0], debiased_bit};
            if (count == 7'd127) ready <= 1'b1;
            else count <= count + 1'b1;
        end
    end
endmodule