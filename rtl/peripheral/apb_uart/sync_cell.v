`timescale 1ns / 1ps

module sync_cell #(
    parameter integer WIDTH      = 8,
    parameter integer NUM_STAGES = 2    // 2 or 3 stages
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [WIDTH-1:0]     async_i,
    output wire [WIDTH-1:0]     sync_o
);

    // SYNCHRONIZATION REGISTERS
    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] sync_ff_0;
    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] sync_ff_1;
    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] sync_ff_2;
    
    // Vivado attribute for better metastability handling
    (* DONT_TOUCH = "TRUE" *) wire [WIDTH-1:0] sync_o_reg;

    // 2-STAGE SYNCHRONIZER (Default)
    generate
        if (NUM_STAGES == 2) begin : SYNC_2STAGE
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    sync_ff_0 <= {WIDTH{1'b0}};
                    sync_ff_1 <= {WIDTH{1'b0}};
                end else begin
                    sync_ff_0 <= async_i;
                    sync_ff_1 <= sync_ff_0;
                end
            end
            assign sync_o_reg = sync_ff_1;
        end
        // 3-STAGE SYNCHRONIZER (Higher reliability)
        else if (NUM_STAGES == 3) begin : SYNC_3STAGE
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    sync_ff_0 <= {WIDTH{1'b0}};
                    sync_ff_1 <= {WIDTH{1'b0}};
                    sync_ff_2 <= {WIDTH{1'b0}};
                end else begin
                    sync_ff_0 <= async_i;
                    sync_ff_1 <= sync_ff_0;
                    sync_ff_2 <= sync_ff_1;
                end
            end
            assign sync_o_reg = sync_ff_2;
        end
    endgenerate
    
    assign sync_o = sync_o_reg;

endmodule