`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/06/2026 02:22:23 PM
// Design Name: 
// Module Name: skid_buffer
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module skid_buffer
#(
    parameter SBUF_TYPE  = 0,
    parameter DATA_WIDTH = 32
)
(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire [DATA_WIDTH-1:0]  bwd_data_i,
    input  wire                   bwd_valid_i,
    input  wire                   fwd_ready_i,
    output wire [DATA_WIDTH-1:0]  fwd_data_o,
    output wire                   bwd_ready_o,
    output wire                   fwd_valid_o
);

    generate
        if (SBUF_TYPE == 0) begin: FULL_REGISTERED
            //Wire
            wire bwd_handshake;
            wire fwd_handshake;
            //Register
            reg [DATA_WIDTH-1:0] bwd_data_d;
            reg [DATA_WIDTH-1:0] fwd_data_d;
            reg bwd_ready_d;
            reg fwd_valid_d;
            reg bwd_ready_q;
            reg fwd_valid_q;
            reg [DATA_WIDTH-1:0] bwd_data_q;
            reg [DATA_WIDTH-1:0] fwd_data_q;

            //Combinational Logic
            assign bwd_handshake = bwd_valid_i && bwd_ready_o;
            assign fwd_handshake = fwd_valid_o && fwd_ready_i;
            assign fwd_data_o = fwd_data_q;
            assign fwd_valid_o = fwd_valid_q;
            assign bwd_ready_o = bwd_ready_q;
            always @(*) begin
                bwd_data_d = bwd_data_q;
                fwd_data_d = fwd_data_q;
                bwd_ready_d = bwd_ready_q;
                fwd_valid_d = fwd_valid_q;
                if (bwd_handshake & fwd_handshake) begin
                    fwd_data_d = bwd_data_i;
                end
                else if(bwd_handshake) begin
                    if(fwd_valid_q) begin
                        bwd_data_d = bwd_data_i;
                        bwd_ready_d = 1'b0; 
                    end
                    else begin
                        fwd_data_d = bwd_data_i;
                        fwd_valid_d = 1'b1;
                    end
                end
                else if(fwd_handshake) begin
                    if(bwd_ready_q) begin
                        fwd_valid_d = 1'b0;
                    end
                    else begin
                        fwd_data_d = bwd_data_q;
                        bwd_ready_d = 1'b1;
                    end
                end
            end
            //Sequential Logic
            //forward path
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    fwd_valid_q <= 1'b0;
                end
                else begin
                    fwd_valid_q <= fwd_valid_d;
                    fwd_data_q <= fwd_data_d;
                end
            end
            //backward path
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    bwd_ready_q <= 1'b1;
                end
                else begin
                    bwd_ready_q <= bwd_ready_d;
                    bwd_data_q <= bwd_data_d;
                end
            end
        end
        else if (SBUF_TYPE == 1) begin: BWD_REGISTERED
            // Type 1: 2-entry FIFO skid buffer (registered bwd_ready_o)
            // bwd_ready_o depends only on registered state — no combinational path from fwd_ready_i
            reg [DATA_WIDTH-1:0] mem [0:1];
            reg [1:0] wr_ptr, rd_ptr;

            wire empty = (wr_ptr == rd_ptr);
            wire full  = (wr_ptr[1] != rd_ptr[1]) && (wr_ptr[0] == rd_ptr[0]);
            wire wr_en = bwd_valid_i & ~full;
            wire rd_en = ~empty & fwd_ready_i;

            assign bwd_ready_o = ~full;
            assign fwd_valid_o = ~empty;
            assign fwd_data_o  = mem[rd_ptr[0]];

            always @(posedge clk) begin
                if (!rst_n) begin
                    wr_ptr <= 2'd0;
                    rd_ptr <= 2'd0;
                end else begin
                    if (wr_en) begin
                        mem[wr_ptr[0]] <= bwd_data_i;
                        wr_ptr <= wr_ptr + 2'd1;
                    end
                    if (rd_en) begin
                        rd_ptr <= rd_ptr + 2'd1;
                    end
                end
            end
        end
        else if (SBUF_TYPE == 2) begin: FULL_SKID
            // Type 2: Full skid buffer (both fwd and bwd registered)
            reg [DATA_WIDTH-1:0] out_data, skid_data;
            reg out_valid, skid_valid;

            assign fwd_data_o  = out_data;
            assign fwd_valid_o = out_valid;
            assign bwd_ready_o = ~skid_valid;

            always @(posedge clk) begin
                if (!rst_n) begin
                    out_valid  <= 1'b0;
                    skid_valid <= 1'b0;
                end else begin
                    if (skid_valid) begin
                        if (fwd_ready_i) begin
                            out_data   <= skid_data;
                            out_valid  <= 1'b1;
                            skid_valid <= 1'b0;
                        end
                    end else begin
                        if (bwd_valid_i) begin
                            if (out_valid && !fwd_ready_i) begin
                                skid_data  <= bwd_data_i;
                                skid_valid <= 1'b1;
                            end else begin
                                out_data  <= bwd_data_i;
                                out_valid <= 1'b1;
                            end
                        end else if (fwd_ready_i) begin
                            out_valid <= 1'b0;
                        end
                    end
                end
            end
        end
        else if (SBUF_TYPE == 3) begin: FWD_REGISTERED
            // Type 3: Forward-registered pipeline register
            // fwd_data_o/fwd_valid_o are registered; bwd_ready_o is combinational
            // Full throughput, 1 cycle latency
            reg [DATA_WIDTH-1:0] data_reg;
            reg valid_reg;

            assign fwd_data_o  = data_reg;
            assign fwd_valid_o = valid_reg;
            assign bwd_ready_o = fwd_ready_i | ~valid_reg;

            always @(posedge clk) begin
                if (!rst_n) begin
                    valid_reg <= 1'b0;
                end else if (bwd_ready_o) begin
                    valid_reg <= bwd_valid_i;
                    data_reg  <= bwd_data_i;
                end
            end
        end
    endgenerate

endmodule
