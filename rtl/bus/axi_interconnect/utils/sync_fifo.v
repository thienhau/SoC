module sync_fifo
#(
    parameter FIFO_TYPE  = 0,     // 0: Standard, 1: FWFT, 2: Full flop (NOTE: not yet implemented, all types use combinational read)
    parameter DATA_WIDTH = 8,
    parameter FIFO_DEPTH = 32,
    parameter ADDR_WIDTH = $clog2(FIFO_DEPTH)
)
(
    input                       clk,
    input                       rst_n,

    input  [DATA_WIDTH-1:0]     data_i,
    output [DATA_WIDTH-1:0]     data_o,

    input                       wr_valid_i,
    input                       rd_valid_i,

    output                      empty_o,
    output                      full_o,
    output                      wr_ready_o,
    output                      rd_ready_o,
    output                      almost_empty_o,
    output                      almost_full_o,
    output [ADDR_WIDTH:0]       counter
);
    // Internal signals
    reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];
    reg [ADDR_WIDTH:0]   wr_ptr, rd_ptr;

    wire [ADDR_WIDTH-1:0] wr_addr = wr_ptr[ADDR_WIDTH-1:0];
    wire [ADDR_WIDTH-1:0] rd_addr = rd_ptr[ADDR_WIDTH-1:0];

    wire wr_en = wr_valid_i & ~full_o;
    wire rd_en = rd_valid_i & ~empty_o;

    // Status flags
    assign empty_o        = (wr_ptr == rd_ptr);
    assign full_o         = (wr_addr == rd_addr) && (wr_ptr[ADDR_WIDTH] != rd_ptr[ADDR_WIDTH]);
    assign almost_empty_o = (rd_ptr + 1'b1) == wr_ptr;
    assign almost_full_o  = (wr_addr + 1'b1) == rd_addr;
    assign counter        = wr_ptr - rd_ptr;
    assign wr_ready_o     = ~full_o;
    assign rd_ready_o     = ~empty_o;

    // Output
    assign data_o = mem[rd_addr];

    // Write logic
    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= {(ADDR_WIDTH+1){1'b0}};
        end else if (wr_en) begin
            mem[wr_addr] <= data_i;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    // Read logic
    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr <= {(ADDR_WIDTH+1){1'b0}};
        end else if (rd_en) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

endmodule
