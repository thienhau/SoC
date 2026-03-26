`timescale 1ns / 1ps

module uart_top #(
    parameter integer DATA_WIDTH     = 8,
    parameter integer INTERNAL_CLOCK = 125_000_000,
    parameter integer BAUD_RATE      = 115_200,
    parameter integer FIFO_DEPTH     = 16,
    parameter integer PARITY_TYPE    = 1,      // 0: none, 1: even, 2: odd
    parameter integer STOP_BITS      = 2       // 1 or 2 stop bits
) (
    // Clock and Reset
    input  wire                     clk,
    input  wire                     rst_n,
    
    // UART Physical Interface
    input  wire                     rxd,
    output wire                     txd,
    
    // CPU Interface - TX
    input  wire [DATA_WIDTH-1:0]    cpu_tx_data,
    input  wire                     cpu_tx_valid,
    output wire                     cpu_tx_ready,
    
    // CPU Interface - RX
    output wire [DATA_WIDTH-1:0]    cpu_rx_data,
    output wire                     cpu_rx_valid,
    input  wire                     cpu_rx_ready,
    
    // Status Indicators
    output wire                     tx_busy,
    output wire                     rx_busy,
    output wire                     tx_fifo_full,
    output wire                     tx_fifo_empty,
    output wire                     rx_fifo_empty,
    output wire                     frame_error,
    output wire                     timeout_error
);

    // =========================================================================
    // INTERNAL SIGNALS
    // =========================================================================
    wire baud_tx_en, baud_rx_en;
    
    // TX Path
    wire [DATA_WIDTH-1:0] tx_fifo_data_out;
    wire tx_fifo_rd_en;
    wire tx_ready;
    wire tx_transaction_en;
    wire tx_fifo_full_int;   // Internal signal from FIFO
    wire tx_fifo_empty_int;  // Internal signal from FIFO
    
    // RX Path
    wire [DATA_WIDTH-1:0] rx_fifo_data_in;
    wire rx_fifo_wr_en;
    wire rx_fifo_full;
    wire rx_transaction_en;
    wire rx_frame_error;
    wire rx_timeout_error;
    wire rx_fifo_empty_int;  // Internal signal from FIFO
    
    // BAUD RATE GENERATOR
    baud_gen #(
        .INTERNAL_CLOCK (INTERNAL_CLOCK),
        .BAUD_RATE      (BAUD_RATE)
    ) baud_gen_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .baud_tx_en (baud_tx_en),
        .baud_rx_en (baud_rx_en)
    );
    
    // TX PATH (CPU → FIFO → UART TX)
    asyn_fifo #(
        .ASFIFO_TYPE      (0),                    // Normal FIFO (no registered delay)
        .DATA_WIDTH       (DATA_WIDTH),
        .FIFO_DEPTH       (FIFO_DEPTH),
        .NUM_SYNC_FF      (0),                    // Same clock - bypass synchronizer
        .ALMOST_FULL_TH   (FIFO_DEPTH - 2),
        .ALMOST_EMPTY_TH  (2)
    ) fifo_tx (
        .clk_wr_domain    (clk),
        .clk_rd_domain    (clk),
        .data_i           (cpu_tx_data),
        .data_o           (tx_fifo_data_out),
        .wr_valid_i       (cpu_tx_valid),
        .rd_valid_i       (tx_fifo_rd_en),
        .empty_o          (tx_fifo_empty_int),
        .full_o           (tx_fifo_full_int),
        .wr_ready_o       (cpu_tx_ready),
        .rd_ready_o       (),
        .almost_empty_o   (),
        .almost_full_o    (),
        .rst_n            (rst_n)
    );
    
    uart_tx #(
        .DATA_WIDTH   (DATA_WIDTH),
        .PARITY_TYPE  (PARITY_TYPE),
        .STOP_BITS    (STOP_BITS)
    ) uart_tx_inst (
        .clk               (clk),
        .rst_n             (rst_n),
        .data_i            (tx_fifo_data_out),
        .data_valid_i      (tx_fifo_rd_en),
        .ready_o           (tx_ready),
        .baudrate_clk_en   (baud_tx_en),      // FIX: Dùng baud_tx_en (1x)
        .transaction_en    (tx_transaction_en),
        .TX                (txd),
        .tx_busy           (tx_busy)
    );
    
    assign tx_fifo_rd_en = tx_ready && !tx_fifo_empty_int;
    
    // RX PATH (UART RX → FIFO → CPU)
    uart_rx #(
        .DATA_WIDTH     (DATA_WIDTH),
        .TIMEOUT_CYCLES (16 * 20)
    ) uart_rx_inst (
        .clk              (clk),
        .rst_n            (rst_n),
        .RX               (rxd),
        .baudrate_clk_en  (baud_rx_en),
        .transaction_en   (rx_transaction_en),
        .data_out_rx      (rx_fifo_data_in),
        .fifo_wr          (rx_fifo_wr_en),
        .frame_error      (rx_frame_error),
        .timeout_error    (rx_timeout_error)
    );
    
    asyn_fifo #(
        .ASFIFO_TYPE      (0),                    // Normal FIFO (no registered delay)
        .DATA_WIDTH       (DATA_WIDTH),
        .FIFO_DEPTH       (FIFO_DEPTH),
        .NUM_SYNC_FF      (0),                    // Same clock - bypass synchronizer
        .ALMOST_FULL_TH   (FIFO_DEPTH - 2),
        .ALMOST_EMPTY_TH  (2)
    ) fifo_rx (
        .clk_wr_domain    (clk),
        .clk_rd_domain    (clk),
        .data_i           (rx_fifo_data_in),
        .data_o           (cpu_rx_data),
        .wr_valid_i       (rx_fifo_wr_en),
        .rd_valid_i       (cpu_rx_ready),
        .empty_o          (rx_fifo_empty_int),
        .full_o           (rx_fifo_full),
        .wr_ready_o       (),
        .rd_ready_o       (cpu_rx_valid),
        .almost_empty_o   (),
        .almost_full_o    (),
        .rst_n            (rst_n)
    );
    
    // STATUS SIGNALS
    assign rx_busy         = rx_transaction_en;
    assign frame_error     = rx_frame_error;
    assign timeout_error   = rx_timeout_error;
    assign tx_fifo_full    = tx_fifo_full_int;
    assign tx_fifo_empty   = tx_fifo_empty_int;
    assign rx_fifo_empty   = rx_fifo_empty_int;

endmodule