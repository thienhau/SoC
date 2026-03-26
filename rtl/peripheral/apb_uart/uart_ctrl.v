`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/26/2025 12:13:43 PM
// Design Name: 
// Module Name: uart_ctrl
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


module uart_ctrl#(
    parameter DATA_WIDTH = 8,
    parameter DATA_W = 32,
    parameter ADDR_W = 32,
    parameter INTERNAL_CLOCK = 125000000,
    parameter BAUD_RATE = 115200,
    parameter FIFO_DEPTH = 16
)(
    input clk,
    input rst_n,
    // UART Physical Interface
    input rxd,
    output txd,
    // AXI4-Lite Interface
    input  [ADDR_W-1:0]   s_axi_awaddr,
    input  [2:0]          s_axi_awprot,
    input                 s_axi_awvalid,
    output                s_axi_awready,
    input  [DATA_W-1:0]   s_axi_wdata,
    input  [(DATA_W/8)-1:0] s_axi_wstrb,
    input                 s_axi_wvalid,
    output                s_axi_wready,
    output [1:0]          s_axi_bresp,
    output                s_axi_bvalid,
    input                 s_axi_bready,
    input  [ADDR_W-1:0]   s_axi_araddr,
    input  [2:0]          s_axi_arprot,
    input                 s_axi_arvalid,
    output                s_axi_arready,
    output [DATA_W-1:0]   s_axi_rdata,
    output [1:0]          s_axi_rresp,
    output                s_axi_rvalid,
    input                 s_axi_rready
);
    // STREAM TX (AXI to UART)
    wire [DATA_WIDTH-1:0] tx_stream_data;
    wire                  tx_stream_valid;
    wire                  tx_stream_ready;
    
    // STREAM RX (UART to AXI)
    wire [DATA_WIDTH-1:0] rx_stream_data;
    wire                  rx_stream_valid;
    wire                  rx_stream_ready;

    // Status signals from UART
    wire tx_fifo_full;
    wire rx_fifo_empty;
    wire tx_busy;
    wire rx_busy;
    wire frame_error;
    wire timeout_error;
    
    // Status register for AXI
    wire [DATA_W-1:0] stat_reg_wire;
    assign stat_reg_wire = {
        24'b0,
        timeout_error,   // bit 7
        frame_error,     // bit 6
        rx_busy,         // bit 5
        tx_busy,         // bit 4
        2'b0,
        ~rx_stream_valid, // bit 1: 1 = no data available, 0 = data available
        tx_fifo_full     // bit 0
    };
    
    // AXI stream signals
    wire                  wr_st_can_accept;
    wire                  rd_st_has_data;
    wire                  wr_st_push;
    wire                  rd_st_pop;
    
    assign wr_st_can_accept = tx_stream_ready;
    assign rd_st_has_data = rx_stream_valid;

    // AXI4-Lite Controller Module
    axi4lite_ctrl #(
        .AXI4_CTRL_CONF(0),     // No configuration registers
        .AXI4_CTRL_STAT(1),     // Enable status register
        .AXI4_CTRL_MEM(0),      // No memory-mapped
        .AXI4_CTRL_WR_ST(1),    // Write Stream enabled
        .AXI4_CTRL_RD_ST(1),    // Read Stream enabled
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .ST_WR_FIFO_NUM(1),
        .ST_RD_FIFO_NUM(1)
    ) u_axi_ctrl (
        .clk(clk),
        .rst_n(rst_n),
        
        // AXI4-Lite Slave Interface
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_arprot(s_axi_arprot),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),
        
        // Status Interface
        .stat_reg_i(stat_reg_wire),
        
        // External Interfaces (unused)
        .mem_wr_rdy_i(1'b0),
        .mem_rd_data_i({DATA_W{1'b0}}),
        .mem_rd_rdy_i(1'b0),
        .conf_reg_o(),
        .mem_wr_data_o(),
        .mem_wr_addr_o(),
        .mem_wr_strb_o(),
        .mem_wr_vld_o(),
        .mem_rd_addr_o(),
        .mem_rd_vld_o(),
        
        // Write Stream Interface (AXI to UART TX)
        .wr_st_can_accept_i(wr_st_can_accept),
        .wr_st_data_o(tx_stream_data[DATA_WIDTH-1:0]),
        .wr_st_push_o(wr_st_push),
        
        // Read Stream Interface (UART RX to AXI)
        .rd_st_data_i({{(DATA_W-DATA_WIDTH){1'b0}}, rx_stream_data}),
        .rd_st_has_data_i(rd_st_has_data),
        .rd_st_pop_o(rd_st_pop)
    );
    
    // Convert AXI stream signals to UART interface signals
    assign tx_stream_valid = wr_st_push;
    assign rx_stream_ready = rd_st_pop;
    
    // Debug monitoring - synchronous to catch pulses


    // UART Top Module
    uart_top #(
        .DATA_WIDTH(DATA_WIDTH),
        .INTERNAL_CLOCK(INTERNAL_CLOCK),
        .BAUD_RATE(BAUD_RATE),
        .FIFO_DEPTH(FIFO_DEPTH)
    ) u_uart_top (
        .clk(clk),
        .rst_n(rst_n),
        
        // UART Physical Interface
        .rxd(rxd),
        .txd(txd),
        
        // CPU/Stream Interface - TX
        .cpu_tx_data(tx_stream_data),
        .cpu_tx_valid(tx_stream_valid),
        .cpu_tx_ready(tx_stream_ready),
        
        // CPU/Stream Interface - RX
        .cpu_rx_data(rx_stream_data),
        .cpu_rx_valid(rx_stream_valid),
        .cpu_rx_ready(rx_stream_ready),
        
        // Status Indicators
        .tx_busy(tx_busy),
        .rx_busy(rx_busy),
        .tx_fifo_full(tx_fifo_full),
        .rx_fifo_empty(rx_fifo_empty),
        .frame_error(frame_error),
        .timeout_error(timeout_error)
    );
    
endmodule
