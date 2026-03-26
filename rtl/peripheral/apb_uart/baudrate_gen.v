`timescale 1ns / 1ps

module baud_gen #(
    parameter integer INTERNAL_CLOCK = 125_000_000,  // 125 MHz
    parameter integer BAUD_RATE      = 115_200
) (
    input  wire clk,
    input  wire rst_n,
    output reg  baud_tx_en,
    output reg  baud_rx_en
);

    localparam real INTERNAL_CLOCK_REAL = 125_000_000.0;
    localparam real BAUD_RATE_REAL      = 115_200.0;
    
    localparam integer MIN_DIVISOR = 2;
    
    // CORRECTED: TX uses 1x baud rate, RX uses 16x oversampling
    // TX_DIVISOR = CLOCK / BAUD (1x for direct bit timing)
    // RX_DIVISOR = CLOCK / (BAUD * 16) (16x for sampling)
    localparam integer TX_DIVISOR_REAL = $rtoi((INTERNAL_CLOCK_REAL / BAUD_RATE_REAL) + 0.5);
    localparam integer TX_DIVISOR = (TX_DIVISOR_REAL > MIN_DIVISOR) ? TX_DIVISOR_REAL : MIN_DIVISOR;
    
    localparam integer RX_DIVISOR_REAL = $rtoi((INTERNAL_CLOCK_REAL / (BAUD_RATE_REAL * 16.0)) + 0.5);
    localparam integer RX_DIVISOR = (RX_DIVISOR_REAL > MIN_DIVISOR) ? RX_DIVISOR_REAL : MIN_DIVISOR;
    
    // Debug: Hiển thị giá trị thực
    localparam real ACTUAL_TX_BAUD = INTERNAL_CLOCK_REAL / TX_DIVISOR;  // 1x baud rate
    localparam real ACTUAL_RX_BAUD = INTERNAL_CLOCK_REAL / (RX_DIVISOR * 16);
    
    // Counter width
    localparam integer MAX_DIVISOR = (TX_DIVISOR > RX_DIVISOR) ? TX_DIVISOR : RX_DIVISOR;
    localparam integer COUNTER_WIDTH = $clog2(MAX_DIVISOR) + 1;
    
    reg [COUNTER_WIDTH-1:0] tx_counter = 0;
    reg [COUNTER_WIDTH-1:0] rx_counter = 0;
    
    // TX BAUD GENERATOR
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_counter <= 0;
            baud_tx_en <= 1'b0;
        end else begin
            baud_tx_en <= 1'b0;
            
            if (tx_counter >= TX_DIVISOR - 1) begin
                tx_counter <= 0;
                baud_tx_en <= 1'b1;
            end else begin
                tx_counter <= tx_counter + 1;
            end
        end
    end
    
    // RX BAUD GENERATOR (16x oversampling)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_counter <= 0;
            baud_rx_en <= 1'b0;
        end else begin
            baud_rx_en <= 1'b0;
            
            if (rx_counter >= RX_DIVISOR - 1) begin
                rx_counter <= 0;
                baud_rx_en <= 1'b1;
            end else begin
                rx_counter <= rx_counter + 1;
            end
        end
    end
    
    // DEBUG INFO
    initial begin
        $display("[BAUD_GEN] INTERNAL_CLOCK = %0d Hz", INTERNAL_CLOCK);
        $display("[BAUD_GEN] BAUD_RATE = %0d bps", BAUD_RATE);
        $display("[BAUD_GEN] TX_DIVISOR = %0d", TX_DIVISOR);
        $display("[BAUD_GEN] RX_DIVISOR = %0d", RX_DIVISOR);
        $display("[BAUD_GEN] Actual TX Baud = %0.1f bps", ACTUAL_TX_BAUD);
        $display("[BAUD_GEN] Actual RX Baud = %0.1f bps", ACTUAL_RX_BAUD);
        $display("[BAUD_GEN] TX Period = %0.1f ns", (1000000000.0 / ACTUAL_TX_BAUD));
        $display("[BAUD_GEN] RX Period = %0.1f ns", (RX_DIVISOR * 8.0));
    end

endmodule