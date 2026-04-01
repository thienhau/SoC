`timescale 1ns / 1ps

module apb_spi #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter FIFO_DEPTH = 16
)(
    input  wire pclk, presetn, psel, penable, pwrite,
    input  wire [ADDR_WIDTH-1:0] paddr,
    input  wire [DATA_WIDTH-1:0] pwdata,
    input  wire [3:0] pstrb,
    output reg  pready, 
    output reg  [DATA_WIDTH-1:0] prdata, 
    output reg  pslverr,

    // SPI Physical Pins
    output wire sclk,
    output wire mosi,
    input  wire miso,
    output reg  cs_n,
    
    // Interrupt & DMA
    output wire spi_irq,
    output wire dma_tx_req,
    output wire dma_rx_req
);

    // --- Register Map ---
    // 0x00: CTRL     [1] CPOL, [0] CPHA
    // 0x04: CS       [0] Chip Select 0
    // 0x08: DIV      SCLK Divider
    // 0x0C: TXDATA   (Ghi vào TX FIFO)
    // 0x10: RXDATA   (Đọc từ RX FIFO)
    // 0x14: STATUS   [3] RX_EMPTY, [2] TX_FULL, [1] DONE, [0] BUSY
    // 0x18: DMA_INT  [3] RX_DMA_EN, [2] TX_DMA_EN, [1] RX_INT_EN, [0] TX_INT_EN

    reg [31:0] reg_div;
    reg [1:0]  reg_ctrl;
    reg [3:0]  reg_dma_int;
    
    wire spi_busy;
    reg  spi_done;

    // --- FIFO Signals ---
    wire tx_fifo_full, tx_fifo_empty;
    wire rx_fifo_full, rx_fifo_empty;
    wire [7:0] tx_fifo_rdata;
    wire [7:0] rx_fifo_rdata;
    
    reg  tx_fifo_wr, tx_fifo_rd;
    reg  rx_fifo_wr, rx_fifo_rd;

    // --- CDC MISO ---
    reg miso_sync1, miso_sync2;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) {miso_sync2, miso_sync1} <= 2'b00;
        else          {miso_sync2, miso_sync1} <= {miso_sync1, miso};
    end

    // ==========================================
    // 1. APB INTERFACE & FIFO CTRL
    // ==========================================
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_ctrl <= 0; reg_div <= 2; cs_n <= 1;
            reg_dma_int <= 0; spi_done <= 0;
            pready <= 0; prdata <= 0; pslverr <= 0;
            tx_fifo_wr <= 0; rx_fifo_rd <= 0;
        end else begin
            pready <= psel && penable;
            pslverr <= 0;
            tx_fifo_wr <= 0;
            rx_fifo_rd <= 0;

            if (psel && penable && pwrite) begin
                case (paddr[11:0])
                    12'h000: reg_ctrl <= pwdata[1:0];
                    12'h004: cs_n <= pwdata[0];
                    12'h008: reg_div <= pwdata;
                    12'h00C: begin
                        if (!tx_fifo_full) tx_fifo_wr <= 1;
                    end
                    12'h014: if (pwdata[1]) spi_done <= 0;
                    12'h018: reg_dma_int <= pwdata[3:0];
                    default: ;
                endcase
            end
            
            if (psel && !penable && !pwrite) begin
                case (paddr[11:0])
                    12'h000: prdata <= {30'b0, reg_ctrl};
                    12'h004: prdata <= {31'b0, cs_n};
                    12'h008: prdata <= reg_div;
                    12'h010: begin 
                        prdata <= {24'b0, rx_fifo_rdata}; 
                        if (!rx_fifo_empty) rx_fifo_rd <= 1;
                    end
                    12'h014: prdata <= {28'b0, rx_fifo_empty, tx_fifo_full, spi_done, spi_busy};
                    12'h018: prdata <= {28'b0, reg_dma_int};
                    default: prdata <= 32'b0;
                endcase
            end
        end
    end

    // ==========================================
    // 2. DMA & IRQ ASSIGNMENTS
    // ==========================================
    assign dma_tx_req = (!tx_fifo_full)  & reg_dma_int[2];
    assign dma_rx_req = (!rx_fifo_empty) & reg_dma_int[3];
    assign spi_irq    = (spi_done & reg_dma_int[0]) | (!rx_fifo_empty & reg_dma_int[1]);

    // ==========================================
    // 3. SPI CORE FSM
    // ==========================================
    localparam IDLE = 0, PHASE1 = 1, PHASE2 = 2;
    reg [2:0]  state;
    reg [31:0] clk_cnt;
    reg [2:0]  bit_cnt;
    reg [7:0]  spi_tx_shift;
    reg [7:0]  spi_rx_shift;
    reg        sclk_reg;
    
    wire cpol = reg_ctrl[1];
    wire cpha = reg_ctrl[0];
    
    assign sclk = (state == IDLE) ? cpol : sclk_reg;
    assign mosi = spi_tx_shift[7];
    assign spi_busy = (state != IDLE) || (!tx_fifo_empty);

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            state <= IDLE; clk_cnt <= 0; bit_cnt <= 0;
            sclk_reg <= 0; spi_tx_shift <= 0; spi_rx_shift <= 0;
            tx_fifo_rd <= 0; rx_fifo_wr <= 0;
        end else begin
            tx_fifo_rd <= 0;
            rx_fifo_wr <= 0;

            case (state)
                IDLE: begin
                    sclk_reg <= cpol;
                    if (!tx_fifo_empty && !tx_fifo_rd) begin
                        spi_tx_shift <= tx_fifo_rdata;
                        tx_fifo_rd   <= 1;
                        clk_cnt      <= 0;
                        bit_cnt      <= 0;
                        state        <= PHASE1;
                    end
                end
                
                PHASE1: begin 
                    if (clk_cnt == reg_div) begin
                        clk_cnt <= 0;
                        sclk_reg <= ~sclk_reg;
                        state <= PHASE2;
                        if (cpha == 0) spi_rx_shift <= {spi_rx_shift[6:0], miso_sync2};
                    end else clk_cnt <= clk_cnt + 1;
                end
                
                PHASE2: begin 
                    if (clk_cnt == reg_div) begin
                        clk_cnt <= 0;
                        sclk_reg <= ~sclk_reg;
                        if (cpha == 1) spi_rx_shift <= {spi_rx_shift[6:0], miso_sync2};
                        
                        if (bit_cnt == 7) begin
                            rx_fifo_wr <= 1;
                            if (tx_fifo_empty) spi_done <= 1;
                            state <= IDLE;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                            spi_tx_shift <= {spi_tx_shift[6:0], 1'b0};
                            state <= PHASE1;
                        end
                    end else clk_cnt <= clk_cnt + 1;
                end
            endcase
        end
    end

    // ==========================================
    // 4. FIFO INSTANTIATIONS
    // ==========================================
    sync_fifo #(
        .FIFO_TYPE(0), 
        .DATA_WIDTH(8), 
        .FIFO_DEPTH(FIFO_DEPTH)
    ) u_tx_fifo (
        .clk            (pclk), 
        .rst_n          (presetn),
        .data_i         (pwdata[7:0]), 
        .data_o         (tx_fifo_rdata),
        .wr_valid_i     (tx_fifo_wr), 
        .rd_valid_i     (tx_fifo_rd),
        .empty_o        (tx_fifo_empty), 
        .full_o         (tx_fifo_full),
        .wr_ready_o     (),
        .rd_ready_o     (),
        .almost_empty_o (),
        .almost_full_o  (),
        .counter        ()
    );

    sync_fifo #(
        .FIFO_TYPE(0), 
        .DATA_WIDTH(8), 
        .FIFO_DEPTH(FIFO_DEPTH)
    ) u_rx_fifo (
        .clk            (pclk), 
        .rst_n          (presetn),
        .data_i         (spi_rx_shift), 
        .data_o         (rx_fifo_rdata),
        .wr_valid_i     (rx_fifo_wr), 
        .rd_valid_i     (rx_fifo_rd),
        .empty_o        (rx_fifo_empty), 
        .full_o         (rx_fifo_full),
        .wr_ready_o     (),
        .rd_ready_o     (),
        .almost_empty_o (),
        .almost_full_o  (),
        .counter        ()
    );

endmodule