`timescale 1ns / 1ps

module apb_i2c #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter FIFO_DEPTH = 16
)(
    input wire pclk, presetn, psel, penable, pwrite,
    input wire [ADDR_WIDTH-1:0] paddr,
    input wire [DATA_WIDTH-1:0] pwdata,
    input wire [3:0] pstrb,
    output reg pready, 
    output reg [DATA_WIDTH-1:0] prdata, 
    output reg pslverr,

    // I2C Open-Drain Interface
    output wire scl_o, scl_oen,
    input  wire scl_i,
    output wire sda_o, sda_oen,
    input  wire sda_i,
    
    // Interrupt & DMA
    output wire i2c_irq,
    output wire dma_tx_req,
    output wire dma_rx_req
);

    // --- Register Map ---
    // 0x00: PRER    Prescaler
    // 0x04: TXDATA  (Ghi vào TX FIFO)
    // 0x08: RXDATA  (Đọc từ RX FIFO)
    // 0x0C: CMD     [7] STA, [6] STO, [5] RD, [4] WR, [3] ACK (gửi đi)
    // 0x10: STAT    [7] RxACK, [6] BUSY, [3] RX_EMPTY, [2] TX_FULL, [1] Transferring, [0] IRQ
    // 0x14: DMA_INT [3] RX_DMA_EN, [2] TX_DMA_EN, [1] RX_INT_EN, [0] DONE_INT_EN

    reg [15:0] reg_prer;
    reg [7:0]  reg_cmd;
    reg [7:0]  reg_stat;
    reg [3:0]  reg_dma_int;

    // --- FIFO Signals ---
    wire tx_fifo_full, tx_fifo_empty;
    wire rx_fifo_full, rx_fifo_empty;
    wire [7:0] tx_fifo_rdata;
    wire [7:0] rx_fifo_rdata;
    
    reg  tx_fifo_wr, tx_fifo_rd;
    reg  rx_fifo_wr, rx_fifo_rd;

    reg [4:0]  state;
    reg [15:0] tick_cnt;
    reg [2:0]  bit_cnt;
    reg [7:0]  shift_reg;
    reg        core_done;
    reg        core_rx_ack;
    reg [7:0]  core_rx_data;

    localparam S_IDLE=0, S_START_A=1, S_START_B=2, S_START_C=3;
    localparam S_BIT_A=4, S_BIT_B=5, S_BIT_C=6, S_BIT_D=7;
    localparam S_ACK_A=8, S_ACK_B=9, S_ACK_C=10, S_ACK_D=11;
    localparam S_STOP_A=12, S_STOP_B=13, S_STOP_C=14;

    // --- CDC cho SCL/SDA Input ---
    reg sda_s1, sda_s2;
    always @(posedge pclk) {sda_s2, sda_s1} <= {sda_s1, sda_i};

    // ==========================================
    // 1. APB INTERFACE & DMA/IRQ
    // ==========================================
    reg cmd_trigger;

    assign dma_tx_req = (!tx_fifo_full)  & reg_dma_int[2];
    assign dma_rx_req = (!rx_fifo_empty) & reg_dma_int[3];
    assign i2c_irq    = (reg_stat[0] & reg_dma_int[0]) | (!rx_fifo_empty & reg_dma_int[1]);

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_prer <= 16'hFFFF; reg_cmd <= 0; reg_stat <= 0; reg_dma_int <= 0;
            pready <= 0; prdata <= 0; pslverr <= 0; cmd_trigger <= 0;
            tx_fifo_wr <= 0; rx_fifo_rd <= 0;
        end else begin
            pready <= psel && penable;
            pslverr <= 0;
            cmd_trigger <= 0;
            tx_fifo_wr <= 0;
            rx_fifo_rd <= 0;

            if (psel && penable && pwrite) begin
                case (paddr[11:0])
                    12'h000: reg_prer <= pwdata[15:0];
                    12'h004: if (!tx_fifo_full) tx_fifo_wr <= 1;
                    12'h00C: begin 
                        reg_cmd <= pwdata[7:0];
                        if (pwdata[7:4] != 4'b0000) begin
                            cmd_trigger <= 1'b1;
                            reg_stat[1] <= 1'b1; // Transferring
                        end
                    end
                    12'h010: if (pwdata[0]) reg_stat[0] <= 1'b0; // Clear IRQ
                    12'h014: reg_dma_int <= pwdata[3:0];
                    default: ;
                endcase
            end
            
            if (psel && !penable && !pwrite) begin
                case (paddr[11:0])
                    12'h000: prdata <= {16'b0, reg_prer};
                    12'h008: begin 
                        prdata <= {24'b0, rx_fifo_rdata};
                        if (!rx_fifo_empty) rx_fifo_rd <= 1;
                    end
                    12'h00C: prdata <= {24'b0, reg_cmd};
                    12'h010: prdata <= {24'b0, reg_stat[7:4], rx_fifo_empty, tx_fifo_full, reg_stat[1:0]};
                    12'h014: prdata <= {28'b0, reg_dma_int};
                    default: prdata <= 0;
                endcase
            end

            if (core_done) begin
                reg_stat[1] <= 1'b0;
                reg_stat[0] <= 1'b1;
                reg_stat[7] <= core_rx_ack;
                reg_cmd     <= 8'b0;
            end
        end
    end

    // ==========================================
    // 2. I2C CORE FSM 
    // ==========================================
    reg scl_out, sda_out;
    assign scl_oen = scl_out; 
    assign sda_oen = sda_out;
    assign scl_o   = 1'b0;    
    assign sda_o   = 1'b0;
    
    wire tick = (tick_cnt == 0);

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            state <= S_IDLE; tick_cnt <= 0; bit_cnt <= 0;
            scl_out <= 1; sda_out <= 1;
            core_done <= 0; shift_reg <= 0;
            tx_fifo_rd <= 0; rx_fifo_wr <= 0;
        end else begin
            core_done <= 0;
            tx_fifo_rd <= 0;
            rx_fifo_wr <= 0;

            if (tick) tick_cnt <= reg_prer;
            else if (state != S_IDLE) tick_cnt <= tick_cnt - 1;

            case (state)
                S_IDLE: begin
                    if (cmd_trigger) begin
                        if (reg_cmd[4] && !tx_fifo_empty) begin
                            shift_reg <= tx_fifo_rdata;
                            tx_fifo_rd <= 1;
                        end else begin
                            shift_reg <= 8'hFF;
                        end
                        
                        bit_cnt <= 7;
                        tick_cnt <= reg_prer;
                        if (reg_cmd[7]) state <= S_START_A;
                        else            state <= S_BIT_A;
                    end
                end

                S_START_A: if (tick) begin sda_out <= 1; scl_out <= 1; state <= S_START_B; end
                S_START_B: if (tick) begin sda_out <= 0; scl_out <= 1; state <= S_START_C; end
                S_START_C: if (tick) begin sda_out <= 0; scl_out <= 0; 
                                           if (reg_cmd[4] || reg_cmd[5]) state <= S_BIT_A;
                                           else begin core_done <= 1; state <= S_IDLE; end
                                     end

                S_BIT_A: if (tick) begin scl_out <= 0; sda_out <= reg_cmd[4] ? shift_reg[7] : 1'b1; state <= S_BIT_B; end
                S_BIT_B: if (tick) begin scl_out <= 1; state <= S_BIT_C; end
                S_BIT_C: if (tick) begin shift_reg <= {shift_reg[6:0], sda_s2}; state <= S_BIT_D; end
                S_BIT_D: if (tick) begin scl_out <= 0;
                            if (bit_cnt == 0) state <= S_ACK_A;
                            else begin bit_cnt <= bit_cnt - 1; state <= S_BIT_A; end
                         end

                S_ACK_A: if (tick) begin scl_out <= 0; sda_out <= reg_cmd[5] ? reg_cmd[3] : 1'b1; state <= S_ACK_B; end
                S_ACK_B: if (tick) begin scl_out <= 1; state <= S_ACK_C; end
                S_ACK_C: if (tick) begin 
                            core_rx_ack <= sda_s2;
                            core_rx_data <= shift_reg;
                            if (reg_cmd[5] && !rx_fifo_full) rx_fifo_wr <= 1;
                            state <= S_ACK_D; 
                         end
                S_ACK_D: if (tick) begin scl_out <= 0; sda_out <= 1; 
                            if (reg_cmd[6]) state <= S_STOP_A;
                            else begin core_done <= 1; state <= S_IDLE; end
                         end

                S_STOP_A: if (tick) begin sda_out <= 0; scl_out <= 0; state <= S_STOP_B; end
                S_STOP_B: if (tick) begin sda_out <= 0; scl_out <= 1; state <= S_STOP_C; end
                S_STOP_C: if (tick) begin sda_out <= 1; scl_out <= 1; core_done <= 1; state <= S_IDLE; end
            endcase
        end
    end

    // ==========================================
    // 3. FIFO INSTANTIATIONS
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
        .data_i         (core_rx_data), 
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