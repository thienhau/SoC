`timescale 1ns / 1ps

module async_fifo #(
    parameter DATA_WIDTH = 52, // Bề rộng packet. VD: {Write(1), Addr(16), Wdata(32), Size(2), Strb(1)}
    parameter ADDR_WIDTH = 4   // Depth = 16
)(
    // Ghi (Write Domain - CPU Clock)
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  winc,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire                  wfull,
    
    // Đọc (Read Domain - System Bus Clock)
    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  rinc,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire                  rempty
);

    reg  [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
    
    reg  [ADDR_WIDTH:0] wptr_bin, wptr_gray;
    reg  [ADDR_WIDTH:0] rptr_bin, rptr_gray;
    
    reg  [ADDR_WIDTH:0] wptr_gray_sync1, wptr_gray_sync2;
    reg  [ADDR_WIDTH:0] rptr_gray_sync1, rptr_gray_sync2;

    // --- Ghi vào RAM ---
    always @(posedge wclk) begin
        if (winc && !wfull) mem[wptr_bin[ADDR_WIDTH-1:0]] <= wdata;
    end
    assign rdata = mem[rptr_bin[ADDR_WIDTH-1:0]];

    // --- Cập nhật con trỏ Ghi (Write Pointer) ---
    wire [ADDR_WIDTH:0] wptr_bin_next = wptr_bin + (winc & ~wfull);
    wire [ADDR_WIDTH:0] wptr_gray_next = (wptr_bin_next >> 1) ^ wptr_bin_next;
    
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wptr_bin  <= 0;
            wptr_gray <= 0;
        end else begin
            wptr_bin  <= wptr_bin_next;
            wptr_gray <= wptr_gray_next;
        end
    end

    // --- Cập nhật con trỏ Đọc (Read Pointer) ---
    wire [ADDR_WIDTH:0] rptr_bin_next = rptr_bin + (rinc & ~rempty);
    wire [ADDR_WIDTH:0] rptr_gray_next = (rptr_bin_next >> 1) ^ rptr_bin_next;
    
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rptr_bin  <= 0;
            rptr_gray <= 0;
        end else begin
            rptr_bin  <= rptr_bin_next;
            rptr_gray <= rptr_gray_next;
        end
    end

    // --- Đồng bộ hóa (2-Flop Synchronizer) ---
    // Đồng bộ Rptr sang Wclk để tính Full
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) {rptr_gray_sync2, rptr_gray_sync1} <= 0;
        else         {rptr_gray_sync2, rptr_gray_sync1} <= {rptr_gray_sync1, rptr_gray};
    end
    
    // Đồng bộ Wptr sang Rclk để tính Empty
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) {wptr_gray_sync2, wptr_gray_sync1} <= 0;
        else         {wptr_gray_sync2, wptr_gray_sync1} <= {wptr_gray_sync1, wptr_gray};
    end

    // --- Điều kiện Empty / Full ---
    assign rempty = (rptr_gray == wptr_gray_sync2);
    // Full xảy ra khi MSB và MSB-1 bị đảo ngược, các bit còn lại giống nhau
    assign wfull  = (wptr_gray_next == {~rptr_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1], rptr_gray_sync2[ADDR_WIDTH-2:0]});

endmodule