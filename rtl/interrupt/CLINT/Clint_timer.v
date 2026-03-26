// ============================================================
//  clint_timer.v  —  bộ đếm mtime 64-bit với Toggle CDC
//
//  Tối ưu:
//    1. Toggle synchronizer (3-FF) thay vì level sync + edge detect
//       -> robust hơn: hoạt động đúng bất kể tỉ lệ clk/rtc
//       -> không bỏ tick kể cả khi rtc ~ sys_clk/4
//    2. Priority ghi: bus write > rtc_tick (tránh race)
//    3. Snapshot 64-bit: latch mtime_hi khi đọc mtime_lo
//       -> software đọc nhất quán dù counter tràn giữa 2 lần đọc
//    4. (* ASYNC_REG = "TRUE" *) đánh dấu FF cho tool timing analysis
//
//  Ports:
//    mtime_wr_valid_i  : bus muốn ghi mtime
//    mtime_wr_hi_i     : 0=ghi lo word, 1=ghi hi word
//    mtime_wr_data_i   : 32-bit data ghi
//    mtime_wr_strb_i   : byte enable
//    snapshot_latch_i  : pulse khi bus đọc mtime_lo -> latch hi shadow
//    mtime_lo_o        : mtime[31:0] live
//    mtime_hi_o        : mtime[63:32] live
//    mtime_hi_snap_o   : shadow register (dùng khi đọc hi)
// ============================================================

`timescale 1ns/1ps
`include "Clint_defines.vh"

module clint_timer (
    input  wire        clk_i,
    input  wire        rst_ni,
    input  wire        rtc_clk_i,      // RTC clock (bất đồng bộ với clk_i)

    //  Write port từ bus ─
    input  wire        mtime_wr_valid_i,
    input  wire        mtime_wr_hi_i,   // 0=lo, 1=hi
    input  wire [31:0] mtime_wr_data_i,
    input  wire  [3:0] mtime_wr_strb_i,

    //  Snapshot trigger ─
    input  wire        snapshot_latch_i, // 1 khi đọc mtime_lo

    //  Outputs 
    output wire [31:0] mtime_lo_o,
    output wire [31:0] mtime_hi_o,
    output wire [31:0] mtime_hi_snap_o  // shadow cho đọc hi
);

    reg rtc_toggle_r;
    always @(posedge rtc_clk_i or negedge rst_ni) begin
        if (!rst_ni) rtc_toggle_r <= 1'b0;
        else         rtc_toggle_r <= ~rtc_toggle_r;
    end

    (* ASYNC_REG = "TRUE" *) reg sync_ff1_r;
    (* ASYNC_REG = "TRUE" *) reg sync_ff2_r;
    (* ASYNC_REG = "TRUE" *) reg sync_ff3_r;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            sync_ff1_r <= 1'b0;
            sync_ff2_r <= 1'b0;
            sync_ff3_r <= 1'b0;
        end else begin
            sync_ff1_r <= rtc_toggle_r; // có thể metastable
            sync_ff2_r <= sync_ff1_r;   // ổn định hơn
            sync_ff3_r <= sync_ff2_r;   // hoàn toàn ổn định
        end
    end

    wire rtc_tick = sync_ff3_r ^ sync_ff2_r;
    reg [31:0] mtime_lo_r;
    reg [31:0] mtime_hi_r;

    // Carry từ lo sang hi khi increment
    wire lo_carry = (mtime_lo_r == 32'hFFFF_FFFF) && rtc_tick;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            mtime_lo_r <= 32'h0;
            mtime_hi_r <= 32'h0;
        end else if (mtime_wr_valid_i && !mtime_wr_hi_i) begin
            //  Ghi LO word 
            if (mtime_wr_strb_i[0]) mtime_lo_r[7:0]   <= mtime_wr_data_i[7:0];
            if (mtime_wr_strb_i[1]) mtime_lo_r[15:8]  <= mtime_wr_data_i[15:8];
            if (mtime_wr_strb_i[2]) mtime_lo_r[23:16] <= mtime_wr_data_i[23:16];
            if (mtime_wr_strb_i[3]) mtime_lo_r[31:24] <= mtime_wr_data_i[31:24];
        end else if (mtime_wr_valid_i && mtime_wr_hi_i) begin
            //  Ghi HI word 
            if (mtime_wr_strb_i[0]) mtime_hi_r[7:0]   <= mtime_wr_data_i[7:0];
            if (mtime_wr_strb_i[1]) mtime_hi_r[15:8]  <= mtime_wr_data_i[15:8];
            if (mtime_wr_strb_i[2]) mtime_hi_r[23:16] <= mtime_wr_data_i[23:16];
            if (mtime_wr_strb_i[3]) mtime_hi_r[31:24] <= mtime_wr_data_i[31:24];
        end else if (rtc_tick) begin
            //  Auto increment 
            mtime_lo_r <= mtime_lo_r + 32'h1;
            if (lo_carry)
                mtime_hi_r <= mtime_hi_r + 32'h1;
        end
    end

    reg [31:0] mtime_hi_snap_r;

    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni)
            mtime_hi_snap_r <= 32'h0;
        else if (snapshot_latch_i)
            mtime_hi_snap_r <= mtime_hi_r; // chụp hi khi lo được đọc
    end

    //  Outputs 
    assign mtime_lo_o      = mtime_lo_r;
    assign mtime_hi_o      = mtime_hi_r;
    assign mtime_hi_snap_o = mtime_hi_snap_r;

endmodule
