// ============================================================
//  clint_regfile.v  —  Thanh ghi msip[] và mtimecmp[]
//
//  Tối ưu:
//    1. Read path: THUẦN TỔ HỢP (combinational) -> 0 cycle latency
//       trong regfile, pipeline được xử lý ở AXI slave
//    2. Write path: 1 clock cycle (registered)
//    3. Address decode: priority casez thuần tổ hợp
//       -> tool synthesis tối ưu thành tree of muxes
//    4. Flat output: mtimecmp_flat_o = [NUM_HARTS*64-1:0]
//       Verilog-2001 không cho phép unpacked
//       array ở port
//    5. Kiểm tra hart index OOB -> error signal
//    6. msip chỉ bit[0] hợp lệ, bit[31:1] luôn đọc 0
//
//  Read latency: 0 cycles (combinational)
//  Write latency: 1 cycle (registered at posedge)
//
//  Ghi chú về mtime:
//    Ghi mtime: regfile KHÔNG lưu, chỉ route tín hiệu mtime_wr_*
//    ra clint_timer.v để timer xử lý.
//    Đọc mtime: lấy trực tiếp từ timer outputs (mtime_lo_i, mtime_hi_snap_i).
// ============================================================

`timescale 1ns/1ps
`include "Clint_defines.vh"

module clint_regfile #(
    parameter NUM_HARTS   = 1,
    parameter HART_IDX_W  = 1,   // = max(1, $clog2(NUM_HARTS)), truyền từ top
    parameter ADDR_W      = 26
) (
    input  wire              clk_i,
    input  wire              rst_ni,

    //  Internal register bus (từ AXI4 slave) 
    // Request
    input  wire              req_valid_i,
    input  wire              req_write_i,    // 1=write, 0=read
    input  wire [ADDR_W-1:0] req_addr_i,
    input  wire       [31:0] req_wdata_i,
    input  wire        [3:0] req_wstrb_i,
    // Response (combinational, valid ngay khi req_valid_i)
    output reg        [31:0] rsp_rdata_o,
    output reg               rsp_error_o,

    //  Timer interface 
    input  wire [31:0] mtime_lo_i,         // live mtime lo
    input  wire [31:0] mtime_hi_snap_i,    // snapshot mtime hi
    // Route mtime write đến timer
    output reg         mtime_wr_valid_o,
    output reg         mtime_wr_hi_o,
    output reg  [31:0] mtime_wr_data_o,
    output reg   [3:0] mtime_wr_strb_o,
    // Snapshot trigger
    output wire        snapshot_latch_o,   // 1 khi đọc mtime_lo

    //  Register outputs (-> clint_irq_gen) 
    output reg  [NUM_HARTS-1:0]     msip_o,
    output reg  [NUM_HARTS*64-1:0]  mtimecmp_flat_o  // flattened
);
    //  Register storage
    reg              msip_r     [0:NUM_HARTS-1]; // chỉ 1 bit/hart
    reg [63:0]       mtimecmp_r [0:NUM_HARTS-1];

    //  Address decode (combinational)
    //  Phân vùng:
    //    in_msip      : addr trong [0x0000, 0x4000)
    //    in_mtimecmp  : addr trong [0x4000, 0xBFF8)
    //    in_mtime_lo  : addr == 0xBFF8
    //    in_mtime_hi  : addr == 0xBFFC
    //    addr_oob     : ngoài map -> error
    // Các cờ vùng (wire, tổ hợp)
    wire in_msip     = (req_addr_i < `CLINT_MTIMECMP_BASE);
    wire in_mtimecmp = (req_addr_i >= `CLINT_MTIMECMP_BASE) &&
                       (req_addr_i < `CLINT_MTIME_LO);
    wire in_mtime_lo = (req_addr_i == `CLINT_MTIME_LO);
    wire in_mtime_hi = (req_addr_i == `CLINT_MTIME_HI);
    wire in_any      = in_msip | in_mtimecmp | in_mtime_lo | in_mtime_hi;

    // Hart index tính từ địa chỉ
    // msip:     word index = addr[25:2], lấy HART_IDX_W bit thấp
    // mtimecmp: dword index = (addr - 0x4000) >> 3
    wire [HART_IDX_W-1:0] msip_hart_idx;
    wire [HART_IDX_W-1:0] mtcmp_hart_idx;
    wire                  mtcmp_hi_sel; // 0=lo word, 1=hi word

    assign msip_hart_idx  = req_addr_i[2 + HART_IDX_W - 1 : 2];
    assign mtcmp_hi_sel   = req_addr_i[2];   // bit 2 phân biệt lo/hi
    
    // (addr - 0x4000) >> 3 -> lấy HART_IDX_W bit bắt đầu từ bit 3
    wire [ADDR_W-1:0] mtcmp_offset = req_addr_i - `CLINT_MTIMECMP_BASE;
    assign mtcmp_hart_idx = mtcmp_offset[3 + HART_IDX_W - 1 : 3];

    // Hart index OOB check (chuyển sang integer để compare)
    wire msip_hart_ok  = ({1'b0, msip_hart_idx}  < NUM_HARTS);
    wire mtcmp_hart_ok = ({1'b0, mtcmp_hart_idx} < NUM_HARTS);

    // Final address error
    wire addr_error = req_valid_i && (
        !in_any ||
        (in_msip     && !msip_hart_ok)  ||
        (in_mtimecmp && !mtcmp_hart_ok)
    );

    // Snapshot trigger: pulse khi đọc mtime_lo
    assign snapshot_latch_o = req_valid_i && !req_write_i && in_mtime_lo;

    // Read mux (priority casez style trong always @(*))
    always @(*) begin
        rsp_rdata_o = 32'h0; // default
        rsp_error_o = addr_error;

        if (req_valid_i && !req_write_i) begin
            if (in_mtime_lo) begin
                rsp_rdata_o = mtime_lo_i;
            end else if (in_mtime_hi) begin
                // Trả shadow register (nhất quán với lo vừa đọc)
                rsp_rdata_o = mtime_hi_snap_i;
            end else if (in_msip && msip_hart_ok) begin
                // Chỉ bit[0] hợp lệ, bit[31:1] luôn = 0
                rsp_rdata_o = {31'h0, msip_r[msip_hart_idx]};
            end else if (in_mtimecmp && mtcmp_hart_ok) begin
                rsp_rdata_o = mtcmp_hi_sel
                    ? mtimecmp_r[mtcmp_hart_idx][63:32]
                    : mtimecmp_r[mtcmp_hart_idx][31:0];
            end
            // in_any nhưng OOB -> rdata=0, error=1 (đã set ở trên)
        end
    end

    integer wi; // loop variable cho reset
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            for (wi = 0; wi < NUM_HARTS; wi = wi + 1) begin
                msip_r[wi]     <= 1'b0;
                mtimecmp_r[wi] <= `CLINT_MTIMECMP_RST;
            end
        end else if (req_valid_i && req_write_i) begin
            if (in_msip && msip_hart_ok) begin
                // Ghi bit[0]. Bit[31:1] bị bỏ qua hoàn toàn.
                if (req_wstrb_i[0])
                    msip_r[msip_hart_idx] <= req_wdata_i[0];
            end else if (in_mtimecmp && mtcmp_hart_ok) begin
                if (!mtcmp_hi_sel) begin
                    // Ghi mtimecmp LO với byte strobe
                    if (req_wstrb_i[0])
                        mtimecmp_r[mtcmp_hart_idx][7:0]   <= req_wdata_i[7:0];
                    if (req_wstrb_i[1])
                        mtimecmp_r[mtcmp_hart_idx][15:8]  <= req_wdata_i[15:8];
                    if (req_wstrb_i[2])
                        mtimecmp_r[mtcmp_hart_idx][23:16] <= req_wdata_i[23:16];
                    if (req_wstrb_i[3])
                        mtimecmp_r[mtcmp_hart_idx][31:24] <= req_wdata_i[31:24];
                end else begin
                    // Ghi mtimecmp HI với byte strobe
                    if (req_wstrb_i[0])
                        mtimecmp_r[mtcmp_hart_idx][39:32] <= req_wdata_i[7:0];
                    if (req_wstrb_i[1])
                        mtimecmp_r[mtcmp_hart_idx][47:40] <= req_wdata_i[15:8];
                    if (req_wstrb_i[2])
                        mtimecmp_r[mtcmp_hart_idx][55:48] <= req_wdata_i[23:16];
                    if (req_wstrb_i[3])
                        mtimecmp_r[mtcmp_hart_idx][63:56] <= req_wdata_i[31:24];
                end
                // mtime write: route ra ngoài (xử lý bằng always @ dưới)
            end
        end
    end

    always @(*) begin
        mtime_wr_valid_o = req_valid_i && req_write_i &&
                           (in_mtime_lo || in_mtime_hi);
        mtime_wr_hi_o    = in_mtime_hi;
        mtime_wr_data_o  = req_wdata_i;
        mtime_wr_strb_o  = req_wstrb_i;
    end

    //  Flatten outputs (-> irq_gen)
    //  nên flatten thành bus phẳng.
    //  irq_gen sẽ dùng [i*64+:64].
    integer gi;
    always @(*) begin
        for (gi = 0; gi < NUM_HARTS; gi = gi + 1) begin
            msip_o[gi]                   = msip_r[gi];
            mtimecmp_flat_o[gi*64 +: 64] = mtimecmp_r[gi];
        end
    end

endmodule