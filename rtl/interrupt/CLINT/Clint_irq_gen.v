// ============================================================
//  clint_irq_gen.v  —  Sinh tín hiệu ngắt cho N hart
//
//  Tối ưu:
//    1. N comparator 64-bit unsigned chạy song song (generate loop)
//       -> synthesis tool tự ánh xạ sang chuỗi adder/compare cells
//    2. PIPELINE_OUTPUT param: thêm FF stage cắt critical path
//       -> mtimecmp register -> comparator -> FF -> CPU
//       -> cho phép tool P&R đặt comparators gần regfile, FF gần CPU
//    3. msip pass-through: 0 logic, trực tiếp từ regfile output
//
//  Input flatten: mtimecmp_flat_i = [NUM_HARTS*64-1:0]
//    Hart i: mtimecmp_flat_i[i*64 +: 64]
//
//  Latency:
//    PIPELINE_OUTPUT=0: tổ hợp, mtip thay đổi ngay khi mtime/mtimecmp đổi
//    PIPELINE_OUTPUT=1: 1 cycle delay (khuyến nghị cho timing closure)
// ============================================================

`timescale 1ns/1ps

module clint_irq_gen #(
    parameter NUM_HARTS      = 1,
    parameter PIPELINE_OUTPUT = 1   // 1: thêm FF output stage
) (
    input  wire clk_i,
    input  wire rst_ni,

    //  Từ regfile ─
    input  wire [NUM_HARTS-1:0]    msip_i,
    input  wire [NUM_HARTS*64-1:0] mtimecmp_flat_i,

    //  Từ timer ─
    input  wire [31:0] mtime_lo_i,
    input  wire [31:0] mtime_hi_i,

    //  Outputs đến CPU hart ─
    output wire [NUM_HARTS-1:0] msip_o,
    output wire [NUM_HARTS-1:0] mtip_o
);

    //  Tổ hợp mtime 64-bit 
    wire [63:0] mtime_w = {mtime_hi_i, mtime_lo_i};

    //  N comparators song song ─
    wire [NUM_HARTS-1:0] mtip_comb;
    wire [NUM_HARTS-1:0] msip_comb;

    genvar i;
    generate
        for (i = 0; i < NUM_HARTS; i = i + 1) begin : gen_cmp
            // 64-bit unsigned compare: mtip khi mtime >= mtimecmp
            assign mtip_comb[i] = (mtime_w >= mtimecmp_flat_i[i*64 +: 64]);
            assign msip_comb[i] = msip_i[i];
        end
    endgenerate

    //  Output stage (optional FF) 
    generate
        if (PIPELINE_OUTPUT) begin : gen_piped
            reg [NUM_HARTS-1:0] msip_r;
            reg [NUM_HARTS-1:0] mtip_r;
            always @(posedge clk_i or negedge rst_ni) begin
                if (!rst_ni) begin
                    msip_r <= {NUM_HARTS{1'b0}};
                    mtip_r <= {NUM_HARTS{1'b0}};
                end else begin
                    msip_r <= msip_comb;
                    mtip_r <= mtip_comb;
                end
            end
            assign msip_o = msip_r;
            assign mtip_o = mtip_r;
        end else begin : gen_direct
            assign msip_o = msip_comb;
            assign mtip_o = mtip_comb;
        end
    endgenerate

endmodule