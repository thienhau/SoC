`timescale 1ns/1ps

// =========================================================================
// MODULE TOP: TRNG 128-BIT (RO + RG ARCHITECTURE)
// =========================================================================
module trng_128b (
    input  wire         clock,
    input  wire         reset,   // Active HIGH reset (chuẩn Chisel cũ)
    input  wire         enable,
    output reg          valid,
    output wire [127:0] rand_out
);
    wire [6:0]   ro_taps;
    wire [127:0] rg_state;
    reg  [7:0]   wait_cnt;

    // Timer: Chờ Ring Generator xáo trộn 128 chu kỳ để Entropy đạt mức tối đa
    always @(posedge clock) begin
        if (reset || !enable) begin
            wait_cnt <= 8'b0;
            valid    <= 1'b0;
        end else begin
            if (wait_cnt == 8'd128) begin
                valid <= 1'b1;
            end else begin
                wait_cnt <= wait_cnt + 1'b1;
                valid <= 1'b0;
            end
        end
    end

    assign rand_out = valid ? rg_state : 128'b0;

    RingOscillator ro (
        .io_i_en  (enable),
        .io_o_out (ro_taps)
    );

    RingGenerator rg (
        .clock       (clock),
        .reset       (reset),
        .io_i_rst    (~enable),
        .io_i_inject (ro_taps),
        .io_o_state  (rg_state)
    );
endmodule

// =========================================================================
// NATIVE 128-BIT RING GENERATOR (CHUẨN G128)
// =========================================================================
module RingGenerator (
    input  wire         clock,
    input  wire         reset,
    input  wire         io_i_rst,
    input  wire [6:0]   io_i_inject,
    output wire [127:0] io_o_state
);
    reg [127:0] stateReg;

    // Đa thức G128: 128, 95, 66, 29, 0
    wire [127:0] feedback = stateReg[127] ? ( (128'b1 << 95) | (128'b1 << 66) | (128'b1 << 29) | 128'b1 ) : 128'b0;

    // Trải đều 7 mũi tiêm từ RO vào 7 vị trí
    wire [127:0] inject_mask = (128'b0)
                             | ({127'b0, io_i_inject[6]} << 114)
                             | ({127'b0, io_i_inject[5]} << 96)
                             | ({127'b0, io_i_inject[4]} << 78)
                             | ({127'b0, io_i_inject[3]} << 60)
                             | ({127'b0, io_i_inject[2]} << 42)
                             | ({127'b0, io_i_inject[1]} << 24)
                             | ({127'b0, io_i_inject[0]} << 6);

    always @(posedge clock) begin
        if (reset || io_i_rst) begin
            stateReg <= 128'b0;
        end else begin
            stateReg <= {stateReg[126:0], 1'b0} ^ feedback ^ inject_mask;
        end
    end

    assign io_o_state = stateReg;
endmodule

// =========================================================================
// XILINX PRIMITIVES RING OSCILLATOR (7 TẦNG)
// =========================================================================
(* KEEP_HIERARCHY = "yes", DONT_TOUCH = "yes" *)
module RingOscillator (
    input  wire       io_i_en,
    output wire [6:0] io_o_out
);
    wire ro_nand_nand_gate_0_io_out; 
    wire ro_invs_not_gate_0_io_out;
    wire ro_invs_not_gate_1_io_out; 
    wire ro_invs_not_gate_2_io_out;
    wire ro_invs_not_gate_3_io_out;
    wire ro_invs_not_gate_4_io_out;
    wire ro_invs_not_gate_5_io_out;

    xilinx_not ro_invs_not_gate_0 (.io_i0(ro_nand_nand_gate_0_io_out), .io_out(ro_invs_not_gate_0_io_out));
    xilinx_not ro_invs_not_gate_1 (.io_i0(ro_invs_not_gate_0_io_out), .io_out(ro_invs_not_gate_1_io_out));
    xilinx_not ro_invs_not_gate_2 (.io_i0(ro_invs_not_gate_1_io_out), .io_out(ro_invs_not_gate_2_io_out));
    xilinx_not ro_invs_not_gate_3 (.io_i0(ro_invs_not_gate_2_io_out), .io_out(ro_invs_not_gate_3_io_out));
    xilinx_not ro_invs_not_gate_4 (.io_i0(ro_invs_not_gate_3_io_out), .io_out(ro_invs_not_gate_4_io_out));
    xilinx_not ro_invs_not_gate_5 (.io_i0(ro_invs_not_gate_4_io_out), .io_out(ro_invs_not_gate_5_io_out));
    xilinx_nand ro_nand_nand_gate_0 (.io_i0(io_i_en), .io_i1(ro_invs_not_gate_5_io_out), .io_out(ro_nand_nand_gate_0_io_out));

    assign io_o_out = { 
        ro_invs_not_gate_5_io_out,
        ro_invs_not_gate_4_io_out,
        ro_invs_not_gate_3_io_out, 
        ro_invs_not_gate_2_io_out, 
        ro_invs_not_gate_1_io_out, 
        ro_invs_not_gate_0_io_out, 
        ro_nand_nand_gate_0_io_out 
    };
endmodule

module xilinx_nand (input wire io_i0, input wire io_i1, output wire io_out);
    xilinx_primitive_nand inst (.i0(io_i0), .i1(io_i1), .out(io_out));
endmodule

module xilinx_not (input wire io_i0, output wire io_out);
    xilinx_primitive_not inst (.i0(io_i0), .out(io_out));
endmodule

(* DONT_TOUCH = "yes" *)
module xilinx_primitive_nand (input wire i0, input wire i1, output wire out);
`ifdef XILINX
    (* DONT_TOUCH = "yes" *) wire w0; (* DONT_TOUCH = "yes" *) wire w1; (* DONT_TOUCH = "yes" *) wire w2;
    LUT6 #(.INIT(64'h0000000000000007)) LUT6_inst (.O(w2), .I0(w0), .I1(w1), .I2(1'b0), .I3(1'b0), .I4(1'b0), .I5(1'b0));
    assign w0 = i0; assign w1 = i1; assign out = w2;
`else
    assign out = ~(i0 & i1);
`endif
endmodule

(* DONT_TOUCH = "yes" *)
module xilinx_primitive_not (input wire i0, output wire out);
`ifdef XILINX
    (* DONT_TOUCH = "yes" *) wire w0; (* DONT_TOUCH = "yes" *) wire w1;
    LUT6 #(.INIT(64'h0000000000000001)) LUT6_inst (.O(w1), .I0(w0), .I1(1'b0), .I2(1'b0), .I3(1'b0), .I4(1'b0), .I5(1'b0));
    assign w0 = i0; assign out = w1;
`else
    assign out = ~i0;
`endif
endmodule