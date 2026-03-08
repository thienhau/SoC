`timescale 1ns / 1ps

module tb_fpu_unit();

    // =========================================================================
    // SIGNALS & INSTANTIATION
    // =========================================================================
    reg         clk;
    reg         reset_n;
    reg         fpu_start;
    reg  [4:0]  fpu_op;
    reg  [31:0] operand_a;
    reg  [31:0] operand_b;
    wire [31:0] result;
    wire        fpu_stall;
    wire        fpu_done;

    fpu_unit #(
        .BITS_PER_CYCLE(2)
    ) dut (
        .clk(clk),
        .reset_n(reset_n),
        .fpu_start(fpu_start),
        .fpu_op(fpu_op),
        .operand_a(operand_a),
        .operand_b(operand_b),
        .result(result),
        .fpu_stall(fpu_stall),
        .fpu_done(fpu_done)
    );

    // =========================================================================
    // CLOCK GENERATOR
    // =========================================================================
    initial begin clk = 0; forever #5 clk = ~clk; end

    // =========================================================================
    // FPU OPCODES
    // =========================================================================
    localparam FOP_ADD      = 5'b00000;
    localparam FOP_SUB      = 5'b00001;
    localparam FOP_MUL      = 5'b00010;
    localparam FOP_CVT_W_S  = 5'b00011; // Float to Int
    localparam FOP_CVT_S_W  = 5'b00100; // Int to Float
    localparam FOP_EQ       = 5'b00101;
    localparam FOP_LT       = 5'b00110;
    localparam FOP_LE       = 5'b00111;
    localparam FOP_DIV      = 5'b01000;
    localparam FOP_SQRT     = 5'b01001;
    localparam FOP_MIN      = 5'b01010;
    localparam FOP_MAX      = 5'b01011;
    localparam FOP_MV_X_W   = 5'b01111; // Bitcast F -> I
    localparam FOP_MV_W_X   = 5'b10000; // Bitcast I -> F
    localparam FOP_CVT_WU_S = 5'b10010; // Float to UInt
    localparam FOP_CVT_S_WU = 5'b10011; // UInt to Float

    // =========================================================================
    // IEEE-754 CONSTANTS (HEX)
    // =========================================================================
    localparam F_ZERO  = 32'h0000_0000;
    localparam F_ONE   = 32'h3F80_0000; // 1.0
    localparam F_TWO   = 32'h4000_0000; // 2.0
    localparam F_THREE = 32'h4040_0000; // 3.0
    localparam F_FOUR  = 32'h4080_0000; // 4.0
    localparam F_FIVE  = 32'h40A0_0000; // 5.0
    localparam F_SEVEN = 32'h40E0_0000; // 7.0
    localparam F_TEN   = 32'h4120_0000; // 10.0
    localparam F_16    = 32'h4180_0000; // 16.0
    localparam F_1P5   = 32'h3FC0_0000; // 1.5
    localparam F_2P5   = 32'h4020_0000; // 2.5
    localparam F_3P5   = 32'h4060_0000; // 3.5
    localparam F_M_ONE = 32'hBF80_0000; // -1.0
    localparam F_M_FIVE= 32'hC0A0_0000; // -5.0
    localparam F_QNAN  = 32'h7FC0_0000; // NaN
    localparam F_INF   = 32'h7F80_0000; // Infinity

    // =========================================================================
    // SCOREBOARD & EXECUTION TASK
    // =========================================================================
    integer test_passed = 0;
    integer test_failed = 0;

    task run_fpu_test(input [4:0] op, input [31:0] a, input [31:0] b, input [31:0] exp_res, input [8*40:1] test_name);
        begin
            @(posedge clk);
            fpu_op    = op;
            operand_a = a;
            operand_b = b;
            fpu_start = 1;

            @(posedge clk);
            fpu_start = 0;

            // Chờ FPU xử lý (FSM có thể chạy nhiều vòng lặp)
            wait(fpu_done == 1'b1);
            
            // Check kết quả (Dùng === để bắt cả bit X/Z nếu có)
            // Đặc cách cho NaN vì NaN có thể có nhiều giá trị Mantissa khác nhau
            if (result === exp_res || (exp_res[30:23] == 8'hFF && result[30:23] == 8'hFF && result[22:0] != 0)) begin
                $display("  [PASS] %0s", test_name);
                test_passed = test_passed + 1;
            end else begin
                $display("  [FAIL] %0s", test_name);
                $display("         Expected: %h, Got: %h", exp_res, result);
                test_failed = test_failed + 1;
            end
            @(posedge clk);
        end
    endtask

    // =========================================================================
    // MAIN TEST SUITE
    // =========================================================================
    initial begin
        fpu_start = 0; fpu_op = 0; operand_a = 0; operand_b = 0;
        reset_n = 0; #50 reset_n = 1; #20;

        $display("\n=========================================================");
        $display(" PHASE 1: BASIC ARITHMETIC (ADD, SUB, MUL, DIV)");
        $display("=========================================================");
        run_fpu_test(FOP_ADD, F_1P5, F_2P5, F_FOUR,  "1.5 + 2.5 = 4.0");
        run_fpu_test(FOP_SUB, F_FIVE, F_ONE, F_FOUR,  "5.0 - 1.0 = 4.0");
        run_fpu_test(FOP_MUL, F_TWO, F_3P5, F_SEVEN, "2.0 * 3.5 = 7.0");
        run_fpu_test(FOP_DIV, F_TEN, F_2P5, F_FOUR,  "10.0 / 2.5 = 4.0");

        $display("\n=========================================================");
        $display(" PHASE 2: MATHEMATICAL FUNCTIONS (SQRT)");
        $display("=========================================================");
        run_fpu_test(FOP_SQRT, F_16, F_ZERO, F_FOUR, "sqrt(16.0) = 4.0");
        run_fpu_test(FOP_SQRT, F_FOUR, F_ZERO, F_TWO,  "sqrt(4.0) = 2.0");
        run_fpu_test(FOP_SQRT, F_M_ONE, F_ZERO, F_QNAN, "sqrt(-1.0) = NaN");

        $display("\n=========================================================");
        $display(" PHASE 3: COMPARISONS & MIN/MAX");
        $display("=========================================================");
        run_fpu_test(FOP_EQ,  F_FIVE, F_FIVE, 32'd1,   "5.0 == 5.0 -> True(1)");
        run_fpu_test(FOP_LT,  F_ONE,  F_FIVE, 32'd1,   "1.0 < 5.0  -> True(1)");
        run_fpu_test(FOP_LE,  F_TEN,  F_TEN,  32'd1,   "10.0 <= 10.0 -> True(1)");
        run_fpu_test(FOP_MIN, F_M_ONE, F_FIVE, F_M_ONE, "min(-1.0, 5.0) = -1.0");
        run_fpu_test(FOP_MAX, F_THREE, F_SEVEN, F_SEVEN, "max(3.0, 7.0) = 7.0");

        $display("\n=========================================================");
        $display(" PHASE 4: CONVERSIONS (FLOAT <-> INT)");
        $display("=========================================================");
        run_fpu_test(FOP_CVT_W_S,  F_FIVE, F_ZERO, 32'd5, "Float(5.0) -> Int(5)");
        run_fpu_test(FOP_CVT_S_W,  32'hFFFF_FFFB, F_ZERO, F_M_FIVE, "Int(-5) -> Float(-5.0)"); // -5 int = 0xFFFFFFFB
        run_fpu_test(FOP_CVT_WU_S, F_TEN, F_ZERO, 32'd10, "Float(10.0) -> UInt(10)");
        run_fpu_test(FOP_CVT_S_WU, 32'd16, F_ZERO, F_16, "UInt(16) -> Float(16.0)");

        $display("\n=========================================================");
        $display(" PHASE 5: BITCAST MOVES");
        $display("=========================================================");
        run_fpu_test(FOP_MV_X_W, F_TEN, F_ZERO, F_TEN, "MV_X_W (F->I Bitcast)");
        run_fpu_test(FOP_MV_W_X, F_TEN, F_ZERO, F_TEN, "MV_W_X (I->F Bitcast)");

        $display("\n=========================================================");
        $display(" PHASE 6: EDGE CASES");
        $display("=========================================================");
        run_fpu_test(FOP_ADD, F_FIVE, F_ZERO, F_FIVE, "5.0 + 0.0 = 5.0");
        run_fpu_test(FOP_MUL, F_TEN,  F_ZERO, F_ZERO, "10.0 * 0.0 = 0.0");
        run_fpu_test(FOP_DIV, F_TEN,  F_ZERO, F_INF,  "10.0 / 0.0 = Infinity"); // Kiểm tra FPU của bạn có chia cho 0 ra Inf không

        #200;
        $display("\n=========================================================");
        $display(" FPU VERIFICATION SCOREBOARD");
        $display("=========================================================");
        $display(" Total Tests : %0d", test_passed + test_failed);
        $display(" PASSED      : %0d", test_passed);
        $display(" FAILED      : %0d", test_failed);
        
        if (test_failed == 0) 
            $display("\n >>> SUCCESS! ALL TESTS PASSED! <<<\n");
        else 
            $display("\n >>> CRITICAL FAILURE DETECTED! <<<\n");
        
        $finish;
    end
endmodule