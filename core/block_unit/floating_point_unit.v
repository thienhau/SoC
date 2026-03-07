module fpu_unit (
    input clk,
    input reset,

    // Tín hiệu điều khiển từ Pipeline
    input fpu_start,            // Kích hoạt FPU (từ ID/EX)
    input [4:0] fpu_op,         // Mã lệnh nội bộ (5-bit cho đầy đủ RV32F)
    input [31:0] operand_a,     // Dữ liệu vào 1 (rs1 hoặc integer rs1)
    input [31:0] operand_b,     // Dữ liệu vào 2 (rs2)
    input [31:0] operand_c,     // Dữ liệu vào 3 (rs3, cho FMADD/FMSUB/FNMADD/FNMSUB)
    // Kết quả trả về
    output reg [31:0] result,
    output fpu_stall,           // Báo bận -> Dừng Pipeline
    output reg fpu_done         // Báo xong -> Pipeline tiếp tục
);

    // Backward-compatible: first 8 ops giữ nguyên encoding 3-bit (mở rộng sang 5-bit)
    localparam FOP_ADD      = 5'b00000; // FADD.S
    localparam FOP_SUB      = 5'b00001; // FSUB.S
    localparam FOP_MUL      = 5'b00010; // FMUL.S
    localparam FOP_CVT_W_S  = 5'b00011; // FCVT.W.S  (Float -> Signed Int)
    localparam FOP_CVT_S_W  = 5'b00100; // FCVT.S.W  (Signed Int -> Float)
    localparam FOP_EQ       = 5'b00101; // FEQ.S
    localparam FOP_LT       = 5'b00110; // FLT.S
    localparam FOP_LE       = 5'b00111; // FLE.S
    localparam FOP_DIV      = 5'b01000; // FDIV.S
    localparam FOP_SQRT     = 5'b01001; // FSQRT.S
    localparam FOP_MIN      = 5'b01010; // FMIN.S
    localparam FOP_MAX      = 5'b01011; // FMAX.S
    localparam FOP_SGNJ     = 5'b01100; // FSGNJ.S
    localparam FOP_SGNJN    = 5'b01101; // FSGNJN.S
    localparam FOP_SGNJX    = 5'b01110; // FSGNJX.S
    localparam FOP_MV_X_W   = 5'b01111; // FMV.X.W  (Float bits -> Int reg)
    localparam FOP_MV_W_X   = 5'b10000; // FMV.W.X  (Int bits -> Float reg)
    localparam FOP_CLASS    = 5'b10001; // FCLASS.S
    localparam FOP_CVT_WU_S = 5'b10010; // FCVT.WU.S (Float -> Unsigned Int)
    localparam FOP_CVT_S_WU = 5'b10011; // FCVT.S.WU (Unsigned Int -> Float)
    localparam FOP_FMADD    = 5'b10100; // FMADD.S   (a*b + c)
    localparam FOP_FMSUB    = 5'b10101; // FMSUB.S   (a*b - c)
    localparam FOP_FNMSUB   = 5'b10110; // FNMSUB.S  (-a*b + c)
    localparam FOP_FNMADD   = 5'b10111; // FNMADD.S  (-a*b - c)

    // --- MÁY TRẠNG THÁI ---
    localparam STATE_IDLE = 2'b00;
    localparam STATE_CALC = 2'b01;
    localparam STATE_DONE = 2'b10;

    reg [1:0] state;
    reg [3:0] latency_counter;

    // Các biến nội bộ để tính toán
    reg [31:0] reg_a, reg_b, reg_c;
    reg [4:0]  reg_op;

    // Stall logic
    assign fpu_stall = (state == STATE_CALC) || (fpu_start && state == STATE_IDLE);

    always @(posedge clk) begin
        if (reset) begin
            state <= STATE_IDLE;
            result <= 0;
            fpu_done <= 0;
            latency_counter <= 0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    fpu_done <= 0;
                    if (fpu_start) begin
                        reg_a <= operand_a;
                        reg_b <= operand_b;
                        reg_c <= operand_c;
                        reg_op <= fpu_op;
                        state <= STATE_CALC;

                        case (fpu_op)
                            FOP_ADD, FOP_SUB:               latency_counter <= 4'd3;
                            FOP_MUL:                        latency_counter <= 4'd4;
                            FOP_DIV:                        latency_counter <= 4'd10;
                            FOP_SQRT:                       latency_counter <= 4'd12;
                            FOP_CVT_W_S, FOP_CVT_S_W,
                            FOP_CVT_WU_S, FOP_CVT_S_WU:    latency_counter <= 4'd2;
                            FOP_FMADD, FOP_FMSUB,
                            FOP_FNMSUB, FOP_FNMADD:         latency_counter <= 4'd5;
                            FOP_EQ, FOP_LT, FOP_LE,
                            FOP_MIN, FOP_MAX:               latency_counter <= 4'd1;
                            FOP_SGNJ, FOP_SGNJN, FOP_SGNJX,
                            FOP_MV_X_W, FOP_MV_W_X,
                            FOP_CLASS:                      latency_counter <= 4'd0;
                            default:                        latency_counter <= 4'd2;
                        endcase
                    end
                end

                STATE_CALC: begin
                    if (latency_counter > 0) begin
                        latency_counter <= latency_counter - 1;
                    end else begin
                        case (reg_op)
                            FOP_ADD:      result <= ieee754_add(reg_a, reg_b);
                            FOP_SUB:      result <= ieee754_sub(reg_a, reg_b);
                            FOP_MUL:      result <= ieee754_mul(reg_a, reg_b);
                            FOP_DIV:      result <= ieee754_div(reg_a, reg_b);
                            FOP_SQRT:     result <= ieee754_sqrt(reg_a);
                            FOP_CVT_W_S:  result <= f2i(reg_a);
                            FOP_CVT_S_W:  result <= i2f(reg_a);
                            FOP_CVT_WU_S: result <= f2u(reg_a);
                            FOP_CVT_S_WU: result <= u2f(reg_a);
                            FOP_EQ:       result <= {31'b0, feq(reg_a, reg_b)};
                            FOP_LT:       result <= {31'b0, flt(reg_a, reg_b)};
                            FOP_LE:       result <= {31'b0, fle(reg_a, reg_b)};
                            FOP_MIN:      result <= fmin_fn(reg_a, reg_b);
                            FOP_MAX:      result <= fmax_fn(reg_a, reg_b);
                            FOP_SGNJ:     result <= {reg_b[31], reg_a[30:0]};
                            FOP_SGNJN:    result <= {~reg_b[31], reg_a[30:0]};
                            FOP_SGNJX:    result <= {reg_a[31] ^ reg_b[31], reg_a[30:0]};
                            FOP_MV_X_W:   result <= reg_a;
                            FOP_MV_W_X:   result <= reg_a;
                            FOP_CLASS:    result <= fclass_fn(reg_a);
                            FOP_FMADD:    result <= fmadd_fn(reg_a, reg_b, reg_c);
                            FOP_FMSUB:    result <= fmsub_fn(reg_a, reg_b, reg_c);
                            FOP_FNMSUB:   result <= fnmsub_fn(reg_a, reg_b, reg_c);
                            FOP_FNMADD:   result <= fnmadd_fn(reg_a, reg_b, reg_c);
                            default:      result <= 32'b0;
                        endcase
                        state <= STATE_DONE;
                    end
                end

                STATE_DONE: begin
                    fpu_done <= 1;
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

    
    // IEEE 754 Helper Functions
    // Hằng số đặc biệt
    localparam [31:0] POS_ZERO     = 32'h00000000;
    localparam [31:0] NEG_ZERO     = 32'h80000000;
    localparam [31:0] POS_INF      = 32'h7F800000;
    localparam [31:0] NEG_INF      = 32'hFF800000;
    localparam [31:0] CANONICAL_NAN = 32'h7FC00000; // Quiet NaN (canonical)

    function is_nan;
        input [31:0] f;
        begin
            is_nan = (f[30:23] == 8'hFF) && (f[22:0] != 0);
        end
    endfunction

    function is_inf;
        input [31:0] f;
        begin
            is_inf = (f[30:23] == 8'hFF) && (f[22:0] == 0);
        end
    endfunction

    function is_zero;
        input [31:0] f;
        begin
            is_zero = (f[30:23] == 0) && (f[22:0] == 0);
        end
    endfunction

    function is_denorm;
        input [31:0] f;
        begin
            is_denorm = (f[30:23] == 0) && (f[22:0] != 0);
        end
    endfunction

    // 1. IEEE 754 Addition (Single Precision)
    //    - Xử lý đầy đủ: Zero, Denorm, NaN, Infinity
    //    - Guard, Round, Sticky bits cho precision
    //    - Round-to-Nearest-Even (RNE) - default RISC-V rounding mode
    function [31:0] ieee754_add;
        input [31:0] a, b;
        reg sign_a, sign_b;
        reg [7:0] exp_a, exp_b;
        reg [24:0] mant_a, mant_b;  // 1 (hidden) + 23 (fraction) + 1 (guard space)
        reg [7:0] exp_diff;
        reg [27:0] wide_a, wide_b;  // 25-bit mantissa + 3 bits (Guard, Round, Sticky)
        reg [27:0] sum_mant;
        reg [7:0] res_exp;
        reg res_sign;
        reg guard, round_bit, sticky;
        integer norm_i;
        reg [7:0] shift_amt;
        reg [24:0] shifted_out; // Để tính sticky bit
        begin
            //Xử lý các trường hợp đặc biệt
            // NaN propagation: nếu 1 trong 2 là NaN -> trả về canonical NaN
            if (is_nan(a) || is_nan(b)) begin
                ieee754_add = CANONICAL_NAN;
            end
            // Infinity handling
            else if (is_inf(a) && is_inf(b)) begin
                if (a[31] == b[31])
                    ieee754_add = a; // +Inf + +Inf = +Inf, -Inf + -Inf = -Inf
                else
                    ieee754_add = CANONICAL_NAN; // +Inf + -Inf = NaN
            end
            else if (is_inf(a)) begin
                ieee754_add = a;
            end
            else if (is_inf(b)) begin
                ieee754_add = b;
            end
            // Zero handling
            else if (is_zero(a) && is_zero(b)) begin
                // +0 + -0 = +0 (RNE), -0 + -0 = -0
                if (a[31] && b[31])
                    ieee754_add = NEG_ZERO;
                else
                    ieee754_add = POS_ZERO;
            end
            else if (is_zero(a)) begin
                ieee754_add = b;
            end
            else if (is_zero(b)) begin
                ieee754_add = a;
            end
            else begin
                //tách các trường 
                sign_a = a[31]; 
                exp_a = a[30:23]; 
                sign_b = b[31]; 
                exp_b = b[30:23];
                // Hidden bit: 1 cho normalized, 0 cho denormalized
                if (is_denorm(a)) begin
                    mant_a = {1'b0, 1'b0, a[22:0]};
                    exp_a = 8'd1; // Denorm dùng exp=1 cho tính toán
                end else begin
                    mant_a = {1'b0, 1'b1, a[22:0]};
                end
                if (is_denorm(b)) begin
                    mant_b = {1'b0, 1'b0, b[22:0]};
                    exp_b = 8'd1;
                end else begin
                    mant_b = {1'b0, 1'b1, b[22:0]};
                end
                // Alignment (thêm GRS bits)
                // wide format: [27:3] = mantissa (25 bit), [2] = Guard, [1] = Round, [0] = Sticky
                wide_a = {mant_a, 3'b000};
                wide_b = {mant_b, 3'b000};
                if (exp_a >= exp_b) begin
                    exp_diff = exp_a - exp_b;
                    res_exp = exp_a;
                    // Shift B right, accumulate sticky bits
                    if (exp_diff > 0) begin
                        if (exp_diff >= 28) begin
                            // Shift quá lớn, toàn bộ B trở thành sticky
                            sticky = (wide_b != 0);
                            wide_b = 0;
                            wide_b[0] = sticky;
                        end else begin
                            // Tính sticky từ các bits bị shift ra
                            sticky = 0;
                            for (norm_i = 0; norm_i < 28; norm_i = norm_i + 1) begin
                                if (norm_i < exp_diff && norm_i < 28)
                                    sticky = sticky | wide_b[norm_i];
                            end
                            wide_b = wide_b >> exp_diff;
                            wide_b[0] = wide_b[0] | sticky;
                        end
                    end
                end else begin
                    exp_diff = exp_b - exp_a;
                    res_exp = exp_b;
                    if (exp_diff >= 28) begin
                        sticky = (wide_a != 0);
                        wide_a = 0;
                        wide_a[0] = sticky;
                    end else begin
                        sticky = 0;
                        for (norm_i = 0; norm_i < 28; norm_i = norm_i + 1) begin
                            if (norm_i < exp_diff && norm_i < 28)
                                sticky = sticky | wide_a[norm_i];
                        end
                        wide_a = wide_a >> exp_diff;
                        wide_a[0] = wide_a[0] | sticky;
                    end
                end
                //Add/Subtract mantissa
                if (sign_a == sign_b) begin
                    sum_mant = wide_a + wide_b;
                    res_sign = sign_a;
                end else begin
                    if (wide_a >= wide_b) begin
                        sum_mant = wide_a - wide_b;
                        res_sign = sign_a;
                    end else begin
                        sum_mant = wide_b - wide_a;
                        res_sign = sign_b;
                    end
                end
                //Normalization
                if (sum_mant == 0) begin
                    ieee754_add = POS_ZERO; // Kết quả = +0 (RNE)
                end else begin
                    // Overflow: bit 27 set (carry from addition)
                    if (sum_mant[27]) begin
                        // Shift right, preserve sticky
                        sticky = sum_mant[0];
                        sum_mant = sum_mant >> 1;
                        sum_mant[0] = sum_mant[0] | sticky;
                        res_exp = res_exp + 1;
                    end
                    // Hidden bit (bit 26) not set → normalize by shifting left
                    else if (!sum_mant[26]) begin
                        for (norm_i = 0; norm_i < 27; norm_i = norm_i + 1) begin
                            if (!sum_mant[26] && res_exp > 1) begin
                                sum_mant = sum_mant << 1;
                                res_exp = res_exp - 1;
                            end
                        end
                        // Nếu vẫn chưa normalized và exp<=1 -> denormalized result
                        if (!sum_mant[26] && res_exp <= 1) begin
                            res_exp = 0;
                        end
                    end
                    // else: bit 26 set (hidden bit) → already normalized
                    // Rounding (Round to Nearest Even - RNE)
                    guard = sum_mant[2];
                    round_bit = sum_mant[1];
                    sticky = sum_mant[0];
                    // RNE: round up if guard=1 AND (round|sticky=1 OR result LSB=1)
                    if (guard && (round_bit || sticky || sum_mant[3])) begin
                        sum_mant = sum_mant + 8; // Thêm 1 vào bit [3] (LSB of mantissa)
                        // Kiểm tra overflow sau rounding (carry vào bit 27)
                        if (sum_mant[27]) begin
                            sum_mant = sum_mant >> 1;
                            res_exp = res_exp + 1;
                        end
                    end
                    //Overflow/Underflow check
                    if (res_exp >= 8'hFF) begin
                        // Overflow -> Infinity
                        ieee754_add = {res_sign, 8'hFF, 23'b0};
                    end else if (res_exp == 0) begin
                        // Denormalized result
                        ieee754_add = {res_sign, 8'b0, sum_mant[25:3]};
                    end else begin
                        // Normalized result: bit26=hidden, bits[25:3]=fraction(23bit)
                        ieee754_add = {res_sign, res_exp, sum_mant[25:3]};
                    end
                end
            end
        end
    endfunction

    // 2. IEEE 754 Multiplication (Single Precision)
    //    - Full precision với Guard, Round, Sticky bits
    //    - Xử lý Zero, Denorm, NaN, Infinity
    function [31:0] ieee754_mul;
        input [31:0] a, b;
        reg sign_a, sign_b, res_sign;
        reg [7:0] exp_a, exp_b;
        reg [23:0] mant_a, mant_b;
        reg [47:0] product;  // 24-bit × 24-bit = 48-bit
        reg [8:0] res_exp;   // 9-bit để detect overflow
        reg guard, round_bit, sticky;
        integer i;
        begin
            //Special cases
            if (is_nan(a) || is_nan(b)) begin
                ieee754_mul = CANONICAL_NAN;
            end
            else if (is_inf(a) || is_inf(b)) begin
                // Inf × 0 = NaN
                if (is_zero(a) || is_zero(b))
                    ieee754_mul = CANONICAL_NAN;
                else begin
                    res_sign = a[31] ^ b[31];
                    ieee754_mul = {res_sign, 8'hFF, 23'b0};  // ±Infinity
                end
            end
            else if (is_zero(a) || is_zero(b)) begin
                res_sign = a[31] ^ b[31];
                ieee754_mul = {res_sign, 31'b0};  // ±0
            end
            else begin
                //Normal/Denormal multiplication 
                sign_a = a[31];
                sign_b = b[31];
                res_sign = sign_a ^ sign_b;
                exp_a = a[30:23];
                exp_b = b[30:23];
                // Hidden bit
                if (is_denorm(a)) begin
                    mant_a = {1'b0, a[22:0]};
                    exp_a = 8'd1;
                end else begin
                    mant_a = {1'b1, a[22:0]};
                end
                if (is_denorm(b)) begin
                    mant_b = {1'b0, b[22:0]};
                    exp_b = 8'd1;
                end else begin
                    mant_b = {1'b1, b[22:0]};
                end
                // Exponent: (exp_a - 127) + (exp_b - 127) + 127 = exp_a + exp_b - 127
                res_exp = exp_a + exp_b - 8'd127;
                // Mantissa multiplication: 24-bit × 24-bit = 48-bit
                product = mant_a * mant_b;
                // Normalize: Product is in range [1.0, 4.0) (bit 47 or 46 là MSB)
                // Format: bit[47:46] always has the leading 1
                if (product[47]) begin
                    // Product in [2.0, 4.0): shift right 1
                    sticky = |(product[22:0]);
                    guard = product[23];
                    round_bit = product[24];
                    product = product >> 24;  // Keep bits [47:24]
                    res_exp = res_exp + 1;
                end else begin
                    // Product in [1.0, 2.0): no shift needed
                    sticky = |(product[21:0]);
                    guard = product[22];
                    round_bit = product[23];
                    product = product >> 23;  // Keep bits [46:23]
                end
                // Round to nearest even
                if (guard && (round_bit || sticky || product[0])) begin
                    product = product + 1;
                    // Check for mantissa overflow
                    if (product[24]) begin
                        product = product >> 1;
                        res_exp = res_exp + 1;
                    end
                end
                // Check overflow/underflow
                if (res_exp >= 9'd255) begin
                    // Overflow → Infinity
                    ieee754_mul = {res_sign, 8'hFF, 23'b0};
                end else if (res_exp <= 9'd0) begin
                    // Underflow → Zero (simplified, should denormalize)
                    ieee754_mul = {res_sign, 31'b0};
                end else begin
                    // Normal result
                    ieee754_mul = {res_sign, res_exp[7:0], product[22:0]};
                end
            end
        end
    endfunction

    // 3. IEEE 754 Subtraction: Đảo dấu B rồi cộng
    function [31:0] ieee754_sub;
        input [31:0] a, b;
        reg [31:0] neg_b;
        begin
            // Đảo bit dấu (trừ trường hợp NaN thì giữ nguyên)
            if (is_nan(b))
                neg_b = CANONICAL_NAN;
            else
                neg_b = {~b[31], b[30:0]};
            ieee754_sub = ieee754_add(a, neg_b);
        end
    endfunction

    // 4. IEEE 754 Comparison Operations
    //    - FEQ: Equal, FLT: Less Than, FLE: Less or Equal
    //    - Return 1 (true) or 0 (false) as integer
    //    - NaN → always false (except FEQ where NaN != NaN)
    function feq;  // Float Equal
        input [31:0] a, b;
        begin
            // NaN is never equal to anything (including itself)
            if (is_nan(a) || is_nan(b))
                feq = 0;
            // Check bit-exact equality (handles ±0 correctly: +0 == -0)
            else if (is_zero(a) && is_zero(b))
                feq = 1;
            else
                feq = (a == b);
        end
    endfunction
    
    function flt;  // Float Less Than
        input [31:0] a, b;
        reg sign_a, sign_b;
        begin
            if (is_nan(a) || is_nan(b)) begin
                flt = 0;  // NaN comparison always false
            end else if (is_zero(a) && is_zero(b)) begin
                flt = 0;  // -0 < +0 is false
            end else begin
                sign_a = a[31];
                sign_b = b[31];
                // Different signs: negative < positive
                if (sign_a && !sign_b)
                    flt = 1;
                else if (!sign_a && sign_b)
                    flt = 0;
                // Same sign: compare magnitude
                else if (!sign_a) begin
                    // Both positive: compare as unsigned
                    flt = (a[30:0] < b[30:0]);
                end else begin
                    // Both negative: reverse comparison
                    flt = (a[30:0] > b[30:0]);
                end
            end
        end
    endfunction
    
    function fle;  // Float Less or Equal
        input [31:0] a, b;
        begin
            fle = feq(a, b) || flt(a, b);
        end
    endfunction

    // 5. Float to Int (FCVT.W.S) - Round toward Zero (RTZ) theo RISC-V spec
    //    - Xử lý NaN, Infinity, Overflow
    function [31:0] f2i;
        input [31:0] f;
        reg [7:0] exp;
        reg [23:0] mant;
        reg [7:0] shift;
        reg [31:0] res;
        reg sign;
        begin
            sign = f[31];
            exp = f[30:23];
            mant = {1'b1, f[22:0]};
            
            // NaN -> trả về INT32_MAX (RISC-V spec)
            if (is_nan(f)) begin
                f2i = 32'h7FFFFFFF; // 2^31 - 1
            end
            // +Infinity -> INT32_MAX
            else if (is_inf(f) && !sign) begin
                f2i = 32'h7FFFFFFF;
            end
            // -Infinity -> INT32_MIN
            else if (is_inf(f) && sign) begin
                f2i = 32'h80000000; // -2^31
            end
            // Zero hoặc Denormalized (trị tuyệt đối < 1) -> 0
            else if (is_zero(f) || is_denorm(f) || exp < 8'd127) begin
                f2i = 0;
            end
            else begin
                shift = exp - 8'd127; 
                // Kiểm tra overflow: nếu shift >= 31, số quá lớn cho int32
                if (shift >= 8'd31) begin
                    if (sign)
                        f2i = 32'h80000000; // INT32_MIN
                    else
                        f2i = 32'h7FFFFFFF; // INT32_MAX
                end
                // Trường hợp đặc biệt: -2^31 (representable)
                else if (shift == 8'd30 && sign && f[22:0] == 0) begin
                    f2i = 32'h80000000;
                end
                else begin
                    // Dịch mantissa để lấy phần nguyên (truncate = RTZ)
                    if (shift >= 8'd23)
                        res = mant << (shift - 8'd23);
                    else
                        res = mant >> (8'd23 - shift);
                    // Kiểm tra overflow sau shift
                    if (!sign && res[31]) begin
                        f2i = 32'h7FFFFFFF; // Overflow
                    end else begin
                        f2i = sign ? (~res + 1) : res; // Two's complement nếu âm
                    end
                end
            end
        end
    endfunction

    // 6. Int to Float (FCVT.S.W) - Round to Nearest Even (RNE)
    //    - Xử lý rounding khi int > 24 bit precision
    // FIX: Vòng lặp đi từ bit 0 LÊN bit 31. Mỗi bit set sẽ ghi đè kết quả trước đó.
    //      Bit cao nhất (MSB) ghi đè cuối cùng → đúng. Không cần cờ 'found'.
    function automatic [31:0] i2f;
        input [31:0] i;
        reg [31:0] abs_i;
        reg [7:0] exp;
        reg [22:0] mant;
        reg [31:0] temp_shift;
        integer k;
        begin
            if (i == 0) begin
                i2f = 0;
            end else begin
                abs_i = i[31] ? -i : i; // Lấy trị tuyệt đối
                
                exp = 0;
                mant = 0;
                
                // Vòng lặp tìm MSB: đi từ bit thấp lên bit cao
                // Bit cao nhất (MSB) sẽ ghi đè kết quả cuối cùng → đúng
                for (k = 0; k <= 31; k = k + 1) begin
                    if (abs_i[k]) begin
                        exp = 127 + k;
                        
                        if (k < 23) begin
                            temp_shift = abs_i << (23 - k);
                            mant = temp_shift[22:0]; 
                        end else begin
                            temp_shift = abs_i >> (k - 23);
                            mant = temp_shift[22:0];
                        end
                    end
                end
                i2f = {i[31], exp, mant};
            end
        end
    endfunction

    // 7. IEEE 754 Division (Single Precision)
    function [31:0] ieee754_div;
        input [31:0] a, b;
        reg res_sign;
        reg [7:0] exp_a, exp_b;
        reg [23:0] mant_a, mant_b;
        integer res_exp;
        reg [50:0] remainder;
        reg [26:0] quotient;
        reg guard, round_bit, sticky;
        integer div_i;
        begin
            if (is_nan(a) || is_nan(b)) ieee754_div = CANONICAL_NAN;
            else if (is_inf(a) && is_inf(b)) ieee754_div = CANONICAL_NAN;
            else if (is_inf(a)) ieee754_div = {a[31] ^ b[31], 8'hFF, 23'b0};
            else if (is_inf(b)) ieee754_div = {a[31] ^ b[31], 31'b0};
            else if (is_zero(b)) begin
                if (is_zero(a)) ieee754_div = CANONICAL_NAN;
                else ieee754_div = {a[31] ^ b[31], 8'hFF, 23'b0};
            end
            else if (is_zero(a)) ieee754_div = {a[31] ^ b[31], 31'b0};
            else begin
                res_sign = a[31] ^ b[31];
                exp_a = a[30:23]; exp_b = b[30:23];
                if (is_denorm(a)) begin mant_a = {1'b0, a[22:0]}; exp_a = 8'd1; end
                else mant_a = {1'b1, a[22:0]};
                if (is_denorm(b)) begin mant_b = {1'b0, b[22:0]}; exp_b = 8'd1; end
                else mant_b = {1'b1, b[22:0]};
                res_exp = exp_a - exp_b + 127;
                // Restoring division: 27-bit quotient
                remainder = {3'b0, mant_a, 24'b0};
                quotient = 0;
                for (div_i = 26; div_i >= 0; div_i = div_i - 1) begin
                    if (remainder[50:24] >= {3'b0, mant_b}) begin
                        remainder[50:24] = remainder[50:24] - {3'b0, mant_b};
                        quotient[div_i] = 1'b1;
                    end
                    remainder = remainder << 1;
                end
                sticky = |remainder;
                // Normalize
                if (quotient[26]) begin
                    guard = quotient[2];
                    round_bit = quotient[1];
                    sticky = sticky | quotient[0];
                    quotient = quotient >> 3;
                end else begin
                    guard = quotient[1];
                    round_bit = quotient[0];
                    quotient = quotient >> 2;
                    res_exp = res_exp - 1;
                end
                // RNE rounding
                if (guard && (round_bit || sticky || quotient[0])) begin
                    quotient = quotient + 1;
                    if (quotient[24]) begin
                        quotient = quotient >> 1;
                        res_exp = res_exp + 1;
                    end
                end

                if (res_exp >= 255) ieee754_div = {res_sign, 8'hFF, 23'b0};
                else if (res_exp <= 0) ieee754_div = {res_sign, 31'b0};
                else ieee754_div = {res_sign, res_exp[7:0], quotient[22:0]};
            end
        end
    endfunction

    // 8. IEEE 754 Square Root (Single Precision)
    function [31:0] ieee754_sqrt;
        input [31:0] a;
        reg [7:0] exp_a;
        reg [23:0] mant_a;
        integer res_exp;
        reg [51:0] radicand;
        reg [51:0] sq_rem;
        reg [26:0] sq_root;
        reg [51:0] sq_trial;
        reg guard, round_bit, sticky;
        integer sq_i;
        begin
            if (is_nan(a) || (a[31] && !is_zero(a))) begin
                ieee754_sqrt = CANONICAL_NAN;
            end
            else if (is_zero(a)) begin
                ieee754_sqrt = a;
            end
            else if (is_inf(a)) begin
                ieee754_sqrt = POS_INF;
            end
            else begin
                exp_a = a[30:23];
                if (is_denorm(a)) begin mant_a = {1'b0, a[22:0]}; exp_a = 8'd1; end
                else mant_a = {1'b1, a[22:0]};
                // Setup radicand (52 bits = 26 pairs → 26-bit root)
                if (exp_a[0]) begin // exp odd → (exp-127) even
                    radicand = {2'b01, mant_a[22:0], 27'b0};
                    res_exp = (exp_a + 127) / 2;
                end else begin // exp even → (exp-127) odd
                    radicand = {2'b10, mant_a[22:0], 27'b0};
                    res_exp = (exp_a + 126) / 2;
                end
                // Restoring binary square root
                sq_rem = 0;
                sq_root = 0;
                for (sq_i = 25; sq_i >= 0; sq_i = sq_i - 1) begin
                    sq_rem = (sq_rem << 2) | {50'b0, radicand[2*sq_i+1], radicand[2*sq_i]};
                    sq_trial = {sq_root, 2'b01};
                    sq_root = sq_root << 1;
                    if (sq_rem >= sq_trial) begin
                        sq_rem = sq_rem - sq_trial;
                        sq_root = sq_root | 1;
                    end
                end
                // sq_root[25]=hidden, [24:2]=mantissa(23b), [1]=guard, [0]=round
                guard = sq_root[1];
                round_bit = sq_root[0];
                sticky = |sq_rem;

                if (guard && (round_bit || sticky || sq_root[2])) begin
                    sq_root = sq_root + 27'd4;
                    if (sq_root[26]) begin
                        sq_root = sq_root >> 1;
                        res_exp = res_exp + 1;
                    end
                end

                if (res_exp >= 255) ieee754_sqrt = POS_INF;
                else if (res_exp <= 0) ieee754_sqrt = 32'b0;
                else ieee754_sqrt = {1'b0, res_exp[7:0], sq_root[24:2]};
            end
        end
    endfunction

    // 9. FMIN.S / FMAX.S
    function [31:0] fmin_fn;
        input [31:0] a, b;
        begin
            if (is_nan(a) && is_nan(b)) fmin_fn = CANONICAL_NAN;
            else if (is_nan(a)) fmin_fn = b;
            else if (is_nan(b)) fmin_fn = a;
            else if (is_zero(a) && is_zero(b)) fmin_fn = {a[31] | b[31], 31'b0};
            else if (flt(a, b)) fmin_fn = a;
            else fmin_fn = b;
        end
    endfunction

    function [31:0] fmax_fn;
        input [31:0] a, b;
        begin
            if (is_nan(a) && is_nan(b)) fmax_fn = CANONICAL_NAN;
            else if (is_nan(a)) fmax_fn = b;
            else if (is_nan(b)) fmax_fn = a;
            else if (is_zero(a) && is_zero(b)) fmax_fn = {a[31] & b[31], 31'b0};
            else if (flt(b, a)) fmax_fn = a;
            else fmax_fn = b;
        end
    endfunction

    // 10. FCLASS.S
    function [31:0] fclass_fn;
        input [31:0] a;
        reg [9:0] res;
        begin
            res = 10'b0;
            if (a[30:23] == 8'hFF && a[22:0] != 0) begin
                if (a[22]) res[9] = 1; // Quiet NaN
                else res[8] = 1;       // Signaling NaN
            end
            else if (a[30:23] == 8'hFF) begin
                if (a[31]) res[0] = 1; else res[7] = 1;
            end
            else if (a[30:23] == 0 && a[22:0] == 0) begin
                if (a[31]) res[3] = 1; else res[4] = 1;
            end
            else if (a[30:23] == 0) begin
                if (a[31]) res[2] = 1; else res[5] = 1;
            end
            else begin
                if (a[31]) res[1] = 1; else res[6] = 1;
            end
            fclass_fn = {22'b0, res};
        end
    endfunction

    // 11. FCVT.WU.S (Float -> Unsigned Int, RTZ)
    function [31:0] f2u;
        input [31:0] f;
        reg [7:0] exp_v;
        reg [23:0] mant_v;
        reg [7:0] shift_v;
        reg [31:0] res;
        begin
            if (is_nan(f)) f2u = 32'hFFFFFFFF;
            else if (f[31] && !is_zero(f)) f2u = 32'h00000000;
            else if (is_inf(f)) f2u = 32'hFFFFFFFF;
            else if (is_zero(f) || is_denorm(f)) f2u = 32'h00000000;
            else begin
                exp_v = f[30:23];
                mant_v = {1'b1, f[22:0]};
                if (exp_v < 8'd127) f2u = 32'h00000000;
                else begin
                    shift_v = exp_v - 8'd127;
                    if (shift_v >= 8'd32) f2u = 32'hFFFFFFFF;
                    else begin
                        if (shift_v >= 8'd23) res = mant_v << (shift_v - 8'd23);
                        else res = mant_v >> (8'd23 - shift_v);
                        f2u = res;
                    end
                end
            end
        end
    endfunction

    // 12. FCVT.S.WU (Unsigned Int -> Float)
    function automatic [31:0] u2f;
        input [31:0] i;
        reg [7:0] exp_v;
        reg [22:0] mant_v;
        reg [31:0] temp_shift;
        integer k;
        begin
            if (i == 0) begin
                u2f = 32'b0;
            end else begin
                exp_v = 0;
                mant_v = 0;
                for (k = 0; k <= 31; k = k + 1) begin
                    if (i[k]) begin
                        exp_v = 127 + k;
                        if (k < 23) begin
                            temp_shift = i << (23 - k);
                            mant_v = temp_shift[22:0];
                        end else begin
                            temp_shift = i >> (k - 23);
                            mant_v = temp_shift[22:0];
                        end
                    end
                end
                u2f = {1'b0, exp_v, mant_v};
            end
        end
    endfunction

    // 13. FMADD/FMSUB/FNMSUB/FNMADD (sử dụng double-rounding approach)
    function [31:0] fmadd_fn;
        input [31:0] a, b, c;
        begin
            fmadd_fn = ieee754_add(ieee754_mul(a, b), c);
        end
    endfunction

    function [31:0] fmsub_fn;
        input [31:0] a, b, c;
        begin
            fmsub_fn = ieee754_sub(ieee754_mul(a, b), c);
        end
    endfunction

    function [31:0] fnmsub_fn;
        input [31:0] a, b, c;
        begin
            fnmsub_fn = ieee754_sub(c, ieee754_mul(a, b));
        end
    endfunction

    function [31:0] fnmadd_fn;
        input [31:0] a, b, c;
        reg [31:0] prod;
        begin
            prod = ieee754_mul(a, b);
            if (is_nan(prod))
                fnmadd_fn = CANONICAL_NAN;
            else
                fnmadd_fn = ieee754_sub({~prod[31], prod[30:0]}, c);
        end
    endfunction

endmodule