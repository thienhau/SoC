//==================================================================================================
// File: fpu_unit.v
// Description: Fully parameterized iterative IEEE-754 FPU
// Fixed: Added FLW/FSW support and missing SQRT restoring algorithm logic.
//==================================================================================================
`timescale 1ns / 1ps

module fpu_unit #(
    parameter BITS_PER_CYCLE = 2 // Có thể đổi thành 1, 2 hoặc 4 để chia nhỏ data path
)(
    input  wire        clk,
    input  wire        reset_n,
    input  wire        fpu_start,
    input  wire [4:0]  fpu_op,
    input  wire [31:0] operand_a,
    input  wire [31:0] operand_b,
    output reg  [31:0] result,
    output wire        fpu_stall,
    output reg         fpu_done
);

    //==============================================================================================
    // Local Parameters - Opcodes
    //==============================================================================================
    localparam FOP_ADD      = 5'b00000;
    localparam FOP_SUB      = 5'b00001;
    localparam FOP_MUL      = 5'b00010;
    localparam FOP_CVT_W_S  = 5'b00011; 
    localparam FOP_CVT_S_W  = 5'b00100; 
    localparam FOP_EQ       = 5'b00101;
    localparam FOP_LT       = 5'b00110;
    localparam FOP_LE       = 5'b00111;
    localparam FOP_DIV      = 5'b01000;
    localparam FOP_SQRT     = 5'b01001;
    localparam FOP_MIN      = 5'b01010;
    localparam FOP_MAX      = 5'b01011;
    localparam FOP_MV_X_W   = 5'b01111; 
    localparam FOP_MV_W_X   = 5'b10000; 
    localparam FOP_CVT_WU_S = 5'b10010; 
    localparam FOP_CVT_S_WU = 5'b10011; 

    // IEEE-754 Constants
    localparam [31:0] POS_ZERO  = 32'h00000000;
    localparam [31:0] NEG_ZERO  = 32'h80000000;
    localparam [31:0] QNAN      = 32'h7FC00000;

    // FSM States
    localparam STATE_IDLE      = 4'd0;
    localparam STATE_UNPACK    = 4'd1;
    localparam STATE_FAST_PATH = 4'd2; 
    localparam STATE_ALIGN     = 4'd3; 
    localparam STATE_ITERATE   = 4'd4; 
    localparam STATE_NORMALIZE = 4'd5;
    localparam STATE_ROUND     = 4'd6;
    localparam STATE_PACK      = 4'd7;
    localparam STATE_DONE      = 4'd8;

    //==============================================================================================
    // Module-Level Variables & Wires (Tuân thủ Verilog-2001)
    //==============================================================================================
    reg [3:0]  state;
    reg        sign_a, sign_b, sign_res;
    reg [8:0]  exp_a, exp_b, exp_res; 
    reg [49:0] mant_a, mant_b, mant_res; 
    reg [4:0]  op_reg;
    reg [6:0]  iteration_count;

    // Biến phụ trợ cho vòng lặp tổ hợp (Combinational variables)
    integer    i;
    integer    shift_amt;
    reg [49:0] temp_mant_res;
    reg [49:0] temp_mant_a;
    reg [49:0] temp_mant_b;
    reg [6:0]  temp_count;
    reg [8:0]  temp_exp_res;
    reg [51:0] next_rem; // Dùng riêng cho khâu lặp SQRT
    reg [51:0] test_val; // Dùng riêng cho khâu lặp SQRT

    // Các cờ kiểm tra toán hạng
    wire a_is_zero = (operand_a[30:23] == 8'd0) && (operand_a[22:0] == 23'd0);
    wire b_is_zero = (operand_b[30:23] == 8'd0) && (operand_b[22:0] == 23'd0);
    wire a_is_nan  = (operand_a[30:23] == 8'hFF) && (operand_a[22:0] != 23'd0);
    wire b_is_nan  = (operand_b[30:23] == 8'hFF) && (operand_b[22:0] != 23'd0);

    // Tính toán Exponent sẵn cho lệnh SQRT để tránh lỗi tràn số signed/unsigned
    wire signed [10:0] s_true_exp     = $signed({3'b0, operand_a[30:23]}) - 11'sd127;
    wire signed [10:0] s_true_exp_odd = s_true_exp - 11'sd1;
    wire [8:0] sqrt_exp_even = $unsigned((s_true_exp >>> 1)) + 9'd127;
    wire [8:0] sqrt_exp_odd  = $unsigned((s_true_exp_odd >>> 1)) + 9'd127;

    // Tín hiệu dành cho làm tròn (Rounding)
    wire w_lsb    = mant_res[25];
    wire w_guard  = mant_res[24];
    wire w_round  = mant_res[23];
    wire w_sticky = |mant_res[22:0];

    assign fpu_stall = (state != STATE_IDLE) || (fpu_start && state == STATE_IDLE);

    //==============================================================================================
    // Main Sequential Block
    //==============================================================================================
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state           <= STATE_IDLE;
            fpu_done        <= 1'b0;
            result          <= 32'd0;
            mant_a          <= 50'd0;
            mant_b          <= 50'd0;
            mant_res        <= 50'd0;
            iteration_count <= 7'd0;
            sign_a          <= 1'b0;
            sign_b          <= 1'b0;
            sign_res        <= 1'b0;
            exp_a           <= 9'd0;
            exp_b           <= 9'd0;
            exp_res         <= 9'd0;
            op_reg          <= 5'd0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    fpu_done <= 1'b0;
                    if (fpu_start) begin
                        op_reg <= fpu_op;
                        state  <= STATE_UNPACK;
                    end
                end

                STATE_UNPACK: begin
                    sign_a <= operand_a[31];
                    sign_b <= (op_reg == FOP_SUB) ? ~operand_b[31] : operand_b[31];

                    exp_a <= (a_is_zero) ? 9'd0 : {1'b0, operand_a[30:23]};
                    exp_b <= (b_is_zero) ? 9'd0 : {1'b0, operand_b[30:23]};

                    mant_a <= a_is_zero ? 50'd0 : {2'b01, operand_a[22:0], 25'd0};
                    mant_b <= b_is_zero ? 50'd0 : {2'b01, operand_b[22:0], 25'd0};
                    
                    mant_res <= 50'd0;
                    
                    if (a_is_nan || b_is_nan) begin
                        result <= QNAN;
                        state  <= STATE_DONE;
                    end else begin
                        case (op_reg)
                            FOP_EQ, FOP_LT, FOP_LE, FOP_MIN, FOP_MAX, FOP_MV_X_W, FOP_MV_W_X: begin
                                state <= STATE_FAST_PATH;
                            end
                            FOP_SQRT: begin
                                if (operand_a[31] && !a_is_zero) begin
                                    result <= QNAN; // Căn của số âm = NaN
                                    state  <= STATE_DONE;
                                end else if (a_is_zero) begin
                                    result <= operand_a; // 0 hoặc -0
                                    state  <= STATE_DONE;
                                end else begin
                                    // Kiểm tra số mũ chẵn/lẻ (bit LSB của exponent là operand_a[23])
                                    if (operand_a[23] == 1'b0) begin 
                                        mant_a  <= {2'b01, operand_a[22:0], 25'd0} << 1;
                                        exp_res <= sqrt_exp_odd;
                                    end else begin
                                        exp_res <= sqrt_exp_even;
                                    end
                                    sign_res        <= 1'b0; // SQRT luôn dương
                                    mant_b          <= 50'd0; // Sử dụng mant_b làm thanh ghi Remainder
                                    iteration_count <= 7'd26;
                                    state           <= STATE_ITERATE;
                                end
                            end
                            FOP_CVT_S_W, FOP_CVT_S_WU: begin
                                sign_a <= operand_a[31];
                                if (op_reg == FOP_CVT_S_W && operand_a[31]) begin
                                    mant_a <= {18'd0, (~operand_a + 1'b1)}; 
                                end else begin
                                    mant_a <= {18'd0, operand_a};
                                end
                                exp_res         <= 9'd158; 
                                iteration_count <= 7'd32; 
                                state           <= STATE_ITERATE;
                            end
                            FOP_ADD, FOP_SUB: begin
                                state <= STATE_ALIGN;
                            end
                            FOP_MUL, FOP_DIV: begin
                                iteration_count <= 7'd26; 
                                state           <= STATE_ITERATE;
                            end
                            FOP_CVT_W_S, FOP_CVT_WU_S: begin
                                iteration_count <= 7'd26; 
                                state           <= STATE_ITERATE;
                            end
                            default: state <= STATE_DONE;
                        endcase
                    end
                end

                STATE_FAST_PATH: begin
                    case (op_reg)
                        FOP_MV_X_W: result <= operand_a; 
                        FOP_MV_W_X: result <= operand_a; 
                        FOP_EQ:     result <= {31'b0, (operand_a == operand_b)};
                        FOP_LT: begin
                            if (sign_a != sign_b) result <= {31'b0, sign_a};
                            else result <= {31'b0, (sign_a ? (operand_a > operand_b) : (operand_a < operand_b))};
                        end
                        FOP_LE: begin
                            if (operand_a == operand_b) result <= 32'd1;
                            else if (sign_a != sign_b) result <= {31'b0, sign_a};
                            else result <= {31'b0, (sign_a ? (operand_a > operand_b) : (operand_a < operand_b))};
                        end
                        FOP_MIN: begin
                            if (sign_a != sign_b) result <= sign_a ? operand_a : operand_b;
                            else result <= (sign_a ? (operand_a > operand_b) : (operand_a < operand_b)) ? operand_a : operand_b;
                        end
                        FOP_MAX: begin
                            if (sign_a != sign_b) result <= sign_b ? operand_a : operand_b;
                            else result <= (sign_a ? (operand_a < operand_b) : (operand_a > operand_b)) ? operand_a : operand_b;
                        end
                    endcase
                    state <= STATE_DONE;
                end

                STATE_ALIGN: begin
                    if (exp_a > exp_b) begin
                        mant_b    <= mant_b >> 1;
                        mant_b[0] <= mant_b[0] | mant_b[1]; 
                        exp_b     <= exp_b + 1;
                    end else if (exp_a < exp_b) begin
                        mant_a    <= mant_a >> 1;
                        mant_a[0] <= mant_a[0] | mant_a[1];
                        exp_a     <= exp_a + 1;
                    end else begin
                        exp_res         <= exp_a;
                        iteration_count <= 7'd1; 
                        state           <= STATE_ITERATE;
                    end
                end

                STATE_ITERATE: begin
                    temp_mant_res = mant_res;
                    temp_mant_a   = mant_a;
                    temp_mant_b   = mant_b;
                    temp_exp_res  = exp_res;
                    temp_count    = iteration_count;

                    for (i = 0; i < BITS_PER_CYCLE; i = i + 1) begin
                        if (temp_count > 0) begin
                            case (op_reg)
                                FOP_ADD, FOP_SUB: begin
                                    if (sign_a == sign_b) begin
                                        temp_mant_res = temp_mant_a + temp_mant_b;
                                        sign_res <= sign_a;
                                    end else begin
                                        if (temp_mant_a >= temp_mant_b) begin
                                            temp_mant_res = temp_mant_a - temp_mant_b;
                                            sign_res <= sign_a;
                                        end else begin
                                            temp_mant_res = temp_mant_b - temp_mant_a;
                                            sign_res <= sign_b;
                                        end
                                    end
                                    temp_count = 0;
                                end

                                FOP_MUL: begin
                                    if (temp_count == 26) begin
                                        temp_exp_res  = exp_a + exp_b - 9'd127;
                                        sign_res      <= sign_a ^ sign_b;
                                        temp_mant_res = 50'd0;
                                    end
                                    if (temp_mant_b[25]) temp_mant_res = temp_mant_res + temp_mant_a; 
                                    temp_mant_a = temp_mant_a << 1;
                                    temp_mant_b = temp_mant_b >> 1;
                                    temp_count  = temp_count - 1;
                                end

                                FOP_DIV: begin
                                    if (temp_count == 26) begin
                                        temp_exp_res = exp_a - exp_b + 9'd127;
                                        sign_res     <= sign_a ^ sign_b;
                                    end
                                    if (temp_mant_a >= temp_mant_b) begin
                                        temp_mant_a   = temp_mant_a - temp_mant_b;
                                        temp_mant_res = {temp_mant_res[48:0], 1'b1};
                                    end else begin
                                        temp_mant_res = {temp_mant_res[48:0], 1'b0};
                                    end
                                    temp_mant_a = temp_mant_a << 1;
                                    temp_count  = temp_count - 1;
                                end
                                
                                FOP_SQRT: begin
                                    // Bóc 2 bit từ Radicand đẩy vào Remainder
                                    next_rem = {temp_mant_b[49:0], temp_mant_a[49:48]};
                                    // Giá trị trừ (Test value) = (Root << 2) | 1
                                    test_val = {temp_mant_res[49:0], 2'b01};
                                    
                                    if (next_rem >= test_val) begin
                                        temp_mant_b   = (next_rem - test_val); // Ép kiểu ngầm định về 50 bit
                                        temp_mant_res = (temp_mant_res << 1) | 1'b1;
                                    end else begin
                                        temp_mant_b   = next_rem[49:0];
                                        temp_mant_res = temp_mant_res << 1;
                                    end
                                    temp_mant_a = temp_mant_a << 2;
                                    temp_count  = temp_count - 1;
                                    
                                    if (temp_count == 0) begin
                                        // Dịch kết quả Root 26-bit hiện tại vào đúng khung căn lề ở bit thứ 48
                                        temp_mant_res = temp_mant_res << 23; 
                                    end
                                end

                                FOP_CVT_S_W, FOP_CVT_S_WU: begin
                                    if (temp_mant_a[31] == 1'b0 && temp_mant_a != 50'd0) begin
                                        temp_mant_a  = temp_mant_a << 1;
                                        temp_exp_res = temp_exp_res - 1;
                                        temp_count   = temp_count - 1;
                                    end else begin
                                        temp_mant_res = temp_mant_a << 17; 
                                        temp_count    = 0;
                                        sign_res      <= (op_reg == FOP_CVT_S_W) ? sign_a : 1'b0;
                                    end
                                end

                                FOP_CVT_W_S, FOP_CVT_WU_S: begin
                                    if (exp_a < 9'd127) begin 
                                        temp_mant_res = 50'd0;
                                        temp_count    = 0;
                                    end else begin
                                        shift_amt = 9'd150 - exp_a; 
                                        if (shift_amt > 0 && shift_amt < 31) begin
                                            temp_mant_res = temp_mant_a >> shift_amt;
                                        end else if (shift_amt <= 0) begin
                                            temp_mant_res = temp_mant_a << (-shift_amt);
                                        end
                                        temp_count = 0;
                                    end
                                end
                                
                                default: temp_count = 0;
                            endcase
                        end
                    end

                    mant_res        <= temp_mant_res;
                    mant_a          <= temp_mant_a;
                    mant_b          <= temp_mant_b;
                    exp_res         <= temp_exp_res;
                    iteration_count <= temp_count;

                    if (temp_count == 0) begin
                        if (op_reg == FOP_CVT_W_S || op_reg == FOP_CVT_WU_S) begin
                            if (op_reg == FOP_CVT_W_S && sign_a) begin
                                result <= ~(mant_res[31:0]) + 1'b1; 
                            end else begin
                                result <= mant_res[31:0];
                            end
                            state <= STATE_DONE;
                        end else begin
                            state <= STATE_NORMALIZE;
                        end
                    end
                end

                STATE_NORMALIZE: begin
                    if (mant_res == 50'd0) begin
                        result <= POS_ZERO;
                        state  <= STATE_DONE;
                    end else if (mant_res[49]) begin 
                        mant_res    <= mant_res >> 1;
                        mant_res[0] <= mant_res[0] | mant_res[1];
                        exp_res     <= exp_res + 1;
                    end else if (!mant_res[48]) begin 
                        mant_res <= mant_res << 1;
                        exp_res  <= exp_res - 1;
                    end else begin
                        state <= STATE_ROUND;
                    end
                end

                STATE_ROUND: begin
                    if (w_guard && (w_round || w_sticky || w_lsb)) begin
                        mant_res <= mant_res + 50'h02000000; 
                        if (mant_res[49]) begin 
                            mant_res <= mant_res >> 1;
                            exp_res  <= exp_res + 1;
                        end else begin
                            state <= STATE_PACK;
                        end
                    end else begin
                        state <= STATE_PACK;
                    end
                end

                STATE_PACK: begin
                    if (exp_res >= 9'd255) begin 
                        result <= {sign_res, 8'hFF, 23'b0};
                    end else if (exp_res[8] || exp_res == 9'd0) begin 
                        result <= POS_ZERO;
                    end else begin
                        result <= {sign_res, exp_res[7:0], mant_res[47:25]};
                    end
                    state <= STATE_DONE;
                end

                STATE_DONE: begin
                    fpu_done <= 1'b1;
                    state    <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end
endmodule