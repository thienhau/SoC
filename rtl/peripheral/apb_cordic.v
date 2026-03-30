`timescale 1ns / 1ps

module apb_cordic (
    // Tín hiệu hệ thống
    input  wire        PCLK,
    input  wire        PRESETn,

    // Giao tiếp APB
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [7:0]  PADDR,
    input  wire [31:0] PWDATA,
    output reg  [31:0] PRDATA,
    output wire        PREADY,
    output wire        PSLVERR
);

    // Bảng bộ nhớ tra cứu Atan cho 16 vòng lặp (Định dạng Q16.16 theo độ)
    wire [31:0] atan_table [0:15];
    assign atan_table[0]  = 32'd2949120; // 45.0000 * 65536
    assign atan_table[1]  = 32'd1740967; // 26.5650 * 65536
    assign atan_table[2]  = 32'd919879;  // 14.0362 * 65536
    assign atan_table[3]  = 32'd466945;  // 7.1250 * 65536
    assign atan_table[4]  = 32'd234378;  // 3.5763 * 65536
    assign atan_table[5]  = 32'd117303;  // 1.7899 * 65536
    assign atan_table[6]  = 32'd58666;   // 0.8951 * 65536
    assign atan_table[7]  = 32'd29334;   // 0.4476 * 65536
    assign atan_table[8]  = 32'd14667;   // 0.2238 * 65536
    assign atan_table[9]  = 32'd7333;    // 0.1119 * 65536
    assign atan_table[10] = 32'd3666;    // 0.0559 * 65536
    assign atan_table[11] = 32'd1833;    // 0.0279 * 65536
    assign atan_table[12] = 32'd916;     // 0.0139 * 65536
    assign atan_table[13] = 32'd458;     // 0.0069 * 65536
    assign atan_table[14] = 32'd229;     // 0.0034 * 65536
    assign atan_table[15] = 32'd114;     // 0.0017 * 65536

    // Thanh ghi nội bộ (Memory Mapped Registers)
    reg [31:0] reg_ctrl;   // 0x00: Bit 0 = Start
    reg [31:0] reg_status; // 0x04: Bit 0 = Done, Bit 1 = Busy
    reg [31:0] reg_xin;    // 0x08: X_in (Scale factor / Giá trị khởi tạo X)
    reg [31:0] reg_yin;    // 0x0C: Y_in (Giá trị khởi tạo Y)
    reg [31:0] reg_zin;    // 0x10: Z_in (Góc đầu vào)
    reg [31:0] reg_xout;   // 0x14: X_out (Kết quả Cos)
    reg [31:0] reg_yout;   // 0x18: Y_out (Kết quả Sin)
    reg [31:0] reg_zout;   // 0x1C: Z_out (Phần dư của góc)

    // Tín hiệu nội bộ cho CORDIC FSM
    localparam IDLE = 2'b00;
    localparam CALC = 2'b01;
    localparam DONE = 2'b10;

    reg [1:0]  state, next_state;
    reg [4:0]  iter_cnt;
    reg signed [31:0] x_reg, y_reg, z_reg;
    reg signed [31:0] x_next, y_next, z_next;
    wire d; 

    // Mặc định luôn sẵn sàng cho APB transaction
    assign PREADY  = 1'b1;
    assign PSLVERR = 1'b0;

    // -------------------------------------------------------------------------
    // APB Write Logic
    // -------------------------------------------------------------------------
    wire apb_write_en = PSEL & PENABLE & PWRITE;

    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            reg_ctrl <= 32'd0;
            reg_xin  <= 32'd39814; // Hằng số K (Khoảng 0.6072 * 65536) để khử Gain
            reg_yin  <= 32'd0;
            reg_zin  <= 32'd0;
        end else begin
            // Tự động clear bit START sau 1 chu kỳ để tạo xung
            if (reg_ctrl[0]) begin
                reg_ctrl[0] <= 1'b0;
            end

            if (apb_write_en) begin
                case (PADDR[7:0])
                    8'h00: reg_ctrl <= PWDATA;
                    8'h08: reg_xin  <= PWDATA;
                    8'h0C: reg_yin  <= PWDATA;
                    8'h10: reg_zin  <= PWDATA;
                    default: ; // Không làm gì ở các địa chỉ khác
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // APB Read Logic
    // -------------------------------------------------------------------------
    wire apb_read_en = PSEL & (~PWRITE);

    always @(*) begin
        if (apb_read_en) begin
            case (PADDR[7:0])
                8'h00: PRDATA = reg_ctrl;
                8'h04: PRDATA = reg_status;
                8'h08: PRDATA = reg_xin;
                8'h0C: PRDATA = reg_yin;
                8'h10: PRDATA = reg_zin;
                8'h14: PRDATA = reg_xout;
                8'h18: PRDATA = reg_yout;
                8'h1C: PRDATA = reg_zout;
                default: PRDATA = 32'd0;
            endcase
        end else begin
            PRDATA = 32'd0;
        end
    end

    // -------------------------------------------------------------------------
    // CORDIC Core FSM & Datapath
    // -------------------------------------------------------------------------
    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            state      <= IDLE;
            iter_cnt   <= 5'd0;
            x_reg      <= 32'd0;
            y_reg      <= 32'd0;
            z_reg      <= 32'd0;
            reg_xout   <= 32'd0;
            reg_yout   <= 32'd0;
            reg_zout   <= 32'd0;
            reg_status <= 32'd0;
        end else begin
            case (state)
                IDLE: begin
                    reg_status[0] <= 1'b0; // Clear Done
                    reg_status[1] <= 1'b0; // Clear Busy
                    iter_cnt      <= 5'd0;
                    
                    if (reg_ctrl[0]) begin // Nhận xung Start
                        state         <= CALC;
                        reg_status[1] <= 1'b1; // Set Busy
                        x_reg         <= reg_xin;
                        y_reg         <= reg_yin;
                        z_reg         <= reg_zin;
                    end
                end

                CALC: begin
                    x_reg <= x_next;
                    y_reg <= y_next;
                    z_reg <= z_next;
                    
                    if (iter_cnt == 5'd15) begin
                        state <= DONE;
                    end else begin
                        iter_cnt <= iter_cnt + 1'b1;
                    end
                end

                DONE: begin
                    reg_xout      <= x_reg;
                    reg_yout      <= y_reg;
                    reg_zout      <= z_reg;
                    reg_status[0] <= 1'b1; // Set Done flag
                    reg_status[1] <= 1'b0; // Clear Busy
                    state         <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

    // Quyết định hướng quay (Rotation Mode) d = 1 nếu z >= 0, d = 0 nếu z < 0
    assign d = (z_reg >= 0);

    // Tính toán vòng lặp CORDIC bằng phép dịch bit
    always @(*) begin
        if (d) begin
            x_next = x_reg - (y_reg >>> iter_cnt);
            y_next = y_reg + (x_reg >>> iter_cnt);
            z_next = z_reg - atan_table[iter_cnt];
        end else begin
            x_next = x_reg + (y_reg >>> iter_cnt);
            y_next = y_reg - (x_reg >>> iter_cnt);
            z_next = z_reg + atan_table[iter_cnt];
        end
    end

endmodule