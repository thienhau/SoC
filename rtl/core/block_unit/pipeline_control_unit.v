//==================================================================================================
// File: pipeline_control_unit.v
//==================================================================================================

module pipeline_control_unit (
    input clk,                 // Bổ sung clk để xử lý FSM xả pipeline
    input reset_n,             // Bổ sung reset
    
    input [6:0] opcode,
    input [2:0] funct3,
    input [4:0] rs1,
    input [4:0] rs2,
    input id_ex_mem_read, 
    input id_ex_jal,
    input id_ex_jalr,
    input [4:0] id_ex_rd,
    input bpu_correct,
    input trap_enter,
    input mret_exec,
    
    // --- TÍN HIỆU TỪ DEBUG MODULE ---
    input  wire dbg_halt_req,
    input  wire dbg_resume_req,
    output wire dbg_halted,
    
    // --- TÍN HIỆU PIPELINE GỐC & MỞ RỘNG ---
    output reg  load_use_stall,
    output reg  flush_branch,
    output reg  flush_jal,
    output reg  flush_trap,
    output wire stall_IF,
    output wire stall_ID,
    output wire stall_EX,
    output wire stall_MEM,
    output wire stall_WB
);

    reg load_use_hazard;

    always @(*) begin
        load_use_hazard = 1'b0;

        if (id_ex_mem_read && (id_ex_rd != 5'd0)) begin
            case (opcode)
                7'b0110011, 7'b0010011, 7'b0000011, 7'b0100011, 7'b1100011, 7'b1100111: begin
                    if ((id_ex_rd == rs1) || (id_ex_rd == rs2)) begin
                        load_use_hazard = 1'b1;
                    end
                end
                7'b1010011, 7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: begin
                    if ((id_ex_rd == rs1) || (id_ex_rd == rs2)) begin
                        load_use_hazard = 1'b1;
                    end
                end
                7'b0000111: begin
                    if (id_ex_rd == rs1) begin
                        load_use_hazard = 1'b1;
                    end
                end
                7'b0100111: begin
                    if ((id_ex_rd == rs1) || (id_ex_rd == rs2)) begin
                        load_use_hazard = 1'b1;
                    end
                end
            endcase
        end
    end

    always @(*) begin
        flush_branch = !bpu_correct;
        flush_jal = id_ex_jal || id_ex_jalr;
        flush_trap = trap_enter || mret_exec;
        load_use_stall = load_use_hazard && !flush_branch && !flush_jal;
    end

    // =========================================================================
    // LOGIC ĐÓNG BĂNG (HALT) DÀNH CHO DEBUG MODULE
    // =========================================================================
    reg [2:0] drain_cnt;
    reg dbg_halted_reg;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            dbg_halted_reg <= 1'b0;
            drain_cnt      <= 3'd0;
        end else begin
            if (dbg_resume_req) begin
                dbg_halted_reg <= 1'b0;
                drain_cnt      <= 3'd0;
            end else if (dbg_halt_req && !dbg_halted_reg) begin
                // Đợi xả các tầng ID, EX, MEM, WB (4 chu kỳ) để bảo toàn dữ liệu
                if (drain_cnt < 3'd4) begin
                    drain_cnt <= drain_cnt + 1;
                end else begin
                    dbg_halted_reg <= 1'b1;
                end
            end
        end
    end

    assign dbg_halted = dbg_halted_reg;

    // =========================================================================
    // XUẤT TÍN HIỆU STALL CHO CÁC TẦNG PIPELINE
    // =========================================================================
    // Lệnh mới ngay lập tức bị chặn ở tầng IF khi có yêu cầu Halt
    assign stall_IF  = load_use_stall || dbg_halt_req;
    
    // Các tầng bên dưới chỉ bị stall khi toàn bộ rác trong ống đã được xả xong
    assign stall_ID  = load_use_stall || dbg_halted_reg;
    assign stall_EX  = dbg_halted_reg;
    assign stall_MEM = dbg_halted_reg;
    assign stall_WB  = dbg_halted_reg;

endmodule