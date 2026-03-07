module pipeline_control_unit (
    input [6:0] opcode,
    input [2:0] funct3,
    input [4:0] rs1, rs2,
    input id_ex_mem_read, 
    input id_ex_jal, id_ex_jalr,
    input [4:0] id_ex_rd,
    input bpu_correct,
    input trap_enter, mret_exec,
    output reg load_use_stall, flush_branch, flush_jal, flush_trap
);
    reg load_use_hazard = 0;

    always @(*) begin
        load_use_stall = 0;
        flush_branch = 0;
        flush_jal = 0;
        flush_trap = 0;
        load_use_hazard = 0;

        // Load-use hazard detection (bao gồm cả FPU)
        if (id_ex_mem_read && (id_ex_rd != 0)) begin
            case (opcode)
                7'b0110011, 7'b0010011, 7'b0000011, 7'b0100011, 
                7'b1100011, 7'b1100111: begin // Integer ops
                    if ((id_ex_rd == rs1) || (id_ex_rd == rs2)) 
                        load_use_hazard = 1;
                end
                7'b1010011, 7'b1000011, 7'b1000111, 7'b1001011, 7'b1001111: begin // FPU ops
                    if ((id_ex_rd == rs1) || (id_ex_rd == rs2)) 
                        load_use_hazard = 1;
                end
                7'b0000111: begin // FLW
                    if (id_ex_rd == rs1) 
                        load_use_hazard = 1;
                end
                7'b0100111: begin // FSW
                    if ((id_ex_rd == rs1) || (id_ex_rd == rs2)) 
                        load_use_hazard = 1;
                end
            endcase
        end

        // Flush
        flush_branch = !bpu_correct;
        flush_jal = id_ex_jal || id_ex_jalr;
        flush_trap = trap_enter || mret_exec;

        // Load use stall
        load_use_stall = load_use_hazard && !flush_branch && !flush_jal;
    end
endmodule