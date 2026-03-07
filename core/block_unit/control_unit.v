module main_control_unit (
    input [6:0] opcode,
    input [6:0] funct7,
    input [2:0] funct3,
    input [4:0] rs2,
    output reg reg_write,
    output reg alu_src,
    output reg mem_write,
    output reg mem_read,
    output reg mem_to_reg,
    output reg branch,
    output reg jal,
    output reg jalr,
    output reg lui,
    output reg auipc,
    output reg mem_unsigned,
    output reg [1:0] alu_op,
    output reg [1:0] mem_size,
    output reg [2:0] md_operation,
    // FPU signals
    output reg fpu_en,
    output reg f_reg_write,
    output reg f_mem_to_reg,
    output reg f_mem_write,
    output reg f_to_x,
    output reg x_to_f,
    output reg [4:0] fpu_operation
);
    always @(*) begin 
        reg_write = 0;
        alu_src = 0;
        mem_write = 0;
        mem_read = 0;
        mem_to_reg = 0;
        branch = 0;
        jal = 0;
        jalr = 0;
        lui = 0;
        auipc = 0;
        alu_op = 2'b00;
        mem_size = 2'b00;
        mem_unsigned = 0;
        md_operation = 3'b000;
        // FPU defaults
        fpu_en = 0;
        f_reg_write = 0;
        f_mem_to_reg = 0;
        f_mem_write = 0;
        f_to_x = 0;
        x_to_f = 0;
        fpu_operation = 5'b00000;
        
        case (opcode)
            7'b0110011: begin // R-type
                reg_write = 1;
                alu_op = 2'b10;
                if (funct7 == 7'b0000001) begin
                    case (funct3)
                        3'b000: md_operation = 3'b000; // MUL
                        3'b001: md_operation = 3'b001; // MULH
                        3'b010: md_operation = 3'b010; // MULHSU
                        3'b011: md_operation = 3'b011; // MULHU
                        3'b100: md_operation = 3'b100; // DIV
                        3'b101: md_operation = 3'b101; // DIVU
                        3'b110: md_operation = 3'b110; // REM
                        3'b111: md_operation = 3'b111; // REMU
                        default: md_operation = 3'b000;
                    endcase
                end
            end
            7'b0010011: begin // I-type (ALU)
                reg_write = 1;
                alu_src = 1;
                alu_op = 2'b10;
            end
            7'b0000011: begin // Load
                alu_src = 1;
                mem_read = 1;
                mem_to_reg = 1;
                reg_write = 1;
                alu_op = 2'b00;
                case (funct3)
                    3'b000: begin // LB
                        mem_size = 2'b10;
                        mem_unsigned = 0;
                    end
                    3'b001: begin // LH
                        mem_size = 2'b01;
                        mem_unsigned = 0;
                    end
                    3'b010: begin // LW
                        mem_size = 2'b00;
                        mem_unsigned = 0;
                    end
                    3'b100: begin // LBU
                        mem_size = 2'b10;
                        mem_unsigned = 1;
                    end
                    3'b101: begin // LHU
                        mem_size = 2'b01;
                        mem_unsigned = 1;
                    end
                    default: begin
                        mem_size = 2'b00;
                        mem_unsigned = 0;
                    end
                endcase
            end
            7'b0100011: begin // Store
                alu_src = 1;
                mem_write = 1;
                alu_op = 2'b00;
                case (funct3)
                    3'b000: mem_size = 2'b10; // SB
                    3'b001: mem_size = 2'b01; // SH
                    3'b010: mem_size = 2'b00; // SW
                    default: mem_size = 2'b00;
                endcase
            end
            7'b1100011: begin // Branch
                branch = 1;
                alu_op = 2'b01;
            end
            7'b0110111: begin // LUI
                lui = 1;
                reg_write = 1;
            end
            7'b0010111: begin // AUIPC
                auipc = 1;
                reg_write = 1;
            end
            7'b1101111: begin // JAL
                jal = 1;
                reg_write = 1;
            end
            7'b1100111: begin // JALR
                jalr = 1;
                reg_write = 1;
                alu_src = 1;
            end
            7'b1110011: begin // System
                if (funct3 != 3'b000) reg_write = 1;
            end
            7'b0000111: begin // FLW
                alu_src = 1;
                mem_read = 1;
                f_mem_to_reg = 1;
                f_reg_write = 1;
                alu_op = 2'b00;
                mem_size = 2'b00; // Word
            end
            7'b0100111: begin // FSW
                alu_src = 1;
                mem_write = 1;
                f_mem_write = 1;
                alu_op = 2'b00;
                mem_size = 2'b00; // Word
            end
            7'b1010011: begin // FPU Operations
                fpu_en = 1;
                case (funct7)
                    7'b0000000: begin // FADD.S
                        f_reg_write = 1;
                        fpu_operation = 5'b00000;
                    end
                    7'b0000100: begin // FSUB.S
                        f_reg_write = 1;
                        fpu_operation = 5'b00001;
                    end
                    7'b0001000: begin // FMUL.S
                        f_reg_write = 1;
                        fpu_operation = 5'b00010;
                    end
                    7'b0001100: begin // FDIV.S
                        f_reg_write = 1;
                        fpu_operation = 5'b01000;
                    end
                    7'b0101100: begin // FSQRT.S
                        f_reg_write = 1;
                        fpu_operation = 5'b01001;
                    end
                    7'b0010000: begin // FSGNJ
                        f_reg_write = 1;
                        case (funct3)
                            3'b000: fpu_operation = 5'b01100; // FSGNJ
                            3'b001: fpu_operation = 5'b01101; // FSGNJN
                            3'b010: fpu_operation = 5'b01110; // FSGNJX
                            default: fpu_operation = 5'b01100;
                        endcase
                    end
                    7'b0010100: begin // FMIN/FMAX
                        f_reg_write = 1;
                        case (funct3)
                            3'b000: fpu_operation = 5'b01010; // FMIN
                            3'b001: fpu_operation = 5'b01011; // FMAX
                            default: fpu_operation = 5'b01010;
                        endcase
                    end
                    7'b1010000: begin // FEQ/FLT/FLE
                        f_to_x = 1;
                        reg_write = 1;
                        case (funct3)
                            3'b010: fpu_operation = 5'b00101; // FEQ
                            3'b001: fpu_operation = 5'b00110; // FLT
                            3'b000: fpu_operation = 5'b00111; // FLE
                            default: fpu_operation = 5'b00101;
                        endcase
                    end
                    7'b1100000: begin // FCVT.W.S / FCVT.WU.S
                        f_to_x = 1;
                        reg_write = 1;
                        if (rs2[0])
                            fpu_operation = 5'b10010; // FCVT.WU.S
                        else
                            fpu_operation = 5'b00011; // FCVT.W.S
                    end
                    7'b1101000: begin // FCVT.S.W / FCVT.S.WU
                        x_to_f = 1;
                        f_reg_write = 1;
                        if (rs2[0])
                            fpu_operation = 5'b10011; // FCVT.S.WU
                        else
                            fpu_operation = 5'b00100; // FCVT.S.W
                    end
                    7'b1110000: begin // FMV.X.W / FCLASS.S
                        f_to_x = 1;
                        reg_write = 1;
                        case (funct3)
                            3'b000: fpu_operation = 5'b01111; // FMV.X.W
                            3'b001: fpu_operation = 5'b10001; // FCLASS.S
                            default: fpu_operation = 5'b01111;
                        endcase
                    end
                    7'b1111000: begin // FMV.W.X
                        x_to_f = 1;
                        f_reg_write = 1;
                        fpu_operation = 5'b10000; // FMV.W.X
                    end
                    default: fpu_en = 0;
                endcase
            end
            7'b1000011: begin // FMADD.S
                fpu_en = 1;
                f_reg_write = 1;
                fpu_operation = 5'b10100;
            end
            7'b1000111: begin // FMSUB.S
                fpu_en = 1;
                f_reg_write = 1;
                fpu_operation = 5'b10101;
            end
            7'b1001011: begin // FNMSUB.S
                fpu_en = 1;
                f_reg_write = 1;
                fpu_operation = 5'b10110;
            end
            7'b1001111: begin // FNMADD.S
                fpu_en = 1;
                f_reg_write = 1;
                fpu_operation = 5'b10111;
            end
            default: begin
                // Do nothing
            end
        endcase
    end
endmodule

module alu_control_unit (
    input [1:0] alu_op,
    input [2:0] funct3,
    input [6:0] funct7, opcode,
    output reg [3:0] alu_ctrl
);
    always @(*) begin
        case (alu_op)
            2'b00: alu_ctrl = 4'b0010; // add (for loads/stores)
            2'b01: alu_ctrl = 4'b0110; // subtract (for branches)
            2'b10: begin // R-type and I-type
                if (opcode == 7'b0010011) begin // I-type
                    case (funct3)
                        3'b000: alu_ctrl = 4'b0010; // ADDI
                        3'b010: alu_ctrl = 4'b0111; // SLTI
                        3'b011: alu_ctrl = 4'b1010; // SLTIU
                        3'b100: alu_ctrl = 4'b0100; // XORI
                        3'b110: alu_ctrl = 4'b0001; // ORI
                        3'b111: alu_ctrl = 4'b0000; // ANDI
                        3'b001: alu_ctrl = 4'b1000; // SLLI
                        3'b101: alu_ctrl = (funct7[5] ? 4'b1011 : 4'b1001); // SRAI/SRLI
                        default: alu_ctrl = 4'b0010;
                    endcase
                end else begin // R-type
                    case (funct3)
                        3'b000: alu_ctrl = (funct7[5] ? 4'b0110 : 4'b0010); // SUB/ADD
                        3'b001: alu_ctrl = 4'b1000; // SLL
                        3'b010: alu_ctrl = 4'b0111; // SLT
                        3'b011: alu_ctrl = 4'b1010; // SLTU
                        3'b100: alu_ctrl = 4'b0100; // XOR
                        3'b101: alu_ctrl = (funct7[5] ? 4'b1011 : 4'b1001); // SRA/SRL
                        3'b110: alu_ctrl = 4'b0001; // OR
                        3'b111: alu_ctrl = 4'b0000; // AND
                        default: alu_ctrl = 4'b0010;
                    endcase
                end
            end
            default: alu_ctrl = 4'b0010;
        endcase
    end
endmodule