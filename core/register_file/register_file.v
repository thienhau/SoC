module register_file (
    input clk, reset,
    input [4:0] read_reg1, read_reg2,
    input mem_wb_reg_write,
    input [4:0] mem_wb_rd,
    input [31:0] mem_wb_write_data,
    output [31:0] read_data1, read_data2
);
    // 1. Mảng chính: Ép vào LUTRAM (Không có Reset)
    (* ram_style = "distributed" *) reg [31:0] rf_main [0:31];

    // 2. Thanh ghi đặc biệt: Dùng Flip-Flops (Có Reset)
    reg [31:0] x2_sp;

    // Ghi dữ liệu
    always @(posedge clk) begin
        if (reset) begin
            x2_sp <= 32'd4096;
        end else if (mem_wb_reg_write) begin
            if (mem_wb_rd == 5'd2) 
                x2_sp <= mem_wb_write_data;
            else if (mem_wb_rd != 5'd0)
                rf_main[mem_wb_rd] <= mem_wb_write_data;
        end
    end

    // Đọc dữ liệu (Mux chọn giữa x0, x2 và phần còn lại)
    assign read_data1 = (read_reg1 == 5'd0) ? 32'd0 :
                        (read_reg1 == 5'd2) ? x2_sp : rf_main[read_reg1];
                        
    assign read_data2 = (read_reg2 == 5'd0) ? 32'd0 :
                        (read_reg2 == 5'd2) ? x2_sp : rf_main[read_reg2];
endmodule

module f_register_file (
    input clk, 
    input reset,
    input [4:0] read_reg1, 
    input [4:0] read_reg2,
    input [4:0] read_reg3,
    output [31:0] read_data1, 
    output [31:0] read_data2,
    output [31:0] read_data3,
    input reg_write_en,   
    input [4:0] write_reg,    
    input [31:0] write_data    
);

    // Ép Vivado sử dụng LUTRAM
    (* ram_style = "distributed" *) reg [31:0] f_regfile [0:31];
    
    // Đọc bất đồng bộ (Asynchronous Read)
    assign read_data1 = f_regfile[read_reg1];
    assign read_data2 = f_regfile[read_reg2];
    assign read_data3 = f_regfile[read_reg3];
    
    integer i;
    // Ghi đồng bộ (Synchronous Write) + Reset bằng initial
    always @(posedge clk) begin
        if (reset) begin
            // Reset tất cả về 0 (dùng vòng lặp)
            for (i = 0; i < 32; i = i + 1) begin
                f_regfile[i] <= 32'b0;
            end
        end else if (reg_write_en) begin
            f_regfile[write_reg] <= write_data;
        end
    end
endmodule

module csr_register_file (
    input clk, reset,
    
    // CSR read/write interface
    input [11:0] csr_addr,          // Địa chỉ đọc (EX stage)
    input [11:0] csr_write_addr,    // Địa chỉ ghi (MEM/WB stage)
    input [31:0] csr_write_data,    // Dữ liệu sẽ ghi vào CSR
    input [1:0]  csr_op,            // 00=none, 01=RW, 10=RS(set), 11=RC(clear)
    input        csr_write_en,      // Tín hiệu cho phép ghi
    output reg [31:0] csr_read_data,// Dữ liệu đọc ra
    
    // Performance counters
    input        count_en,          // Cho phép đếm mcycle
    input        instret_en,        // Lệnh hoàn thành (cộng minstret)
    
    // Trap interface
    input        trap_enter,        // Tín hiệu vào Trap (từ ecall, ebreak, lỗi)
    input        mret_exec,         // Lệnh mret thực thi
    input [31:0] trap_cause,        // Mã nguyên nhân (mcause)
    input [11:0] trap_pc,           // PC của lệnh gây lỗi/ngắt
    input [31:0] trap_val,          // Giá trị lỗi (mtval)
    
    output [11:0] mtvec_out,        // Địa chỉ hàm xử lý ngắt (vector)
    output [11:0] mepc_out,         // Địa chỉ PC trả về
    output [31:0] mie_out,          // Xuất toàn bộ thanh ghi mie
    output        mstatus_mie       // Cờ cho phép ngắt toàn cục (mstatus[3])
);

    // Machine Information (read-only)
    localparam [31:0] MVENDORID  = 32'h0;
    localparam [31:0] MARCHID    = 32'h0;
    localparam [31:0] MIMPID     = 32'h01000000;
    localparam [31:0] MHARTID    = 32'h0;
    localparam [31:0] MISA       = 32'h40000100; // RV32I (bit 8 = I), MXL=1

    // Machine Trap Setup & Handling
    reg [31:0] mstatus;     // 0x300
    reg [31:0] mie;         // 0x304
    reg [31:0] mtvec;       // 0x305
    reg [31:0] mscratch;    // 0x340
    reg [31:0] mepc;        // 0x341
    reg [31:0] mcause;      // 0x342
    reg [31:0] mtval;       // 0x343
    reg [31:0] mip;         // 0x344

    // Machine Counters
    reg [63:0] mcycle;      // 0xB00/0xB80
    reg [63:0] minstret;    // 0xB02/0xB82

    assign mtvec_out   = mtvec[11:0];
    assign mepc_out    = mepc[11:0];
    assign mstatus_mie = mstatus[3];
    assign mie_out     = mie;
    
    // Tổ hợp đọc CSR
    always @(*) begin
        case (csr_addr)
            // Machine Info
            12'hF11: csr_read_data = MVENDORID;
            12'hF12: csr_read_data = MARCHID;
            12'hF13: csr_read_data = MIMPID;
            12'hF14: csr_read_data = MHARTID;
            12'h301: csr_read_data = MISA;
            // Trap Setup & Handling
            12'h300: csr_read_data = mstatus;
            12'h304: csr_read_data = mie;
            12'h305: csr_read_data = mtvec;
            12'h340: csr_read_data = mscratch;
            12'h341: csr_read_data = mepc;
            12'h342: csr_read_data = mcause;
            12'h343: csr_read_data = mtval;
            12'h344: csr_read_data = mip;
            // Counters
            12'hB00, 12'hC00, 12'hC01: csr_read_data = mcycle[31:0];
            12'hB80, 12'hC80, 12'hC81: csr_read_data = mcycle[63:32];
            12'hB02, 12'hC02:          csr_read_data = minstret[31:0];
            12'hB82, 12'hC82:          csr_read_data = minstret[63:32];
            default: csr_read_data = 32'b0;
        endcase
    end
    
    // Tuần tự ghi CSR và xử lý Trap
    always @(posedge clk) begin
        if (reset) begin
            mstatus  <= 32'h00001800; // MPP=11, MIE=0
            mie      <= 32'b0;
            mtvec    <= 32'b0;
            mscratch <= 32'b0;
            mepc     <= 32'b0;
            mcause   <= 32'b0;
            mtval    <= 32'b0;
            mip      <= 32'b0;
            mcycle   <= 64'b0;
            minstret <= 64'b0;
        end else begin
            // Cập nhật bộ đếm
            if (count_en)   mcycle <= mcycle + 1;
            if (instret_en) minstret <= minstret + 1;
            
            // Xử lý Trap (Ưu tiên cao nhất)
            if (trap_enter) begin
                mepc   <= {14'b0, trap_pc};
                mcause <= trap_cause;
                mtval  <= trap_val;
                mstatus[7] <= mstatus[3]; // MPIE = MIE
                mstatus[3] <= 1'b0;       // MIE = 0
                mstatus[12:11] <= 2'b11;  // MPP = Machine
            end
            // Xử lý MRET
            else if (mret_exec) begin
                mstatus[3] <= mstatus[7]; // MIE = MPIE
                mstatus[7] <= 1'b1;       // MPIE = 1
                mstatus[12:11] <= 2'b11;
            end
            // Ghi CSR từ phần mềm
            else if (csr_write_en && csr_op != 2'b00) begin
                case (csr_write_addr)
                    12'h300: begin
                        mstatus[3]     <= csr_write_data[3];
                        mstatus[7]     <= csr_write_data[7];
                        mstatus[12:11] <= csr_write_data[12:11];
                    end
                    12'h304: mie      <= csr_write_data & 32'h00000888;
                    12'h305: mtvec    <= csr_write_data;
                    12'h340: mscratch <= csr_write_data;
                    12'h341: mepc     <= csr_write_data;
                    12'h342: mcause   <= csr_write_data;
                    12'h343: mtval    <= csr_write_data;
                    12'hB00: mcycle[31:0]   <= csr_write_data;
                    12'hB80: mcycle[63:32]  <= csr_write_data;
                    12'hB02: minstret[31:0] <= csr_write_data;
                    12'hB82: minstret[63:32]<= csr_write_data;
                endcase
            end
        end
    end
endmodule