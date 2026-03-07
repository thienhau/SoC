// =============================================================================
// SUB-MODULE: ICache Data RAM (Lưu trữ lệnh)
// =============================================================================
module icache_data_ram (
    input clk,
    input [3:0] index,
    input [1:0] write_en, // [1] cho Way 2, [0] cho Way 1
    input [63:0] write_data,
    output [63:0] data1_out,
    output [63:0] data2_out
);
    // Ép Vivado sử dụng LUTRAM
    (* ram_style = "distributed" *) reg [63:0] data1 [0:15];
    (* ram_style = "distributed" *) reg [63:0] data2 [0:15];

    // Đọc bất đồng bộ (Asynchronous Read)
    assign data1_out = data1[index];
    assign data2_out = data2[index];

    // Ghi đồng bộ (Synchronous Write)
    always @(posedge clk) begin
        if (write_en[0]) data1[index] <= write_data;
        if (write_en[1]) data2[index] <= write_data;
    end
endmodule

// =============================================================================
// SUB-MODULE: ICache Tag RAM (Lưu trữ Tag)
// =============================================================================
module icache_tag_ram (
    input clk,
    input [3:0] index,
    input [1:0] write_en,
    input [4:0] write_tag,
    output [4:0] tag1_out,
    output [4:0] tag2_out
);
    (* ram_style = "distributed" *) reg [4:0] tag1 [0:15];
    (* ram_style = "distributed" *) reg [4:0] tag2 [0:15];

    assign tag1_out = tag1[index];
    assign tag2_out = tag2[index];

    always @(posedge clk) begin
        if (write_en[0]) tag1[index] <= write_tag;
        if (write_en[1]) tag2[index] <= write_tag;
    end
endmodule

// =============================================================================
// MAIN MODULE: Instruction Cache Controller
// =============================================================================
module instruction_cache (
    input clk, reset,
    input flush,
    input cpu_read_req,
    input [11:0] cpu_addr,
    input [63:0] mem_read_data,
    input mem_read_valid,
    output reg mem_read_req,
    output reg [11:0] mem_addr,
    output reg [31:0] cpu_read_data,
    output reg icache_hit,
    output reg icache_stall
);
    // Giải mã địa chỉ (12-bit)
    // Cấu trúc: [11:10] (unused/base) | [9:5] Tag (5 bit) | [4:1] Index (4 bit) | [0] Offset
    wire [9:0] address = cpu_addr[11:2];
    wire [4:0] tag = address[9:5];
    wire [3:0] index = address[4:1];
    wire word_offset = address[0];

    // Tín hiệu kết nối với module con
    wire [63:0] data1_out, data2_out;
    wire [4:0] tag1_out, tag2_out;
    
    // Metadata (Valid và PLRU) dùng Flip-Flops để Reset được
    reg valid1 [0:15];
    reg valid2 [0:15];
    reg plru [0:15];
    
    // FSM State
    parameter IDLE = 1'b0, MEM_READ = 1'b1;
    reg state = IDLE;
    reg next_state = IDLE;

    // Tín hiệu điều khiển nội bộ
    reg [1:0] way_update; // Bit [0] update Way 1, Bit [1] update Way 2

    // -------------------------------------------------------------------------
    // 1. Kết nối Module con
    // -------------------------------------------------------------------------
    icache_data_ram DATA_RAM (
        .clk(clk), 
        .index(index), 
        .write_en(way_update), 
        .write_data(mem_read_data),
        .data1_out(data1_out), 
        .data2_out(data2_out)
    );

    icache_tag_ram TAG_RAM (
        .clk(clk), 
        .index(index), 
        .write_en(way_update), 
        .write_tag(tag),
        .tag1_out(tag1_out), 
        .tag2_out(tag2_out)
    );

    // -------------------------------------------------------------------------
    // 2. Logic cập nhật Metadata (Valid bit & PLRU)
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            state <= IDLE;
            for (i=0; i<16; i=i+1) begin 
                valid1[i] <= 0; 
                valid2[i] <= 0; 
                plru[i] <= 0; 
            end
        end else if (flush) begin
            state <= IDLE;
        end else begin
            state <= next_state;
            
            // Cập nhật PLRU khi có Hit
            if (cpu_read_req && state == IDLE) begin
                if (valid1[index] && tag1_out == tag) plru[index] <= 1'b1;
                else if (valid2[index] && tag2_out == tag) plru[index] <= 1'b0;
            end 
            // Cập nhật Valid và PLRU khi nạp dòng mới (Refill)
            else if (way_update != 0) begin
                if (way_update[0]) begin 
                    valid1[index] <= 1; 
                    plru[index] <= 1; // Mới dùng Way 1 -> Victim tiếp theo ưu tiên Way 2 (0)
                end else if (way_update[1]) begin 
                    valid2[index] <= 1; 
                    plru[index] <= 0; // Mới dùng Way 2 -> Victim tiếp theo ưu tiên Way 1 (1)
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. Logic tổ hợp (FSM & Output)
    // -------------------------------------------------------------------------
    always @(*) begin
        // Giá trị mặc định
        next_state = state;
        icache_hit = 0; 
        icache_stall = 0; 
        mem_read_req = 0; 
        cpu_read_data = 0; 
        mem_addr = 0; 
        way_update = 2'b00;

        if (flush) begin
            icache_stall = 1;
        end else begin
            case (state)
                IDLE: begin
                    if (cpu_read_req) begin
                        // So sánh Tag
                        if (valid1[index] && tag1_out == tag) begin
                            icache_hit = 1;
                            // Chọn từ Word cao hoặc thấp trong block 64-bit
                            cpu_read_data = (word_offset == 0) ? data1_out[31:0] : data1_out[63:32];
                        end else if (valid2[index] && tag2_out == tag) begin
                            icache_hit = 1;
                            cpu_read_data = (word_offset == 0) ? data2_out[31:0] : data2_out[63:32];
                        end else begin
                            // Miss -> Chuyển sang đọc RAM
                            icache_stall = 1; 
                            mem_read_req = 1;
                            // Địa chỉ gửi xuống RAM = {Tag, Index, 0 offset, 00 byte align}
                            mem_addr = {tag, index, 1'b0, 2'b00};
                            next_state = MEM_READ;
                        end
                    end
                end

                MEM_READ: begin
                    icache_stall = 1; 
                    mem_read_req = 1;
                    mem_addr = {tag, index, 1'b0, 2'b00};
                    
                    if (mem_read_valid) begin
                        icache_stall = 0; 
                        mem_read_req = 0; 
                        next_state = IDLE;
                        
                        // Forward data ngay lập tức cho CPU để giảm trễ
                        cpu_read_data = (word_offset == 0) ? mem_read_data[31:0] : mem_read_data[63:32];
                        
                        // Chọn Way để ghi đè (Replacement Policy)
                        if (!valid1[index]) way_update[0] = 1;      // Way 1 trống
                        else if (!valid2[index]) way_update[1] = 1; // Way 2 trống
                        else if (plru[index] == 0) way_update[0] = 1; // PLRU trỏ Way 1
                        else way_update[1] = 1;                       // PLRU trỏ Way 2
                    end
                end
            endcase
        end
    end
endmodule