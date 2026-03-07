`timescale 1ns / 1ps

// =============================================================================
// SUB-MODULE: DCache Tag RAM (Lưu trữ 4-Way Tags) - 26 bit Tag cho địa chỉ 32 bit
// =============================================================================
module dcache_tag_ram (
    input  wire        clk,
    input  wire [3:0]  index,
    input  wire [3:0]  write_en, // One-hot: [0]=Way1 ... [3]=Way4
    input  wire [25:0] write_tag,
    output wire [25:0] t1, 
    output wire [25:0] t2, 
    output wire [25:0] t3, 
    output wire [25:0] t4
);
    (* ram_style = "distributed" *) reg [25:0] tag1 [0:15];
    (* ram_style = "distributed" *) reg [25:0] tag2 [0:15];
    (* ram_style = "distributed" *) reg [25:0] tag3 [0:15];
    (* ram_style = "distributed" *) reg [25:0] tag4 [0:15];

    assign t1 = tag1[index]; 
    assign t2 = tag2[index];
    assign t3 = tag3[index]; 
    assign t4 = tag4[index];

    always @(posedge clk) begin
        if (write_en[0]) tag1[index] <= write_tag;
        if (write_en[1]) tag2[index] <= write_tag;
        if (write_en[2]) tag3[index] <= write_tag;
        if (write_en[3]) tag4[index] <= write_tag;
    end
endmodule

// =============================================================================
// SUB-MODULE: DCache Data RAM (Lưu trữ 4-Way Data - 32 bit / Block)
// =============================================================================
module dcache_data_ram (
    input  wire        clk,
    input  wire [3:0]  index,
    input  wire [3:0]  write_en,
    input  wire [31:0] write_data,
    output wire [31:0] d1, 
    output wire [31:0] d2, 
    output wire [31:0] d3, 
    output wire [31:0] d4
);
    (* ram_style = "distributed" *) reg [31:0] data1 [0:15];
    (* ram_style = "distributed" *) reg [31:0] data2 [0:15];
    (* ram_style = "distributed" *) reg [31:0] data3 [0:15];
    (* ram_style = "distributed" *) reg [31:0] data4 [0:15];

    assign d1 = data1[index]; 
    assign d2 = data2[index];
    assign d3 = data3[index]; 
    assign d4 = data4[index];

    always @(posedge clk) begin
        if (write_en[0]) data1[index] <= write_data;
        if (write_en[1]) data2[index] <= write_data;
        if (write_en[2]) data3[index] <= write_data;
        if (write_en[3]) data4[index] <= write_data;
    end
endmodule

// =============================================================================
// MAIN MODULE: Data Cache Controller (32-bit Address Logic)
// =============================================================================
module data_cache (
    input  wire        clk, 
    input  wire        rst_n,          // Sửa thành Reset mức thấp
    input  wire        cpu_read_req, 
    input  wire        cpu_write_req,
    input  wire [31:0] cpu_addr,       // Sửa thành 32-bit
    input  wire [31:0] mem_read_data,
    input  wire [31:0] cpu_write_data,
    input  wire        mem_unsigned, 
    input  wire [1:0]  mem_size,
    input  wire        mem_read_valid, 
    input  wire        mem_write_back_valid,
    output reg         mem_read_req, 
    output reg         mem_write_req,
    output reg  [31:0] mem_addr,       // Sửa thành 32-bit
    output reg  [31:0] cpu_read_data,
    output reg  [31:0] mem_write_data,
    output reg         dcache_hit, 
    output reg         dcache_stall
);

    // --- KHAI BÁO CÁC HÀM (FUNCTIONS) TỐI ƯU ---

    // 1. Hàm đọc dữ liệu (Byte/Half/Word/Unsigned)
    function [31:0] read_data_with_size;
        input [31:0] data; 
        input [1:0]  size; 
        input [1:0]  offset; 
        input        unsigned_flag;
        reg   [31:0] result;
        begin
            case (size)
                2'b00: result = data; // Word
                2'b01: begin // Half-word
                    if (offset[1] == 1'b0) result = unsigned_flag ? {16'b0, data[15:0]} : {{16{data[15]}}, data[15:0]};
                    else                   result = unsigned_flag ? {16'b0, data[31:16]} : {{16{data[31]}}, data[31:16]};
                end
                2'b10: begin // Byte
                    case (offset)
                        2'b00: result = unsigned_flag ? {24'b0, data[7:0]}   : {{24{data[7]}}, data[7:0]};
                        2'b01: result = unsigned_flag ? {24'b0, data[15:8]}  : {{24{data[15]}}, data[15:8]};
                        2'b10: result = unsigned_flag ? {24'b0, data[23:16]} : {{24{data[23]}}, data[23:16]};
                        2'b11: result = unsigned_flag ? {24'b0, data[31:24]} : {{24{data[31]}}, data[31:24]};
                    endcase
                end
                default: result = data;
            endcase
            read_data_with_size = result;
        end
    endfunction

    // 2. Hàm chuẩn bị dữ liệu ghi (Chỉ ghi đè Byte/Half tương ứng)
    function [31:0] write_data_with_size;
        input [31:0] original_data; 
        input [31:0] write_data; 
        input [1:0]  size; 
        input [1:0]  offset;
        reg   [31:0] result;
        begin
            result = original_data;
            case (size)
                2'b00: result = write_data; // Word
                2'b01: begin // Half-word
                    if (offset[1] == 1'b0) result[15:0]  = write_data[15:0];
                    else                   result[31:16] = write_data[15:0];
                end
                2'b10: begin // Byte
                    case (offset)
                        2'b00: result[7:0]   = write_data[7:0];
                        2'b01: result[15:8]  = write_data[7:0];
                        2'b10: result[23:16] = write_data[7:0];
                        2'b11: result[31:24] = write_data[7:0];
                    endcase
                end
            endcase
            write_data_with_size = result;
        end
    endfunction

    // 3. Hàm chọn Way nạn nhân bằng Tree-PLRU
    function [1:0] select_replacement_way;
        input [2:0] plru_bits;
        reg   [1:0] way;
        begin
            if (plru_bits[0] == 1'b0) way = (plru_bits[1] == 1'b0) ? 2'b00 : 2'b01; // Gốc trái
            else                      way = (plru_bits[2] == 1'b0) ? 2'b10 : 2'b11; // Gốc phải
            select_replacement_way = way;
        end
    endfunction

    // 4. Hàm cập nhật Tree-PLRU
    function [2:0] update_plru;
        input [2:0] old_plru; 
        input [1:0] accessed_way;
        reg   [2:0] new_plru;
        begin
            new_plru = old_plru;
            case (accessed_way)
                2'b00: begin new_plru[0] = 1'b1; new_plru[1] = 1'b1; end
                2'b01: begin new_plru[0] = 1'b1; new_plru[1] = 1'b0; end
                2'b10: begin new_plru[0] = 1'b0; new_plru[2] = 1'b1; end
                2'b11: begin new_plru[0] = 1'b0; new_plru[2] = 1'b0; end
            endcase
            update_plru = new_plru;
        end
    endfunction

    // --- PHÂN TÍCH ĐỊA CHỈ 32-BIT VÀ BIẾN NỘI BỘ ---
    // Block size = 32 bits (4 bytes)
    // Cấu trúc: [31:6] Tag (26 bit) | [5:2] Index (4 bit) | [1:0] Byte Offset
    wire [25:0] tag         = cpu_addr[31:6];
    wire [3:0]  index       = cpu_addr[5:2];
    wire [1:0]  byte_offset = cpu_addr[1:0];

    wire [25:0] t1, t2, t3, t4;
    wire [31:0] d1, d2, d3, d4;
    
    // Metadata arrays (Valid, Dirty, PLRU)
    reg       valid1[0:15], valid2[0:15], valid3[0:15], valid4[0:15];
    reg       dirty1[0:15], dirty2[0:15], dirty3[0:15], dirty4[0:15];
    reg [2:0] plru[0:15];

    // FSM
    parameter IDLE = 2'b00, MEM_READ = 2'b01, MEM_WRITE_BACK = 2'b10;
    reg [1:0] state, next_state;

    reg [1:0]  target_way;       // Chọn Way nào để xử lý (Hit hoặc Replace)
    reg        victim_dirty;     // Cờ đánh dấu Way bị chọn thay thế đang dơ
    reg [3:0]  way_write_en;     // Truyền xuống Data/Tag RAM
    reg [31:0] final_write_data; // Dữ liệu sẽ đẩy vào Data RAM

    // Kết nối Sub-Modules
    dcache_tag_ram TAGS (
        .clk(clk), 
        .index(index), 
        .write_en(way_write_en), 
        .write_tag(tag), 
        .t1(t1), 
        .t2(t2), 
        .t3(t3), 
        .t4(t4)
    );

    dcache_data_ram DATA (
        .clk(clk), 
        .index(index), 
        .write_en(way_write_en), 
        .write_data(final_write_data), 
        .d1(d1), 
        .d2(d2), 
        .d3(d3), 
        .d4(d4)
    );

    // -------------------------------------------------------------------------
    // LOGIC CHUYỂN TRẠNG THÁI (FSM & METADATA SEQUENTIAL)
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            for(i=0; i<16; i=i+1) begin 
                valid1[i]<=1'b0; dirty1[i]<=1'b0; 
                valid2[i]<=1'b0; dirty2[i]<=1'b0; 
                valid3[i]<=1'b0; dirty3[i]<=1'b0; 
                valid4[i]<=1'b0; dirty4[i]<=1'b0; 
                plru[i]<=3'b000; 
            end
        end else begin
            state <= next_state;

            // Xóa Dirty bit chuẩn xác khi Ghi-trả xong
            if (state == MEM_WRITE_BACK && mem_write_back_valid) begin
                case(target_way)
                    2'd0: dirty1[index] <= 1'b0;
                    2'd1: dirty2[index] <= 1'b0;
                    2'd2: dirty3[index] <= 1'b0;
                    2'd3: dirty4[index] <= 1'b0;
                endcase
            end

            // Cập nhật Metadata khi có thao tác Ghi Thành Công (RAM Update)
            if (way_write_en != 4'b0000) begin
                plru[index] <= update_plru(plru[index], target_way);
                
                if (state == MEM_READ) begin 
                    // Fill Data từ Mem (Miss Read)
                    case(target_way)
                        2'd0: begin valid1[index]<=1'b1; dirty1[index]<=1'b0; end
                        2'd1: begin valid2[index]<=1'b1; dirty2[index]<=1'b0; end
                        2'd2: begin valid3[index]<=1'b1; dirty3[index]<=1'b0; end
                        2'd3: begin valid4[index]<=1'b1; dirty4[index]<=1'b0; end
                    endcase
                end else if (state == IDLE) begin 
                    // Cập nhật từ CPU (Write Hit hoặc Write Miss Allocate)
                    case(target_way)
                        2'd0: begin valid1[index]<=1'b1; dirty1[index]<=1'b1; end
                        2'd1: begin valid2[index]<=1'b1; dirty2[index]<=1'b1; end
                        2'd2: begin valid3[index]<=1'b1; dirty3[index]<=1'b1; end
                        2'd3: begin valid4[index]<=1'b1; dirty4[index]<=1'b1; end
                    endcase
                end
            end else if (state == IDLE && cpu_read_req && dcache_hit) begin
                // Nếu Read Hit, chỉ cập nhật PLRU (không cập nhật valid/dirty)
                plru[index] <= update_plru(plru[index], target_way);
            end
        end
    end

    // -------------------------------------------------------------------------
    // LOGIC TỔ HỢP (COMBINATIONAL)
    // -------------------------------------------------------------------------
    always @(*) begin
        // Reset ngõ ra
        next_state       = state;
        dcache_hit       = 1'b0; 
        dcache_stall     = 1'b0;
        mem_read_req     = 1'b0; 
        mem_write_req    = 1'b0; 
        mem_addr         = 32'b0;
        cpu_read_data    = 32'b0; 
        mem_write_data   = 32'b0;
        way_write_en     = 4'b0000; 
        final_write_data = 32'b0; 
        victim_dirty     = 1'b0;

        // 1. Hit/Miss Detection & Victim Selection
        if      (valid1[index] && t1 == tag) begin dcache_hit=1'b1; target_way=2'd0; end
        else if (valid2[index] && t2 == tag) begin dcache_hit=1'b1; target_way=2'd1; end
        else if (valid3[index] && t3 == tag) begin dcache_hit=1'b1; target_way=2'd2; end
        else if (valid4[index] && t4 == tag) begin dcache_hit=1'b1; target_way=2'd3; end
        else begin
            dcache_hit = 1'b0;
            // Thuật toán ưu tiên invalid way trước, sau đó dùng PLRU
            if      (!valid1[index]) target_way = 2'd0;
            else if (!valid2[index]) target_way = 2'd1;
            else if (!valid3[index]) target_way = 2'd2;
            else if (!valid4[index]) target_way = 2'd3;
            else                     target_way = select_replacement_way(plru[index]);
        end

        // 2. Kiểm tra Victim có Dirty không?
        case (target_way)
            2'd0: victim_dirty = dirty1[index] & valid1[index];
            2'd1: victim_dirty = dirty2[index] & valid2[index];
            2'd2: victim_dirty = dirty3[index] & valid3[index];
            2'd3: victim_dirty = dirty4[index] & valid4[index];
        endcase

        // 3. FSM Handling
        case (state)
            IDLE: begin
                if (cpu_read_req) begin
                    if (dcache_hit) begin
                        // --- READ HIT ---
                        case (target_way)
                            2'd0: cpu_read_data = read_data_with_size(d1, mem_size, byte_offset, mem_unsigned);
                            2'd1: cpu_read_data = read_data_with_size(d2, mem_size, byte_offset, mem_unsigned);
                            2'd2: cpu_read_data = read_data_with_size(d3, mem_size, byte_offset, mem_unsigned);
                            2'd3: cpu_read_data = read_data_with_size(d4, mem_size, byte_offset, mem_unsigned);
                        endcase
                    end else begin
                        // --- READ MISS ---
                        dcache_stall = 1'b1;
                        if (victim_dirty) next_state = MEM_WRITE_BACK;
                        else              next_state = MEM_READ;
                    end
                end else if (cpu_write_req) begin
                    if (dcache_hit) begin
                        // --- WRITE HIT ---
                        way_write_en[target_way] = 1'b1; // Kích hoạt ghi RAM
                        case (target_way)
                            2'd0: final_write_data = write_data_with_size(d1, cpu_write_data, mem_size, byte_offset);
                            2'd1: final_write_data = write_data_with_size(d2, cpu_write_data, mem_size, byte_offset);
                            2'd2: final_write_data = write_data_with_size(d3, cpu_write_data, mem_size, byte_offset);
                            2'd3: final_write_data = write_data_with_size(d4, cpu_write_data, mem_size, byte_offset);
                        endcase
                    end else begin
                        // --- WRITE MISS ---
                        if (victim_dirty) begin
                            dcache_stall = 1'b1;
                            next_state   = MEM_WRITE_BACK;
                        end else if (mem_size != 2'b00) begin
                            // Ghi Byte/Half thì phải Fetch Data về trước
                            dcache_stall = 1'b1;
                            next_state   = MEM_READ;
                        end else begin
                            // Write Allocate trực tiếp: Đè Full Word (mem_size = 2'b00)
                            dcache_stall             = 1'b0; // Không bị trễ
                            way_write_en[target_way] = 1'b1;
                            final_write_data         = cpu_write_data;
                        end
                    end
                end
            end

            MEM_READ: begin
                dcache_stall = 1'b1;
                mem_read_req = 1'b1;
                mem_addr     = {tag, index, 2'b00}; // Địa chỉ 32-bit thẳng hàng 4 Bytes

                if (mem_read_valid) begin
                    dcache_stall = 1'b0;
                    mem_read_req = 1'b0;
                    
                    // Nạp dữ liệu vào CACHE
                    way_write_en[target_way] = 1'b1;
                    final_write_data         = mem_read_data;
                    
                    // Đẩy dữ liệu luôn cho CPU (Data Forwarding)
                    cpu_read_data = read_data_with_size(mem_read_data, mem_size, byte_offset, mem_unsigned);
                    
                    next_state = IDLE;
                end
            end

            MEM_WRITE_BACK: begin
                dcache_stall  = 1'b1;
                mem_write_req = 1'b1;

                // Chuẩn bị địa chỉ 32-bit & dữ liệu cũ để đẩy ra bộ nhớ
                case (target_way)
                    2'd0: begin mem_addr = {t1, index, 2'b00}; mem_write_data = d1; end
                    2'd1: begin mem_addr = {t2, index, 2'b00}; mem_write_data = d2; end
                    2'd2: begin mem_addr = {t3, index, 2'b00}; mem_write_data = d3; end
                    2'd3: begin mem_addr = {t4, index, 2'b00}; mem_write_data = d4; end
                endcase

                if (mem_write_back_valid) begin
                    mem_write_req = 1'b0;
                    // Phân luồng tiếp theo
                    if (cpu_read_req || (cpu_write_req && mem_size != 2'b00))
                        next_state = MEM_READ;
                    else 
                        next_state = IDLE; // Quay về Write Allocate (nếu đang Write Word)
                end
            end
            
            default: next_state = IDLE;
        endcase
    end
endmodule