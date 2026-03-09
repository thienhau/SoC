`timescale 1ns / 1ps

// =============================================================================
// SUB-MODULE: ICache Data RAM (Lưu trữ lệnh - 64 bit / Block)
// =============================================================================
module icache_data_ram (
    input  wire        clk,
    input  wire [3:0]  index,
    input  wire [1:0]  write_en,
    input  wire [63:0] write_data,
    output wire [63:0] data1_out,
    output wire [63:0] data2_out
);
    (* ram_style = "distributed" *) reg [63:0] data1 [0:15];
    (* ram_style = "distributed" *) reg [63:0] data2 [0:15];

    assign data1_out = data1[index];
    assign data2_out = data2[index];

    always @(posedge clk) begin
        if (write_en[0]) data1[index] <= write_data;
        if (write_en[1]) data2[index] <= write_data;
    end
endmodule

// =============================================================================
// SUB-MODULE: ICache Tag RAM (Lưu trữ Tag - 25 bit Tag cho địa chỉ 32 bit)
// =============================================================================
module icache_tag_ram (
    input  wire        clk,
    input  wire [3:0]  index,
    input  wire [1:0]  write_en,
    input  wire [24:0] write_tag,
    output wire [24:0] tag1_out,
    output wire [24:0] tag2_out
);
    (* ram_style = "distributed" *) reg [24:0] tag1 [0:15];
    (* ram_style = "distributed" *) reg [24:0] tag2 [0:15];

    assign tag1_out = tag1[index];
    assign tag2_out = tag2[index];

    always @(posedge clk) begin
        if (write_en[0]) tag1[index] <= write_tag;
        if (write_en[1]) tag2[index] <= write_tag;
    end
endmodule

// =============================================================================
// MAIN MODULE: Instruction Cache Controller (AXI4 Burst Master)
// =============================================================================
module instruction_cache (
    input  wire        clk,
    input  wire        rst_n,          
    input  wire        flush,
    
    // Interface CPU
    input  wire        cpu_read_req,
    input  wire [31:0] cpu_addr,     
    output reg  [31:0] cpu_read_data,
    output reg         icache_hit,
    output reg         icache_stall,
    
    // Interface AXI4 Full (Chỉ dùng kênh Read)
    output reg  [31:0] m_axi_araddr,
    output wire [7:0]  m_axi_arlen,
    output wire [2:0]  m_axi_arsize,
    output wire [1:0]  m_axi_arburst,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,
    
    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output reg         m_axi_rready
);

    // Giải mã địa chỉ
    wire [24:0] tag         = cpu_addr[31:7];
    wire [3:0]  index       = cpu_addr[6:3];
    wire        word_offset = cpu_addr[2];

    wire [63:0] data1_out, data2_out;
    wire [24:0] tag1_out, tag2_out;
    
    reg valid1 [0:15];
    reg valid2 [0:15];
    reg plru   [0:15];

    // Cấu hình Burst cố định
    assign m_axi_arlen   = 8'd1;   // 2 nhịp (Len + 1)
    assign m_axi_arsize  = 3'b010; // 4 byte mỗi nhịp
    assign m_axi_arburst = 2'b01;  // INCR burst

    // Các trạng thái FSM
    localparam IDLE       = 3'd0;
    localparam AR_REQ     = 3'd1;
    localparam R_WAIT_1   = 3'd2;
    localparam R_WAIT_2   = 3'd3;
    localparam UPDATE_RAM = 3'd4;
    
    reg [2:0] state, next_state;
    reg [1:0] way_update;
    reg [63:0] fetch_buffer;
    
    // Khởi tạo RAM con
    icache_data_ram DATA_RAM (
        .clk(clk), 
        .index(index), 
        .write_en(way_update), 
        .write_data(fetch_buffer),
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

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            fetch_buffer <= 64'b0;
            for (i=0; i<16; i=i+1) begin 
                valid1[i] <= 1'b0;
                valid2[i] <= 1'b0; 
                plru[i]   <= 1'b0; 
            end
        end else if (flush) begin
            // Khi có flush, hủy bỏ dữ liệu đang nạp dở và về IDLE
            state <= IDLE;
            fetch_buffer <= 64'b0;
            for (i=0; i<16; i=i+1) begin 
                valid1[i] <= 1'b0;
                valid2[i] <= 1'b0; 
            end
        end else begin
            state <= next_state;

            // Cập nhật PLRU khi CPU đọc trúng (Hit)
            if (cpu_read_req && state == IDLE) begin
                if (valid1[index] && tag1_out == tag) plru[index] <= 1'b1;
                else if (valid2[index] && tag2_out == tag) plru[index] <= 1'b0;
            end 
            // Cập nhật PLRU và Valid sau khi nạp từ Bus
            else if (state == UPDATE_RAM) begin
                if (way_update[0]) begin 
                    valid1[index] <= 1'b1;
                    plru[index]   <= 1'b1; 
                end else if (way_update[1]) begin 
                    valid2[index] <= 1'b1;
                    plru[index]   <= 1'b0; 
                end
            end

            // Capture Data từ Bus AXI (Ghép 2 nhịp 32-bit thành 64-bit)
            if (state == R_WAIT_1 && m_axi_rvalid && m_axi_rready) begin
                fetch_buffer[31:0] <= m_axi_rdata;
            end else if (state == R_WAIT_2 && m_axi_rvalid && m_axi_rready) begin
                fetch_buffer[63:32] <= m_axi_rdata;
            end
        end
    end

    // Logic tổ hợp điều khiển FSM và tín hiệu Bus
    always @(*) begin
        // Giá trị mặc định
        next_state    = state;
        icache_hit    = 1'b0;
        icache_stall  = 1'b0; 
        cpu_read_data = 32'b0;
        way_update    = 2'b00;
        m_axi_arvalid = 1'b0;
        m_axi_araddr  = {tag, index, 3'b000}; // Luôn căn lề 8 byte
        m_axi_rready  = 1'b0;

        if (flush) begin
            // Khi flush: Ép dừng Bus, báo Stall và ép trạng thái về IDLE
            icache_stall  = 1'b1;
            m_axi_arvalid = 1'b0;
            m_axi_rready  = 1'b0;
            next_state    = IDLE;
        end else begin
            case (state)
                IDLE: begin
                    if (cpu_read_req) begin
                        if (valid1[index] && tag1_out == tag) begin
                            icache_hit    = 1'b1;
                            cpu_read_data = (word_offset == 1'b0) ? data1_out[31:0] : data1_out[63:32];
                        end else if (valid2[index] && tag2_out == tag) begin
                            icache_hit    = 1'b1;
                            cpu_read_data = (word_offset == 1'b0) ? data2_out[31:0] : data2_out[63:32];
                        end else begin
                            // Miss: Bắt đầu quy trình nạp từ Bus
                            icache_stall  = 1'b1;
                            next_state    = AR_REQ;
                        end
                    end
                end

                AR_REQ: begin
                    icache_stall  = 1'b1;
                    m_axi_arvalid = 1'b1;
                    if (m_axi_arready) begin
                        next_state = R_WAIT_1;
                    end
                end

                R_WAIT_1: begin
                    icache_stall = 1'b1;
                    m_axi_rready = 1'b1;
                    if (m_axi_rvalid) begin
                        next_state = R_WAIT_2;
                    end
                end

                R_WAIT_2: begin
                    icache_stall = 1'b1;
                    m_axi_rready = 1'b1;
                    if (m_axi_rvalid && m_axi_rlast) begin
                        next_state = UPDATE_RAM;
                    end
                end
                
                UPDATE_RAM: begin
                    icache_stall = 1'b1;
                    // Thuật toán thay thế PLRU
                    if (!valid1[index])      way_update[0] = 1'b1;
                    else if (!valid2[index]) way_update[1] = 1'b1;
                    else if (plru[index] == 1'b0) way_update[0] = 1'b1;
                    else                     way_update[1] = 1'b1;
                    
                    next_state = IDLE;
                end
                
                default: next_state = IDLE;
            endcase
        end
    end

endmodule