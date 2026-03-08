`timescale 1ns / 1ps

// =============================================================================
// SUB-MODULE: ICache Data RAM (Lưu trữ lệnh - 64 bit / Block)
// =============================================================================
module icache_data_ram (
    input  wire        clk,
    input  wire [3:0]  index,
    input  wire [1:0]  write_en, // [1] cho Way 2, [0] cho Way 1
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
// MAIN MODULE: Instruction Cache Controller (32-bit Address)
// =============================================================================
module instruction_cache (
    input  wire        clk,
    input  wire        rst_n,          
    input  wire        flush,
    input  wire        cpu_read_req,
    input  wire [31:0] cpu_addr,     
    input  wire [63:0] mem_read_data,
    input  wire        mem_read_ready,       // BỔ SUNG: Tín hiệu báo Bus đã nhận request
    input  wire        mem_read_valid,
    output reg         mem_read_req,
    output reg  [31:0] mem_addr,       
    output reg  [31:0] cpu_read_data,
    output reg         icache_hit,
    output reg         icache_stall
);

    wire [24:0] tag         = cpu_addr[31:7];
    wire [3:0]  index       = cpu_addr[6:3];
    wire        word_offset = cpu_addr[2];

    wire [63:0] data1_out, data2_out;
    wire [24:0] tag1_out, tag2_out;
    
    reg valid1 [0:15];
    reg valid2 [0:15];
    reg plru   [0:15];
    
    parameter IDLE = 1'b0, MEM_READ = 1'b1;
    reg state, next_state;

    reg [1:0] way_update;
    
    // BỔ SUNG: Biến lưu trạng thái đã gửi Request thành công
    reg req_sent; 

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
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            req_sent <= 1'b0;
            for (i=0; i<16; i=i+1) begin 
                valid1[i] <= 1'b0;
                valid2[i] <= 1'b0; 
                plru[i]   <= 1'b0; 
            end
        end else if (flush) begin
            state <= IDLE;
            req_sent <= 1'b0;
        end else begin
            state <= next_state;

            // BỔ SUNG: Quản lý cờ Handshake
            if (state == IDLE) begin
                req_sent <= 1'b0;
            end else if (state == MEM_READ && mem_read_ready && mem_read_req) begin
                req_sent <= 1'b1;
            end

            if (cpu_read_req && state == IDLE) begin
                if (valid1[index] && tag1_out == tag) plru[index] <= 1'b1;
                else if (valid2[index] && tag2_out == tag) plru[index] <= 1'b0;
            end 
            else if (way_update != 2'b00) begin
                if (way_update[0]) begin 
                    valid1[index] <= 1'b1;
                    plru[index]   <= 1'b1; 
                end else if (way_update[1]) begin 
                    valid2[index] <= 1'b1;
                    plru[index]   <= 1'b0; 
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. Logic tổ hợp (FSM & Output)
    // -------------------------------------------------------------------------
    always @(*) begin
        next_state    = state;
        icache_hit    = 1'b0;
        icache_stall  = 1'b0; 
        mem_read_req  = 1'b0; 
        cpu_read_data = 32'b0; 
        mem_addr      = 32'b0;
        way_update    = 2'b00;

        if (flush) begin
            icache_stall = 1'b1;
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
                            icache_stall  = 1'b1;
                            mem_read_req  = 1'b1;
                            mem_addr      = {tag, index, 3'b000};
                            next_state    = MEM_READ;
                        end
                    end
                end

                MEM_READ: begin
                    icache_stall = 1'b1;
                    mem_read_req = !req_sent; // BỔ SUNG: Chỉ giữ req khi bus chưa ready
                    mem_addr     = {tag, index, 3'b000};
                    
                    if (mem_read_valid) begin
                        icache_stall = 1'b0;
                        mem_read_req = 1'b0; 
                        next_state   = IDLE;
                        
                        cpu_read_data = (word_offset == 1'b0) ? mem_read_data[31:0] : mem_read_data[63:32];
                        
                        if (!valid1[index])      way_update[0] = 1'b1;
                        else if (!valid2[index]) way_update[1] = 1'b1;
                        else if (plru[index] == 1'b0) way_update[0] = 1'b1;
                        else                     way_update[1] = 1'b1;
                    end
                end
            endcase
        end
    end
endmodule