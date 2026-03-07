`timescale 1ns / 1ps

module axi4l_ram_slave #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter BASE_ADDR  = 32'h8000_0000,
    parameter MEM_DEPTH  = 16384 // 16384 words = 64 Kilobytes
)(
    input wire clk,
    input wire rst_n,

    // Kênh Ghi Địa Chỉ (Write Address)
    input  wire [ADDR_WIDTH-1:0]  s_axi_awaddr,
    input  wire                   s_axi_awvalid,
    output reg                    s_axi_awready,

    // Kênh Ghi Dữ Liệu (Write Data)
    input  wire [DATA_WIDTH-1:0]  s_axi_wdata,
    input  wire [3:0]             s_axi_wstrb,
    input  wire                   s_axi_wvalid,
    output reg                    s_axi_wready,

    // Kênh Phản Hồi Ghi (Write Response)
    output reg  [1:0]             s_axi_bresp,
    output reg                    s_axi_bvalid,
    input  wire                   s_axi_bready,

    // Kênh Đọc Địa Chỉ (Read Address)
    input  wire [ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire                   s_axi_arvalid,
    output reg                    s_axi_arready,

    // Kênh Đọc Dữ Liệu (Read Data)
    output reg  [DATA_WIDTH-1:0]  s_axi_rdata,
    output reg  [1:0]             s_axi_rresp,
    output reg                    s_axi_rvalid,
    input  wire                   s_axi_rready
);

    // Mảng bộ nhớ RAM nội bộ
    reg [DATA_WIDTH-1:0] ram_memory [0:MEM_DEPTH-1];

    // Các biến cờ (Flags) điều khiển FSM AXI Handshake
    reg aw_en;
    reg [ADDR_WIDTH-1:0] write_addr_latch;
    reg [ADDR_WIDTH-1:0] read_addr_latch;

    // Tính toán địa chỉ mảng (Bỏ đi BASE_ADDR và chia 4 để trỏ tới Word)
    wire [13:0] write_index = (write_addr_latch - BASE_ADDR) >> 2;
    wire [13:0] read_index  = (read_addr_latch  - BASE_ADDR) >> 2;

    // =========================================================================
    // LOGIC KÊNH GHI (WRITE CHANNEL)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
            aw_en         <= 1'b1;
        end else begin
            if (~s_axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en) begin
                s_axi_awready <= 1'b1;
                aw_en         <= 1'b0;
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_awready <= 1'b0;
                aw_en         <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_addr_latch <= 32'h0;
        end else begin
            if (~s_axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en) begin
                write_addr_latch <= s_axi_awaddr;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_wready <= 1'b0;
        end else begin
            if (~s_axi_wready && s_axi_wvalid && s_axi_awvalid && aw_en) begin
                s_axi_wready <= 1'b1;
            end else begin
                s_axi_wready <= 1'b0;
            end
        end
    end

    // Ghi dữ liệu vào mảng RAM (Hỗ trợ Byte Strobe - PSTRB)
    always @(posedge clk) begin
        if (s_axi_wready && s_axi_wvalid && s_axi_awready && s_axi_awvalid) begin
            if (s_axi_wstrb[0]) ram_memory[write_index][7:0]   <= s_axi_wdata[7:0];
            if (s_axi_wstrb[1]) ram_memory[write_index][15:8]  <= s_axi_wdata[15:8];
            if (s_axi_wstrb[2]) ram_memory[write_index][23:16] <= s_axi_wdata[23:16];
            if (s_axi_wstrb[3]) ram_memory[write_index][31:24] <= s_axi_wdata[31:24];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
        end else begin
            if (s_axi_awready && s_axi_awvalid && s_axi_wready && s_axi_wvalid && ~s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00; // Trả về OKAY
            end else begin
                if (s_axi_bready && s_axi_bvalid) begin
                    s_axi_bvalid <= 1'b0;
                end
            end
        end
    end

    // =========================================================================
    // LOGIC KÊNH ĐỌC (READ CHANNEL)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready   <= 1'b0;
            read_addr_latch <= 32'h0;
        end else begin
            if (~s_axi_arready && s_axi_arvalid) begin
                s_axi_arready   <= 1'b1;
                read_addr_latch <= s_axi_araddr;
            end else begin
                s_axi_arready <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rresp  <= 2'b00;
            s_axi_rdata  <= 32'h0;
        end else begin
            if (s_axi_arready && s_axi_arvalid && ~s_axi_rvalid) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00; // Trả về OKAY
                s_axi_rdata  <= ram_memory[read_index];
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule