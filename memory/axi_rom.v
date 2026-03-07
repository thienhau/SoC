`timescale 1ns / 1ps

module axi4l_rom_slave #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter BASE_ADDR  = 32'h0000_1000,
    parameter MEM_DEPTH  = 4096 // 4096 words = 16 Kilobytes
)(
    input wire clk,
    input wire rst_n,

    // Kênh Ghi (Sẽ báo lỗi nếu truy cập)
    input  wire [ADDR_WIDTH-1:0]  s_axi_awaddr,
    input  wire                   s_axi_awvalid,
    output reg                    s_axi_awready,
    input  wire [DATA_WIDTH-1:0]  s_axi_wdata,
    input  wire [3:0]             s_axi_wstrb,
    input  wire                   s_axi_wvalid,
    output reg                    s_axi_wready,
    output reg  [1:0]             s_axi_bresp,
    output reg                    s_axi_bvalid,
    input  wire                   s_axi_bready,

    // Kênh Đọc
    input  wire [ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire                   s_axi_arvalid,
    output reg                    s_axi_arready,
    output reg  [DATA_WIDTH-1:0]  s_axi_rdata,
    output reg  [1:0]             s_axi_rresp,
    output reg                    s_axi_rvalid,
    input  wire                   s_axi_rready
);

    // Mảng bộ nhớ ROM
    reg [DATA_WIDTH-1:0] rom_memory [0:MEM_DEPTH-1];

    // Nạp mã nhị phân khi tổng hợp mạch (Khởi tạo ROM)
    initial begin
        $readmemh("bootrom.hex", rom_memory);
    end

    // =========================================================================
    // LOGIC KÊNH GHI (CHẶN HOÀN TOÀN)
    // =========================================================================
    // Phản hồi lỗi (SLVERR = 2'b10) khi có bất kỳ yêu cầu ghi nào
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
        end else begin
            // Bắt tay kênh Ghi
            if (~s_axi_awready && s_axi_awvalid && s_axi_wvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
            end

            // Gửi tín hiệu hoàn tất Ghi kèm mã Lỗi
            if (s_axi_awready && s_axi_awvalid && s_axi_wready && s_axi_wvalid && ~s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b10; // 2'b10 = SLVERR (Lỗi truy cập phần cứng)
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // LOGIC KÊNH ĐỌC
    // =========================================================================
    reg [ADDR_WIDTH-1:0] read_addr_latch;
    wire [11:0] read_index = (read_addr_latch - BASE_ADDR) >> 2;

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
                s_axi_rdata  <= rom_memory[read_index];
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule