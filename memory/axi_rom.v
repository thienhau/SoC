module axi4l_rom_slave #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32,
    parameter BASE_ADDR  = 16'h1000, // Địa chỉ bắt đầu của BootROM
    parameter MEM_DEPTH  = 1024      // 1024 words = 4KB
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // --- Kênh Ghi (Sẽ trả về lỗi vì đây là ROM) ---
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

    // --- Kênh Đọc ---
    input  wire [ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire                   s_axi_arvalid,
    output reg                    s_axi_arready,
    output reg  [DATA_WIDTH-1:0]  s_axi_rdata,
    output reg  [1:0]             s_axi_rresp,
    output reg                    s_axi_rvalid,
    input  wire                   s_axi_rready
);

    // Tính toán số bit cần thiết để định tuyến index trong mảng nhớ
    localparam ADDR_LSB = $clog2(DATA_WIDTH/8); // = 2 (Do 32-bit = 4 bytes)
    localparam INDEX_WIDTH = $clog2(MEM_DEPTH);

    // Khởi tạo bộ nhớ nội bộ (Ép công cụ tổng hợp thành Block RAM)
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] rom_array [0:MEM_DEPTH-1];

    // Nạp firmware vào ROM lúc biên dịch
    initial begin
        $readmemh("bootrom.hex", rom_array);
    end

    // =========================================================================
    // XỬ LÝ KÊNH ĐỌC (READ CHANNELS) - Có độ trễ 1 chu kỳ để tương thích BRAM
    // =========================================================================
    localparam R_ST_IDLE      = 2'b00;
    localparam R_ST_MEM_DELAY = 2'b01; // Đợi 1 cycle để BRAM xuất dữ liệu
    localparam R_ST_RESP      = 2'b10;

    reg [1:0] r_state;
    reg [INDEX_WIDTH-1:0] read_index;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state       <= R_ST_IDLE;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= 32'b0;
            read_index    <= 0;
        end else begin
            case (r_state)
                R_ST_IDLE: begin
                    s_axi_rvalid <= 1'b0;
                    // Sẵn sàng nhận địa chỉ
                    s_axi_arready <= 1'b1; 
                    
                    if (s_axi_arvalid && s_axi_arready) begin
                        s_axi_arready <= 1'b0;
                        // Tính toán chỉ số mảng (Trừ đi Base Address và dịch bit)
                        read_index <= (s_axi_araddr - BASE_ADDR) >> ADDR_LSB;
                        r_state    <= R_ST_MEM_DELAY;
                    end
                end

                R_ST_MEM_DELAY: begin
                    // Đọc dữ liệu từ Block RAM mất 1 chu kỳ clock
                    s_axi_rdata <= rom_array[read_index];
                    s_axi_rresp <= 2'b00; // OKAY
                    s_axi_rvalid <= 1'b1;
                    r_state     <= R_ST_RESP;
                end

                R_ST_RESP: begin
                    // Giữ rvalid ở mức cao cho đến khi Master kéo rready lên
                    if (s_axi_rready && s_axi_rvalid) begin
                        s_axi_rvalid <= 1'b0;
                        r_state      <= R_ST_IDLE;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // XỬ LÝ KÊNH GHI (WRITE CHANNELS) - Trả về lỗi SLVERR
    // =========================================================================
    localparam W_ST_IDLE = 1'b0;
    localparam W_ST_RESP = 1'b1;
    reg w_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state       <= W_ST_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
        end else begin
            case (w_state)
                W_ST_IDLE: begin
                    s_axi_awready <= 1'b1;
                    s_axi_wready  <= 1'b1;
                    s_axi_bvalid  <= 1'b0;

                    // Chờ Master gửi yêu cầu Ghi
                    if (s_axi_awvalid && s_axi_wvalid) begin
                        s_axi_awready <= 1'b0;
                        s_axi_wready  <= 1'b0;
                        
                        s_axi_bresp  <= 2'b10; // SLVERR (Lỗi do cố tình ghi vào ROM)
                        s_axi_bvalid <= 1'b1;
                        w_state      <= W_ST_RESP;
                    end
                end

                W_ST_RESP: begin
                    if (s_axi_bready && s_axi_bvalid) begin
                        s_axi_bvalid <= 1'b0;
                        w_state      <= W_ST_IDLE;
                    end
                end
            endcase
        end
    end

endmodule