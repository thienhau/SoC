module axi4l_ram_slave #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32,
    parameter BASE_ADDR  = 16'h8000, // Địa chỉ bắt đầu của RAM
    parameter MEM_DEPTH  = 2048      // 2048 words = 8KB
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // --- Kênh Ghi ---
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

    localparam ADDR_LSB = $clog2(DATA_WIDTH/8);
    localparam INDEX_WIDTH = $clog2(MEM_DEPTH);

    // Khởi tạo bộ nhớ (True Dual Port BRAM inferred)
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram_array [0:MEM_DEPTH-1];

    // Có thể load file hex khởi tạo RAM nếu cần
    // initial $readmemh("ram_init.hex", ram_array);

    integer byte_idx;

    // =========================================================================
    // FSM XỬ LÝ KÊNH GHI (WRITE PROCESS)
    // =========================================================================
    localparam W_ST_IDLE  = 2'b00;
    localparam W_ST_WRITE = 2'b01;
    localparam W_ST_RESP  = 2'b10;

    reg [1:0] w_state;
    reg [ADDR_WIDTH-1:0] write_addr_latch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state       <= W_ST_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            write_addr_latch <= 0;
        end else begin
            case (w_state)
                W_ST_IDLE: begin
                    s_axi_bvalid  <= 1'b0;
                    // Bật Ready để đón nhận địa chỉ và dữ liệu
                    s_axi_awready <= 1'b1;
                    s_axi_wready  <= 1'b1;

                    if (s_axi_awvalid && s_axi_wvalid) begin
                        write_addr_latch <= s_axi_awaddr;
                        s_axi_awready    <= 1'b0;
                        s_axi_wready     <= 1'b0;
                        w_state          <= W_ST_WRITE;
                    end
                end

                W_ST_WRITE: begin
                    // Tính toán index thực tế
                    begin : RAM_WRITE_BLOCK
                        reg [INDEX_WIDTH-1:0] w_index;
                        w_index = (write_addr_latch - BASE_ADDR) >> ADDR_LSB;
                        
                        // Kiểm tra biên (Out-of-bounds)
                        if (w_index < MEM_DEPTH) begin
                            // Ghi từng byte phụ thuộc vào WSTRB
                            for (byte_idx = 0; byte_idx < (DATA_WIDTH/8); byte_idx = byte_idx + 1) begin
                                if (s_axi_wstrb[byte_idx]) begin
                                    ram_array[w_index][(byte_idx*8) +: 8] <= s_axi_wdata[(byte_idx*8) +: 8];
                                end
                            end
                            s_axi_bresp <= 2'b00; // OKAY
                        end else begin
                            s_axi_bresp <= 2'b10; // SLVERR (Ghi ra ngoài giới hạn RAM)
                        end
                    end
                    
                    s_axi_bvalid <= 1'b1;
                    w_state      <= W_ST_RESP;
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

    // =========================================================================
    // FSM XỬ LÝ KÊNH ĐỌC (READ PROCESS)
    // =========================================================================
    localparam R_ST_IDLE      = 2'b00;
    localparam R_ST_MEM_DELAY = 2'b01;
    localparam R_ST_RESP      = 2'b10;

    reg [1:0] r_state;
    reg [ADDR_WIDTH-1:0] read_addr_latch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state       <= R_ST_IDLE;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= 32'b0;
            read_addr_latch <= 0;
        end else begin
            case (r_state)
                R_ST_IDLE: begin
                    s_axi_rvalid  <= 1'b0;
                    s_axi_arready <= 1'b1; 
                    
                    if (s_axi_arvalid && s_axi_arready) begin
                        read_addr_latch <= s_axi_araddr;
                        s_axi_arready   <= 1'b0;
                        r_state         <= R_ST_MEM_DELAY;
                    end
                end

                R_ST_MEM_DELAY: begin
                    begin : RAM_READ_BLOCK
                        reg [INDEX_WIDTH-1:0] r_index;
                        r_index = (read_addr_latch - BASE_ADDR) >> ADDR_LSB;
                        
                        if (r_index < MEM_DEPTH) begin
                            s_axi_rdata <= ram_array[r_index];
                            s_axi_rresp <= 2'b00; // OKAY
                        end else begin
                            s_axi_rdata <= 32'hDEADBEEF; // Dữ liệu rác
                            s_axi_rresp <= 2'b10;        // SLVERR
                        end
                    end
                    
                    s_axi_rvalid <= 1'b1;
                    r_state      <= R_ST_RESP;
                end

                R_ST_RESP: begin
                    if (s_axi_rready && s_axi_rvalid) begin
                        s_axi_rvalid <= 1'b0;
                        r_state      <= R_ST_IDLE;
                    end
                end
            endcase
        end
    end

endmodule