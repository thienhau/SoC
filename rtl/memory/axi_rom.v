`timescale 1ns / 1ps

module axi_rom_ip #(
    parameter ADDR_WIDTH        = 32,
    parameter DATA_WIDTH        = 32,
    parameter ID_WIDTH          = 5,
    parameter ADDR_MASK         = 32'h0000_3FFF, // Mask 16KB
    parameter MEM_DEPTH         = 4096,          // 16KB / 4 = 4096 Words
    parameter INIT_FILE         = "boot.hex"     // File chứa mã máy
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Kênh Ghi (Sẽ chặn và báo lỗi SLVERR)
    input  wire [ID_WIDTH-1:0]      s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire [7:0]               s_axi_awlen,
    input  wire [2:0]               s_axi_awsize,
    input  wire [1:0]               s_axi_awburst,
    input  wire                     s_axi_awlock,
    input  wire [3:0]               s_axi_awcache,
    input  wire [2:0]               s_axi_awprot,
    input  wire [3:0]               s_axi_awqos,
    input  wire [3:0]               s_axi_awregion,
    input  wire                     s_axi_awvalid,
    output reg                      s_axi_awready,

    input  wire [DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]s_axi_wstrb,
    input  wire                     s_axi_wlast,
    input  wire                     s_axi_wvalid,
    output reg                      s_axi_wready,

    output reg  [ID_WIDTH-1:0]      s_axi_bid,
    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    // Kênh Đọc (Hoạt động chính)
    input  wire [ID_WIDTH-1:0]      s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire [7:0]               s_axi_arlen,
    input  wire [2:0]               s_axi_arsize,
    input  wire [1:0]               s_axi_arburst,
    input  wire                     s_axi_arlock,
    input  wire [3:0]               s_axi_arcache,
    input  wire [2:0]               s_axi_arprot,
    input  wire [3:0]               s_axi_arqos,
    input  wire [3:0]               s_axi_arregion,
    input  wire                     s_axi_arvalid,
    output reg                      s_axi_arready,

    output reg  [ID_WIDTH-1:0]      s_axi_rid,
    output reg  [DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rlast,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready
);

    // ==========================================
    // ROM VÀ KHỞI TẠO
    // ==========================================
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] rom_memory [0:MEM_DEPTH-1];

    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, rom_memory);
        end
    end

    // ==========================================
    // WRITE FSM (DUMMY SINK & RETURN ERROR)
    // ==========================================
    localparam W_IDLE = 2'd0, W_SINK = 2'd1, W_RESP = 2'd2;

    reg [1:0] w_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state <= W_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_bid <= 0;
            s_axi_bresp <= 2'b00;
        end else begin
            case (w_state)
                W_IDLE: begin
                    s_axi_awready <= 1'b1;
                    if (s_axi_awvalid && s_axi_awready) begin
                        s_axi_bid <= s_axi_awid;
                        s_axi_awready <= 1'b0;
                        s_axi_wready <= 1'b1;
                        w_state <= W_SINK;
                    end
                end
                W_SINK: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        if (s_axi_wlast) begin
                            s_axi_wready <= 1'b0;
                            s_axi_bvalid <= 1'b1;
                            s_axi_bresp <= 2'b10; // SLVERR
                            w_state <= W_RESP;
                        end
                    end
                end
                W_RESP: begin
                    if (s_axi_bready && s_axi_bvalid) begin
                        s_axi_bvalid <= 1'b0;
                        w_state <= W_IDLE;
                    end
                end
            endcase
        end
    end

    // ==========================================
    // READ FSM
    // ==========================================
    localparam R_IDLE = 2'd0, R_FETCH = 2'd1, R_DATA = 2'd2;

    reg [1:0] r_state;
    reg [ADDR_WIDTH-1:0] ar_addr_reg;
    reg [2:0] ar_size_reg;
    reg [7:0] ar_len_reg;
    reg [7:0] r_count;
    
    // Memory Map Decoding: Tính offset dựa trên Mask
    wire [$clog2(MEM_DEPTH)-1:0] raddr_idx = (ar_addr_reg & ADDR_MASK) >> 2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state <= R_IDLE;
            s_axi_arready <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rlast <= 1'b0;
            s_axi_rid <= 0;
            s_axi_rresp <= 2'b00;
        end else begin
            case (r_state)
                R_IDLE: begin
                    s_axi_arready <= 1'b1;
                    s_axi_rvalid <= 1'b0;
                    if (s_axi_arvalid && s_axi_arready) begin
                        ar_addr_reg <= s_axi_araddr;
                        ar_size_reg <= s_axi_arsize;
                        ar_len_reg  <= s_axi_arlen;
                        s_axi_rid   <= s_axi_arid;
                        r_count     <= 8'd0;
                        
                        s_axi_arready <= 1'b0;
                        r_state <= R_FETCH;
                    end
                end
                R_FETCH: begin
                    s_axi_rdata <= rom_memory[raddr_idx];
                    s_axi_rvalid <= 1'b1;
                    s_axi_rresp <= 2'b00; // OKAY
                    s_axi_rlast <= (r_count == ar_len_reg);
                    r_state <= R_DATA;
                end
                R_DATA: begin
                    if (s_axi_rvalid && s_axi_rready) begin
                        if (s_axi_rlast) begin
                            s_axi_rvalid <= 1'b0;
                            s_axi_rlast <= 1'b0;
                            r_state <= R_IDLE;
                        end else begin
                            r_count <= r_count + 1;
                            ar_addr_reg <= ar_addr_reg + (1 << ar_size_reg);
                            s_axi_rvalid <= 1'b0;
                            r_state <= R_FETCH;
                        end
                    end
                end
            endcase
        end
    end

endmodule