`timescale 1ns / 1ps

/**
 * MODULE: axi_interconnect
 * DESCRIPTION: Bộ kết nối trung tâm AXI4-Lite (Multi-Master, Multi-Slave).
 * Quản lý định tuyến và trọng tài giữa 3 Master và 4 Slave.
 * * MAP ĐỊA CHỈ (ADDRESS MAP):
 * - S0: BOOT ROM    (0x0000_1000 - 0x0000_4FFF) -> Chứa Bootloader mồi
 * - S1: SYSTEM RAM  (0x8000_0000 - 0x8000_FFFF) -> Thực thi chương trình chính
 * - S2: APB BRIDGE  (0x4000_0000 - 0x4FFF_FFFF) -> Điều khiển ngoại vi
 * - S3: SPI FLASH   (0x2000_0000 - 0x2FFF_FFFF) -> Nơi chứa Firmware chính
 */

module axi_interconnect #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input wire clk,
    input wire rst_n,

    // =========================================================================
    // KÊNH MASTER 0 (M0): I-CACHE (Instruction Fetch - Chỉ Đọc)
    // =========================================================================
    input  wire [ADDR_WIDTH-1:0]  m0_araddr,
    input  wire                   m0_arvalid,
    output reg                    m0_arready,
    output reg  [DATA_WIDTH-1:0]  m0_rdata,
    output reg  [1:0]             m0_rresp,
    output reg                    m0_rvalid,
    input  wire                   m0_rready,

    // =========================================================================
    // KÊNH MASTER 1 (M1): D-CACHE (Data Access - Đọc/Ghi)
    // =========================================================================
    input  wire [ADDR_WIDTH-1:0]  m1_awaddr,
    input  wire                   m1_awvalid,
    output reg                    m1_awready,
    input  wire [DATA_WIDTH-1:0]  m1_wdata,
    input  wire [3:0]             m1_wstrb,
    input  wire                   m1_wvalid,
    output reg                    m1_wready,
    output reg  [1:0]             m1_bresp,
    output reg                    m1_bvalid,
    input  wire                   m1_bready,
    input  wire [ADDR_WIDTH-1:0]  m1_araddr,
    input  wire                   m1_arvalid,
    output reg                    m1_arready,
    output reg  [DATA_WIDTH-1:0]  m1_rdata,
    output reg  [1:0]             m1_rresp,
    output reg                    m1_rvalid,
    input  wire                   m1_rready,

    // =========================================================================
    // KÊNH MASTER 2 (M2): DEBUG MODULE (JTAG Debug - Đọc/Ghi)
    // =========================================================================
    input  wire [ADDR_WIDTH-1:0]  m2_awaddr,
    input  wire                   m2_awvalid,
    output reg                    m2_awready,
    input  wire [DATA_WIDTH-1:0]  m2_wdata,
    input  wire [3:0]             m2_wstrb,
    input  wire                   m2_wvalid,
    output reg                    m2_wready,
    output reg  [1:0]             m2_bresp,
    output reg                    m2_bvalid,
    input  wire                   m2_bready,
    input  wire [ADDR_WIDTH-1:0]  m2_araddr,
    input  wire                   m2_arvalid,
    output reg                    m2_arready,
    output reg  [DATA_WIDTH-1:0]  m2_rdata,
    output reg  [1:0]             m2_rresp,
    output reg                    m2_rvalid,
    input  wire                   m2_rready,

    // =========================================================================
    // KÊNH SLAVE 0 (S0): BOOT ROM
    // =========================================================================
    output reg  [ADDR_WIDTH-1:0]  s0_araddr,
    output reg                    s0_arvalid,
    input  wire                   s0_arready,
    input  wire [DATA_WIDTH-1:0]  s0_rdata,
    input  wire [1:0]             s0_rresp,
    input  wire                   s0_rvalid,
    output reg                    s0_rready,

    // =========================================================================
    // KÊNH SLAVE 1 (S1): SYSTEM RAM
    // =========================================================================
    output reg  [ADDR_WIDTH-1:0]  s1_awaddr,
    output reg                    s1_awvalid,
    input  wire                   s1_awready,
    output reg  [DATA_WIDTH-1:0]  s1_wdata,
    output reg  [3:0]             s1_wstrb,
    output reg                    s1_wvalid,
    input  wire                   s1_wready,
    input  wire [1:0]             s1_bresp,
    input  wire                   s1_bvalid,
    output reg                    s1_bready,
    output reg  [ADDR_WIDTH-1:0]  s1_araddr,
    output reg                    s1_arvalid,
    input  wire                   s1_arready,
    input  wire [DATA_WIDTH-1:0]  s1_rdata,
    input  wire [1:0]             s1_rresp,
    input  wire                   s1_rvalid,
    output reg                    s1_rready,

    // =========================================================================
    // KÊNH SLAVE 2 (S2): APB BRIDGE (Ngoại vi)
    // =========================================================================
    output reg  [ADDR_WIDTH-1:0]  s2_awaddr,
    output reg                    s2_awvalid,
    input  wire                   s2_awready,
    output reg  [DATA_WIDTH-1:0]  s2_wdata,
    output reg  [3:0]             s2_wstrb,
    output reg                    s2_wvalid,
    input  wire                   s2_wready,
    input  wire [1:0]             s2_bresp,
    input  wire                   s2_bvalid,
    output reg                    s2_bready,
    output reg  [ADDR_WIDTH-1:0]  s2_araddr,
    output reg                    s2_arvalid,
    input  wire                   s2_arready,
    input  wire [DATA_WIDTH-1:0]  s2_rdata,
    input  wire [1:0]             s2_rresp,
    input  wire                   s2_rvalid,
    output reg                    s2_rready,

    // =========================================================================
    // KÊNH SLAVE 3 (S3): AXI SPI FLASH (MỚI)
    // =========================================================================
    output reg  [ADDR_WIDTH-1:0]  s3_araddr,
    output reg                    s3_arvalid,
    input  wire                   s3_arready,
    input  wire [DATA_WIDTH-1:0]  s3_rdata,
    input  wire [1:0]             s3_rresp,
    input  wire                   s3_rvalid,
    output reg                    s3_rready
);

    // =========================================================================
    // TRỌNG TÀI KÊNH ĐỌC (READ ARBITRATION)
    // =========================================================================
    localparam R_IDLE      = 1'b0;
    localparam R_TRANSFER  = 1'b1;

    reg        r_state;
    reg [1:0]  r_grant; // 0:M0, 1:M1, 2:M2
    reg [1:0]  r_token; // Thẻ xoay vòng

    wire [2:0] r_req = {m2_arvalid, m1_arvalid, m0_arvalid};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state <= R_IDLE;
            r_grant <= 2'b00;
            r_token <= 2'b00;
        end else begin
            case (r_state)
                R_IDLE: begin
                    if (r_req != 3'b000) begin
                        r_state <= R_TRANSFER;
                        // Thuật toán Round-Robin
                        case (r_token)
                            2'd0: begin
                                if      (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; end
                                else if (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; end
                                else               begin r_grant <= 2'd2; r_token <= 2'd0; end
                            end
                            2'd1: begin
                                if      (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; end
                                else if (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd0; end
                                else               begin r_grant <= 2'd0; r_token <= 2'd1; end
                            end
                            2'd2: begin
                                if      (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd0; end
                                else if (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; end
                                else               begin r_grant <= 2'd1; r_token <= 2'd2; end
                            end
                            default: r_token <= 2'd0;
                        endcase
                    end
                end
                R_TRANSFER: begin
                    // Kết thúc khi hoàn thành chu kỳ RVALID & RREADY của Master đang giữ Grant
                    if ((r_grant == 2'd0 && m0_rvalid && m0_rready) ||
                        (r_grant == 2'd1 && m1_rvalid && m1_rready) ||
                        (r_grant == 2'd2 && m2_rvalid && m2_rready)) begin
                        r_state <= R_IDLE;
                    end
                end
            endcase
        end
    end

    // Giải mã địa chỉ đọc (Read Address Decoding)
    wire [ADDR_WIDTH-1:0] current_r_addr = (r_grant == 2'd0) ? m0_araddr : 
                                           (r_grant == 2'd1) ? m1_araddr : m2_araddr;

    wire r_sel_s0 = (current_r_addr >= 32'h0000_1000) && (current_r_addr <= 32'h0000_4FFF);
    wire r_sel_s3 = (current_r_addr >= 32'h2000_0000) && (current_r_addr <= 32'h2FFF_FFFF); // SPI FLASH
    wire r_sel_s2 = (current_r_addr >= 32'h4000_0000) && (current_r_addr <= 32'h4FFF_FFFF);
    wire r_sel_s1 = (current_r_addr >= 32'h8000_0000) && (current_r_addr <= 32'h8000_FFFF);

    // Ghép kênh dữ liệu đọc (Read Muxing)
    always @(*) begin
        // Mặc định các Master không nhận gì
        m0_arready = 1'b0; m0_rvalid = 1'b0; m0_rdata = 32'h0; m0_rresp = 2'b00;
        m1_arready = 1'b0; m1_rvalid = 1'b0; m1_rdata = 32'h0; m1_rresp = 2'b00;
        m2_arready = 1'b0; m2_rvalid = 1'b0; m2_rdata = 32'h0; m2_rresp = 2'b00;
        
        // Mặc định các Slave không bị gọi
        s0_arvalid = 1'b0; s0_araddr = 32'h0; s0_rready = 1'b0;
        s1_arvalid = 1'b0; s1_araddr = 32'h0; s1_rready = 1'b0;
        s2_arvalid = 1'b0; s2_araddr = 32'h0; s2_rready = 1'b0;
        s3_arvalid = 1'b0; s3_araddr = 32'h0; s3_rready = 1'b0;

        if (r_state == R_TRANSFER) begin
            // Xử lý lỗi địa chỉ (Decode Error)
            if (!(r_sel_s0 || r_sel_s1 || r_sel_s2 || r_sel_s3)) begin
                if (r_grant == 2'd0) begin m0_arready = 1'b1; m0_rvalid = 1'b1; m0_rresp = 2'b11; end
                if (r_grant == 2'd1) begin m1_arready = 1'b1; m1_rvalid = 1'b1; m1_rresp = 2'b11; end
                if (r_grant == 2'd2) begin m2_arready = 1'b1; m2_rvalid = 1'b1; m2_rresp = 2'b11; end
            end else begin
                // Gán tín hiệu ARVALID và ADDR tới Slave tương ứng
                if (r_sel_s0) begin s0_araddr = current_r_addr; s0_arvalid = 1'b1; end
                else if (r_sel_s1) begin s1_araddr = current_r_addr; s1_arvalid = 1'b1; end
                else if (r_sel_s2) begin s2_araddr = current_r_addr; s2_arvalid = 1'b1; end
                else if (r_sel_s3) begin s3_araddr = current_r_addr; s3_arvalid = 1'b1; end

                // Phản hồi từ Slave về Master đang giữ Grant
                case (r_grant)
                    2'd0: begin
                        m0_arready = r_sel_s0 ? s0_arready : r_sel_s1 ? s1_arready : r_sel_s2 ? s2_arready : s3_arready;
                        m0_rdata   = r_sel_s0 ? s0_rdata   : r_sel_s1 ? s1_rdata   : r_sel_s2 ? s2_rdata   : s3_rdata;
                        m0_rresp   = r_sel_s0 ? s0_rresp   : r_sel_s1 ? s1_rresp   : r_sel_s2 ? s2_rresp   : s3_rresp;
                        m0_rvalid  = r_sel_s0 ? s0_rvalid  : r_sel_s1 ? s1_rvalid  : r_sel_s2 ? s2_rvalid  : s3_rvalid;
                        s0_rready  = r_sel_s0 ? m0_rready  : 1'b0;
                        s1_rready  = r_sel_s1 ? m0_rready  : 1'b0;
                        s2_rready  = r_sel_s2 ? m0_rready  : 1'b0;
                        s3_rready  = r_sel_s3 ? m0_rready  : 1'b0;
                    end
                    2'd1: begin
                        m1_arready = r_sel_s0 ? s0_arready : r_sel_s1 ? s1_arready : r_sel_s2 ? s2_arready : s3_arready;
                        m1_rdata   = r_sel_s0 ? s0_rdata   : r_sel_s1 ? s1_rdata   : r_sel_s2 ? s2_rdata   : s3_rdata;
                        m1_rresp   = r_sel_s0 ? s0_rresp   : r_sel_s1 ? s1_rresp   : r_sel_s2 ? s2_rresp   : s3_rresp;
                        m1_rvalid  = r_sel_s0 ? s0_rvalid  : r_sel_s1 ? s1_rvalid  : r_sel_s2 ? s2_rvalid  : s3_rvalid;
                        s0_rready  = r_sel_s0 ? m1_rready  : 1'b0;
                        s1_rready  = r_sel_s1 ? m1_rready  : 1'b0;
                        s2_rready  = r_sel_s2 ? m1_rready  : 1'b0;
                        s3_rready  = r_sel_s3 ? m1_rready  : 1'b0;
                    end
                    2'd2: begin
                        m2_arready = r_sel_s0 ? s0_arready : r_sel_s1 ? s1_arready : r_sel_s2 ? s2_arready : s3_arready;
                        m2_rdata   = r_sel_s0 ? s0_rdata   : r_sel_s1 ? s1_rdata   : r_sel_s2 ? s2_rdata   : s3_rdata;
                        m2_rresp   = r_sel_s0 ? s0_rresp   : r_sel_s1 ? s1_rresp   : r_sel_s2 ? s2_rresp   : s3_rresp;
                        m2_rvalid  = r_sel_s0 ? s0_rvalid  : r_sel_s1 ? s1_rvalid  : r_sel_s2 ? s2_rvalid  : s3_rvalid;
                        s0_rready  = r_sel_s0 ? m2_rready  : 1'b0;
                        s1_rready  = r_sel_s1 ? m2_rready  : 1'b0;
                        s2_rready  = r_sel_s2 ? m2_rready  : 1'b0;
                        s3_rready  = r_sel_s3 ? m2_rready  : 1'b0;
                    end
                endcase
            end
        end
    end

    // =========================================================================
    // TRỌNG TÀI KÊNH GHI (WRITE ARBITRATION)
    // =========================================================================
    localparam W_IDLE   = 1'b0;
    localparam W_ACTIVE = 1'b1;

    reg        w_state;
    reg        w_grant; // 0:M1, 1:M2
    reg        w_token;

    wire [1:0] w_req = {m2_awvalid, m1_awvalid};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state <= W_IDLE;
            w_grant <= 1'b0;
            w_token <= 1'b0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    if (w_req != 2'b00) begin
                        w_state <= W_ACTIVE;
                        if (w_token == 1'b0) begin
                            if (w_req[0]) begin w_grant <= 1'b0; w_token <= 1'b1; end
                            else          begin w_grant <= 1'b1; w_token <= 1'b0; end
                        end else begin
                            if (w_req[1]) begin w_grant <= 1'b1; w_token <= 1'b0; end
                            else          begin w_grant <= 1'b0; w_token <= 1'b1; end
                        end
                    end
                end
                W_ACTIVE: begin
                    if ((w_grant == 1'b0 && m1_bvalid && m1_bready) ||
                        (w_grant == 1'b1 && m2_bvalid && m2_bready)) begin
                        w_state <= W_IDLE;
                    end
                end
            endcase
        end
    end

    // Giải mã địa chỉ ghi (Write Address Decoding)
    wire [ADDR_WIDTH-1:0] current_w_addr = (w_grant == 1'b0) ? m1_awaddr : m2_awaddr;

    wire w_sel_s1 = (current_w_addr >= 32'h8000_0000) && (current_w_addr <= 32'h8000_FFFF);
    wire w_sel_s2 = (current_w_addr >= 32'h4000_0000) && (current_w_addr <= 32'h4FFF_FFFF);

    // Ghép kênh dữ liệu ghi (Write Muxing)
    always @(*) begin
        m1_awready = 1'b0; m1_wready = 1'b0; m1_bvalid = 1'b0; m1_bresp = 2'b00;
        m2_awready = 1'b0; m2_wready = 1'b0; m2_bvalid = 1'b0; m2_bresp = 2'b00;

        s1_awvalid = 1'b0; s1_awaddr = 32'h0; s1_wvalid = 1'b0; s1_wdata = 32'h0; s1_wstrb = 4'h0; s1_bready = 1'b0;
        s2_awvalid = 1'b0; s2_awaddr = 32'h0; s2_wvalid = 1'b0; s2_wdata = 32'h0; s2_wstrb = 4'h0; s2_bready = 1'b0;

        if (w_state == W_ACTIVE) begin
            // Giải mã lỗi (Slave 0 và Slave 3 không cho phép ghi)
            if (!(w_sel_s1 || w_sel_s2)) begin
                if (w_grant == 1'b0) begin m1_awready = 1'b1; m1_wready = 1'b1; m1_bvalid = 1'b1; m1_bresp = 2'b11; end
                if (w_grant == 1'b1) begin m2_awready = 1'b1; m2_wready = 1'b1; m2_bvalid = 1'b1; m2_bresp = 2'b11; end
            end else begin
                // Routing dữ liệu từ Master tới Slave
                if (w_sel_s1) begin
                    s1_awaddr  = current_w_addr;
                    s1_awvalid = (w_grant == 1'b0) ? m1_awvalid : m2_awvalid;
                    s1_wdata   = (w_grant == 1'b0) ? m1_wdata   : m2_wdata;
                    s1_wstrb   = (w_grant == 1'b0) ? m1_wstrb   : m2_wstrb;
                    s1_wvalid  = (w_grant == 1'b0) ? m1_wvalid  : m2_wvalid;
                end else if (w_sel_s2) begin
                    s2_awaddr  = current_w_addr;
                    s2_awvalid = (w_grant == 1'b0) ? m1_awvalid : m2_awvalid;
                    s2_wdata   = (w_grant == 1'b0) ? m1_wdata   : m2_wdata;
                    s2_wstrb   = (w_grant == 1'b0) ? m1_wstrb   : m2_wstrb;
                    s2_wvalid  = (w_grant == 1'b0) ? m1_wvalid  : m2_wvalid;
                end

                // Routing phản hồi từ Slave về Master
                if (w_grant == 1'b0) begin
                    m1_awready = w_sel_s1 ? s1_awready : s2_awready;
                    m1_wready  = w_sel_s1 ? s1_wready  : s2_wready;
                    m1_bvalid  = w_sel_s1 ? s1_bvalid  : s2_bvalid;
                    m1_bresp   = w_sel_s1 ? s1_bresp   : s2_bresp;
                    s1_bready  = w_sel_s1 ? m1_bready  : 1'b0;
                    s2_bready  = w_sel_s2 ? m1_bready  : 1'b0;
                end else begin
                    m2_awready = w_sel_s1 ? s1_awready : s2_awready;
                    m2_wready  = w_sel_s1 ? s1_wready  : s2_wready;
                    m2_bvalid  = w_sel_s1 ? s1_bvalid  : s2_bvalid;
                    m2_bresp   = w_sel_s1 ? s1_bresp   : s2_bresp;
                    s1_bready  = w_sel_s1 ? m2_bready  : 1'b0;
                    s2_bready  = w_sel_s2 ? m2_bready  : 1'b0;
                end
            end
        end
    end

endmodule