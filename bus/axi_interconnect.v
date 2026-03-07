`timescale 1ns / 1ps

module axi_interconnect #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32
)(
    // Masters
    // M0: I-Cache (Chỉ Đọc)
    input  wire [ADDR_WIDTH-1:0]  m0_araddr,
    input  wire                   m0_arvalid,
    output reg                    m0_arready,
    output reg  [DATA_WIDTH-1:0]  m0_rdata,
    output reg  [1:0]             m0_rresp,
    output reg                    m0_rvalid,
    input  wire                   m0_rready,

    // M1: D-Cache (Đọc & Ghi)
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

    // Slaves
    // S0: ROM (0x1000 - 0x1FFF)
    output wire [ADDR_WIDTH-1:0]  s0_araddr,
    output wire                   s0_arvalid,
    input  wire                   s0_arready,
    input  wire [DATA_WIDTH-1:0]  s0_rdata,
    input  wire [1:0]             s0_rresp,
    input  wire                   s0_rvalid,
    output wire                   s0_rready,

    // S1: RAM (0x8000 - 0x8FFF)
    output wire [ADDR_WIDTH-1:0]  s1_awaddr,
    output wire                   s1_awvalid,
    input  wire                   s1_awready,
    output wire [DATA_WIDTH-1:0]  s1_wdata,
    output wire [3:0]             s1_wstrb,
    output wire                   s1_wvalid,
    input  wire                   s1_wready,
    input  wire [1:0]             s1_bresp,
    input  wire                   s1_bvalid,
    output wire                   s1_bready,
    output wire [ADDR_WIDTH-1:0]  s1_araddr,
    output wire                   s1_arvalid,
    input  wire                   s1_arready,
    input  wire [DATA_WIDTH-1:0]  s1_rdata,
    input  wire [1:0]             s1_rresp,
    input  wire                   s1_rvalid,
    output wire                   s1_rready,

    // S2: APB Bridge (0x4000 - 0x4FFF)
    output wire [ADDR_WIDTH-1:0]  s2_awaddr,
    output wire                   s2_awvalid,
    input  wire                   s2_awready,
    output wire [DATA_WIDTH-1:0]  s2_wdata,
    output wire [3:0]             s2_wstrb,
    output wire                   s2_wvalid,
    input  wire                   s2_wready,
    input  wire [1:0]             s2_bresp,
    input  wire                   s2_bvalid,
    output wire                   s2_bready,
    output wire [ADDR_WIDTH-1:0]  s2_araddr,
    output wire                   s2_arvalid,
    input  wire                   s2_arready,
    input  wire [DATA_WIDTH-1:0]  s2_rdata,
    input  wire [1:0]             s2_rresp,
    input  wire                   s2_rvalid,
    output wire                   s2_rready
);

    // ==========================================
    // GIẢI MÃ ĐỊA CHỈ (ADDRESS DECODING)
    // ==========================================
    // Dùng 4 bit cao nhất để định tuyến
    wire dec_w_ram = (m1_awaddr[15:12] == 4'h8);
    wire dec_w_apb = (m1_awaddr[15:12] == 4'h4);
    wire dec_w_err = !(dec_w_ram || dec_w_apb);

    // Trọng tài kênh Đọc: Ưu tiên M0 (I-Cache)
    wire r_grant_m0 = m0_arvalid;
    wire r_grant_m1 = m1_arvalid && !m0_arvalid;
    wire [15:0] current_araddr = r_grant_m0 ? m0_araddr : m1_araddr;
    
    wire dec_r_rom = (current_araddr[15:12] == 4'h1);
    wire dec_r_ram = (current_araddr[15:12] == 4'h8);
    wire dec_r_apb = (current_araddr[15:12] == 4'h4);
    wire dec_r_err = !(dec_r_rom || dec_r_ram || dec_r_apb);

    // ==========================================
    // ĐỊNH TUYẾN KÊNH GHI (WRITE ROUTING)
    // ==========================================
    assign s1_awaddr  = m1_awaddr;
    assign s1_wdata   = m1_wdata;
    assign s1_wstrb   = m1_wstrb;
    assign s1_awvalid = m1_awvalid && dec_w_ram;
    assign s1_wvalid  = m1_wvalid  && dec_w_ram;
    assign s1_bready  = m1_bready  && dec_w_ram;

    assign s2_awaddr  = m1_awaddr;
    assign s2_wdata   = m1_wdata;
    assign s2_wstrb   = m1_wstrb;
    assign s2_awvalid = m1_awvalid && dec_w_apb;
    assign s2_wvalid  = m1_wvalid  && dec_w_apb;
    assign s2_bready  = m1_bready  && dec_w_apb;

    always @(*) begin
        if (dec_w_ram) begin
            m1_awready = s1_awready;
            m1_wready  = s1_wready;
            m1_bresp   = s1_bresp;
            m1_bvalid  = s1_bvalid;
        end else if (dec_w_apb) begin
            m1_awready = s2_awready;
            m1_wready  = s2_wready;
            m1_bresp   = s2_bresp;
            m1_bvalid  = s2_bvalid;
        end else begin
            // Xử lý địa chỉ rác (DECERR)
            m1_awready = 1'b1;
            m1_wready  = 1'b1;
            m1_bresp   = 2'b11; // DECERR
            m1_bvalid  = m1_awvalid && m1_wvalid;
        end
    end

    // ==========================================
    // ĐỊNH TUYẾN KÊNH ĐỌC (READ ROUTING)
    // ==========================================
    assign s0_araddr  = current_araddr;
    assign s0_arvalid = (r_grant_m0 || r_grant_m1) && dec_r_rom;
    assign s0_rready  = r_grant_m0 ? m0_rready : m1_rready;

    assign s1_araddr  = current_araddr;
    assign s1_arvalid = (r_grant_m0 || r_grant_m1) && dec_r_ram;
    assign s1_rready  = r_grant_m0 ? m0_rready : m1_rready;

    assign s2_araddr  = current_araddr;
    assign s2_arvalid = (r_grant_m0 || r_grant_m1) && dec_r_apb;
    assign s2_rready  = r_grant_m0 ? m0_rready : m1_rready;

    reg        mux_arready;
    reg [31:0] mux_rdata;
    reg [1:0]  mux_rresp;
    reg        mux_rvalid;

    always @(*) begin
        if (dec_r_rom) begin
            mux_arready = s0_arready; mux_rdata = s0_rdata; mux_rresp = s0_rresp; mux_rvalid = s0_rvalid;
        end else if (dec_r_ram) begin
            mux_arready = s1_arready; mux_rdata = s1_rdata; mux_rresp = s1_rresp; mux_rvalid = s1_rvalid;
        end else if (dec_r_apb) begin
            mux_arready = s2_arready; mux_rdata = s2_rdata; mux_rresp = s2_rresp; mux_rvalid = s2_rvalid;
        end else begin
            // Địa chỉ rác
            mux_arready = 1'b1; mux_rdata = 32'h0; mux_rresp = 2'b11; mux_rvalid = (m0_arvalid || m1_arvalid);
        end
    end

    always @(*) begin
        // Mặc định kéo thấp
        m0_arready = 1'b0; m0_rdata = 32'h0; m0_rresp = 2'b00; m0_rvalid = 1'b0;
        m1_arready = 1'b0; m1_rdata = 32'h0; m1_rresp = 2'b00; m1_rvalid = 1'b0;
        
        if (r_grant_m0) begin
            m0_arready = mux_arready; m0_rdata = mux_rdata; m0_rresp = mux_rresp; m0_rvalid = mux_rvalid;
        end else if (r_grant_m1) begin
            m1_arready = mux_arready; m1_rdata = mux_rdata; m1_rresp = mux_rresp; m1_rvalid = mux_rvalid;
        end
    end

endmodule