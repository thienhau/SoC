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
    // 1. GIẢI MÃ ĐỊA CHỈ KÊNH GHI (WRITE DECODING)
    // ==========================================
    // Master 1 (D-Cache) và Master 2 (Debug) có thể ghi.
    // Giả sử dùng mạch trọng tài ưu tiên M2 > M1 cho kênh ghi (Debug có quyền cao nhất).
    
    wire w_grant_m2 = m2_awvalid;
    wire w_grant_m1 = m1_awvalid && !m2_awvalid;
    wire [15:0] current_awaddr = w_grant_m2 ? m2_awaddr : m1_awaddr;

    // Mapping: ROM(1000-4FFF) [Bỏ qua kênh Ghi], APB(5000-7FFF), RAM(8000-FFFF)
    wire dec_w_apb = (current_awaddr >= 16'h5000) && (current_awaddr <= 16'h7FFF);
    wire dec_w_ram = (current_awaddr >= 16'h8000); // 8000 đến FFFF
    wire dec_w_err = !(dec_w_apb || dec_w_ram);

    // Gán tín hiệu Write xuống S1 (RAM)
    assign s1_awaddr  = current_awaddr;
    assign s1_wdata   = w_grant_m2 ? m2_wdata : m1_wdata;
    assign s1_wstrb   = w_grant_m2 ? m2_wstrb : m1_wstrb;
    assign s1_awvalid = (w_grant_m1 || w_grant_m2) && dec_w_ram;
    assign s1_wvalid  = (w_grant_m1 || w_grant_m2) && dec_w_ram;
    assign s1_bready  = w_grant_m2 ? m2_bready : m1_bready;

    // Gán tín hiệu Write xuống S2 (APB Bridge)
    assign s2_awaddr  = current_awaddr;
    assign s2_wdata   = w_grant_m2 ? m2_wdata : m1_wdata;
    assign s2_wstrb   = w_grant_m2 ? m2_wstrb : m1_wstrb;
    assign s2_awvalid = (w_grant_m1 || w_grant_m2) && dec_w_apb;
    assign s2_wvalid  = (w_grant_m1 || w_grant_m2) && dec_w_apb;
    assign s2_bready  = w_grant_m2 ? m2_bready : m1_bready;

    // Phản hồi Kênh Ghi về Masters
    always @(*) begin
        m1_awready = 1'b0; m1_wready = 1'b0; m1_bvalid = 1'b0; m1_bresp = 2'b00;
        m2_awready = 1'b0; m2_wready = 1'b0; m2_bvalid = 1'b0; m2_bresp = 2'b00;

        if (w_grant_m2) begin
            if (dec_w_ram)      {m2_awready, m2_wready, m2_bvalid, m2_bresp} = {s1_awready, s1_wready, s1_bvalid, s1_bresp};
            else if (dec_w_apb) {m2_awready, m2_wready, m2_bvalid, m2_bresp} = {s2_awready, s2_wready, s2_bvalid, s2_bresp};
            else                {m2_awready, m2_wready, m2_bvalid, m2_bresp} = {1'b1, 1'b1, m2_awvalid && m2_wvalid, 2'b11}; // DECERR
        end else if (w_grant_m1) begin
            if (dec_w_ram)      {m1_awready, m1_wready, m1_bvalid, m1_bresp} = {s1_awready, s1_wready, s1_bvalid, s1_bresp};
            else if (dec_w_apb) {m1_awready, m1_wready, m1_bvalid, m1_bresp} = {s2_awready, s2_wready, s2_bvalid, s2_bresp};
            else                {m1_awready, m1_wready, m1_bvalid, m1_bresp} = {1'b1, 1'b1, m1_awvalid && m1_wvalid, 2'b11}; // DECERR
        end
    end

    // ==========================================
    // 2. GIẢI MÃ ĐỊA CHỈ KÊNH ĐỌC (READ DECODING)
    // ==========================================
    // Trọng tài ưu tiên: M2 (Debug) > M0 (I-Cache) > M1 (D-Cache)
    wire r_grant_m2 = m2_arvalid;
    wire r_grant_m0 = m0_arvalid && !m2_arvalid;
    wire r_grant_m1 = m1_arvalid && !m0_arvalid && !m2_arvalid;
    
    wire [15:0] current_araddr = r_grant_m2 ? m2_araddr : (r_grant_m0 ? m0_araddr : m1_araddr);

    // Mapping Đọc
    wire dec_r_rom = (current_araddr >= 16'h1000) && (current_araddr <= 16'h4FFF);
    wire dec_r_apb = (current_araddr >= 16'h5000) && (current_araddr <= 16'h7FFF);
    wire dec_r_ram = (current_araddr >= 16'h8000);
    wire dec_r_err = !(dec_r_rom || dec_r_ram || dec_r_apb);

    // Gán tín hiệu Read xuống S0 (ROM), S1 (RAM), S2 (APB)
    assign s0_araddr  = current_araddr;
    assign s0_arvalid = (r_grant_m0 || r_grant_m1 || r_grant_m2) && dec_r_rom;
    assign s0_rready  = r_grant_m2 ? m2_rready : (r_grant_m0 ? m0_rready : m1_rready);

    assign s1_araddr  = current_araddr;
    assign s1_arvalid = (r_grant_m0 || r_grant_m1 || r_grant_m2) && dec_r_ram;
    assign s1_rready  = r_grant_m2 ? m2_rready : (r_grant_m0 ? m0_rready : m1_rready);

    assign s2_araddr  = current_araddr;
    assign s2_arvalid = (r_grant_m0 || r_grant_m1 || r_grant_m2) && dec_r_apb;
    assign s2_rready  = r_grant_m2 ? m2_rready : (r_grant_m0 ? m0_rready : m1_rready);

    // Phản hồi Kênh Đọc về Masters
    always @(*) begin
        m0_arready = 1'b0; m0_rvalid = 1'b0; m0_rdata = 32'h0; m0_rresp = 2'b00;
        m1_arready = 1'b0; m1_rvalid = 1'b0; m1_rdata = 32'h0; m1_rresp = 2'b00;
        m2_arready = 1'b0; m2_rvalid = 1'b0; m2_rdata = 32'h0; m2_rresp = 2'b00;

        if (r_grant_m2) begin
            if (dec_r_rom)      {m2_arready, m2_rvalid, m2_rdata, m2_rresp} = {s0_arready, s0_rvalid, s0_rdata, s0_rresp};
            else if (dec_r_ram) {m2_arready, m2_rvalid, m2_rdata, m2_rresp} = {s1_arready, s1_rvalid, s1_rdata, s1_rresp};
            else if (dec_r_apb) {m2_arready, m2_rvalid, m2_rdata, m2_rresp} = {s2_arready, s2_rvalid, s2_rdata, s2_rresp};
            else                {m2_arready, m2_rvalid, m2_rdata, m2_rresp} = {1'b1, 1'b1, 32'h0, 2'b11};
        end else if (r_grant_m0) begin
            if (dec_r_rom)      {m0_arready, m0_rvalid, m0_rdata, m0_rresp} = {s0_arready, s0_rvalid, s0_rdata, s0_rresp};
            else if (dec_r_ram) {m0_arready, m0_rvalid, m0_rdata, m0_rresp} = {s1_arready, s1_rvalid, s1_rdata, s1_rresp};
            else if (dec_r_apb) {m0_arready, m0_rvalid, m0_rdata, m0_rresp} = {s2_arready, s2_rvalid, s2_rdata, s2_rresp};
            else                {m0_arready, m0_rvalid, m0_rdata, m0_rresp} = {1'b1, 1'b1, 32'h0, 2'b11};
        end else if (r_grant_m1) begin
            if (dec_r_rom)      {m1_arready, m1_rvalid, m1_rdata, m1_rresp} = {s0_arready, s0_rvalid, s0_rdata, s0_rresp};
            else if (dec_r_ram) {m1_arready, m1_rvalid, m1_rdata, m1_rresp} = {s1_arready, s1_rvalid, s1_rdata, s1_rresp};
            else if (dec_r_apb) {m1_arready, m1_rvalid, m1_rdata, m1_rresp} = {s2_arready, s2_rvalid, s2_rdata, s2_rresp};
            else                {m1_arready, m1_rvalid, m1_rdata, m1_rresp} = {1'b1, 1'b1, 32'h0, 2'b11};
        end
    end
endmodule