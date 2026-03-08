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
    // KÊNH SLAVE 3 (S3): AXI SPI FLASH
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
    // HÀM GIẢI MÃ ĐỊA CHỈ (ADDRESS DECODING LATCHED)
    // =========================================================================
    function [2:0] decode_addr;
        input [ADDR_WIDTH-1:0] addr;
        begin
            if      (addr >= 32'h0000_1000 && addr <= 32'h0000_4FFF) decode_addr = 3'd0; // S0
            else if (addr >= 32'h8000_0000 && addr <= 32'h8000_FFFF) decode_addr = 3'd1; // S1
            else if (addr >= 32'h4000_0000 && addr <= 32'h4FFF_FFFF) decode_addr = 3'd2; // S2
            else if (addr >= 32'h2000_0000 && addr <= 32'h2FFF_FFFF) decode_addr = 3'd3; // S3
            else                                                     decode_addr = 3'd4; // ERROR
        end
    endfunction

    // =========================================================================
    // TRỌNG TÀI KÊNH ĐỌC (READ ARBITRATION FSM)
    // =========================================================================
    localparam R_IDLE = 2'd0;
    localparam R_ADDR = 2'd1;
    localparam R_DATA = 2'd2;

    reg [1:0] r_state;
    reg [1:0] r_grant; 
    reg [1:0] r_token;
    reg [2:0] r_target; // Chốt Slave ID để Data phase không bị tuột tín hiệu

    wire [2:0] r_req = {m2_arvalid, m1_arvalid, m0_arvalid};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state  <= R_IDLE;
            r_grant  <= 2'd0;
            r_token  <= 2'd0;
            r_target <= 3'd0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    if (r_req != 3'b000) begin
                        r_state <= R_ADDR;
                        if (r_token == 2'd0) begin
                            if      (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; r_target <= decode_addr(m0_araddr); end
                            else if (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; r_target <= decode_addr(m1_araddr); end
                            else               begin r_grant <= 2'd2; r_token <= 2'd0; r_target <= decode_addr(m2_araddr); end
                        end else if (r_token == 2'd1) begin
                            if      (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; r_target <= decode_addr(m1_araddr); end
                            else if (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd0; r_target <= decode_addr(m2_araddr); end
                            else               begin r_grant <= 2'd0; r_token <= 2'd1; r_target <= decode_addr(m0_araddr); end
                        end else begin
                            if      (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd0; r_target <= decode_addr(m2_araddr); end
                            else if (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; r_target <= decode_addr(m0_araddr); end
                            else               begin r_grant <= 2'd1; r_token <= 2'd2; r_target <= decode_addr(m1_araddr); end
                        end
                    end
                end
                R_ADDR: begin
                    if ((r_grant == 2'd0 && m0_arvalid && m0_arready) ||
                        (r_grant == 2'd1 && m1_arvalid && m1_arready) ||
                        (r_grant == 2'd2 && m2_arvalid && m2_arready)) begin
                        r_state <= R_DATA;
                    end
                end
                R_DATA: begin
                    if ((r_grant == 2'd0 && m0_rvalid && m0_rready) ||
                        (r_grant == 2'd1 && m1_rvalid && m1_rready) ||
                        (r_grant == 2'd2 && m2_rvalid && m2_rready)) begin
                        r_state <= R_IDLE;
                    end
                end
            endcase
        end
    end

    // Định tuyến Kênh Đọc Combinational
    always @(*) begin
        m0_arready = 1'b0; m0_rvalid = 1'b0; m0_rdata = 32'h0; m0_rresp = 2'b00;
        m1_arready = 1'b0; m1_rvalid = 1'b0; m1_rdata = 32'h0; m1_rresp = 2'b00;
        m2_arready = 1'b0; m2_rvalid = 1'b0; m2_rdata = 32'h0; m2_rresp = 2'b00;
        
        s0_arvalid = 1'b0; s0_araddr = 32'h0; s0_rready = 1'b0;
        s1_arvalid = 1'b0; s1_araddr = 32'h0; s1_rready = 1'b0;
        s2_arvalid = 1'b0; s2_araddr = 32'h0; s2_rready = 1'b0;
        s3_arvalid = 1'b0; s3_araddr = 32'h0; s3_rready = 1'b0;

        if (r_state == R_ADDR) begin
            if (r_target == 3'd4) begin
                // Báo lỗi bằng cách Dummy ACK AddressPhase
                if (r_grant == 2'd0) m0_arready = 1'b1;
                if (r_grant == 2'd1) m1_arready = 1'b1;
                if (r_grant == 2'd2) m2_arready = 1'b1;
            end else begin
                case (r_target)
                    3'd0: begin
                        s0_araddr = (r_grant == 2'd0) ? m0_araddr : (r_grant == 2'd1) ? m1_araddr : m2_araddr;
                        s0_arvalid = (r_grant == 2'd0) ? m0_arvalid : (r_grant == 2'd1) ? m1_arvalid : m2_arvalid;
                        if (r_grant == 2'd0) m0_arready = s0_arready;
                        if (r_grant == 2'd1) m1_arready = s0_arready;
                        if (r_grant == 2'd2) m2_arready = s0_arready;
                    end
                    3'd1: begin
                        s1_araddr = (r_grant == 2'd0) ? m0_araddr : (r_grant == 2'd1) ? m1_araddr : m2_araddr;
                        s1_arvalid = (r_grant == 2'd0) ? m0_arvalid : (r_grant == 2'd1) ? m1_arvalid : m2_arvalid;
                        if (r_grant == 2'd0) m0_arready = s1_arready;
                        if (r_grant == 2'd1) m1_arready = s1_arready;
                        if (r_grant == 2'd2) m2_arready = s1_arready;
                    end
                    3'd2: begin
                        s2_araddr = (r_grant == 2'd0) ? m0_araddr : (r_grant == 2'd1) ? m1_araddr : m2_araddr;
                        s2_arvalid = (r_grant == 2'd0) ? m0_arvalid : (r_grant == 2'd1) ? m1_arvalid : m2_arvalid;
                        if (r_grant == 2'd0) m0_arready = s2_arready;
                        if (r_grant == 2'd1) m1_arready = s2_arready;
                        if (r_grant == 2'd2) m2_arready = s2_arready;
                    end
                    3'd3: begin
                        s3_araddr = (r_grant == 2'd0) ? m0_araddr : (r_grant == 2'd1) ? m1_araddr : m2_araddr;
                        s3_arvalid = (r_grant == 2'd0) ? m0_arvalid : (r_grant == 2'd1) ? m1_arvalid : m2_arvalid;
                        if (r_grant == 2'd0) m0_arready = s3_arready;
                        if (r_grant == 2'd1) m1_arready = s3_arready;
                        if (r_grant == 2'd2) m2_arready = s3_arready;
                    end
                endcase
            end
        end else if (r_state == R_DATA) begin
            // Chỉ định tuyến Data/Response (không nhồi địa chỉ nữa)
            if (r_target == 3'd4) begin
                if (r_grant == 2'd0) begin m0_rvalid = 1'b1; m0_rresp = 2'b11; end
                if (r_grant == 2'd1) begin m1_rvalid = 1'b1; m1_rresp = 2'b11; end
                if (r_grant == 2'd2) begin m2_rvalid = 1'b1; m2_rresp = 2'b11; end
            end else begin
                case (r_target)
                    3'd0: begin
                        if (r_grant == 2'd0) begin m0_rvalid = s0_rvalid; m0_rdata = s0_rdata; m0_rresp = s0_rresp; s0_rready = m0_rready; end
                        if (r_grant == 2'd1) begin m1_rvalid = s0_rvalid; m1_rdata = s0_rdata; m1_rresp = s0_rresp; s0_rready = m1_rready; end
                        if (r_grant == 2'd2) begin m2_rvalid = s0_rvalid; m2_rdata = s0_rdata; m2_rresp = s0_rresp; s0_rready = m2_rready; end
                    end
                    3'd1: begin
                        if (r_grant == 2'd0) begin m0_rvalid = s1_rvalid; m0_rdata = s1_rdata; m0_rresp = s1_rresp; s1_rready = m0_rready; end
                        if (r_grant == 2'd1) begin m1_rvalid = s1_rvalid; m1_rdata = s1_rdata; m1_rresp = s1_rresp; s1_rready = m1_rready; end
                        if (r_grant == 2'd2) begin m2_rvalid = s1_rvalid; m2_rdata = s1_rdata; m2_rresp = s1_rresp; s1_rready = m2_rready; end
                    end
                    3'd2: begin
                        if (r_grant == 2'd0) begin m0_rvalid = s2_rvalid; m0_rdata = s2_rdata; m0_rresp = s2_rresp; s2_rready = m0_rready; end
                        if (r_grant == 2'd1) begin m1_rvalid = s2_rvalid; m1_rdata = s2_rdata; m1_rresp = s2_rresp; s2_rready = m1_rready; end
                        if (r_grant == 2'd2) begin m2_rvalid = s2_rvalid; m2_rdata = s2_rdata; m2_rresp = s2_rresp; s2_rready = m2_rready; end
                    end
                    3'd3: begin
                        if (r_grant == 2'd0) begin m0_rvalid = s3_rvalid; m0_rdata = s3_rdata; m0_rresp = s3_rresp; s3_rready = m0_rready; end
                        if (r_grant == 2'd1) begin m1_rvalid = s3_rvalid; m1_rdata = s3_rdata; m1_rresp = s3_rresp; s3_rready = m1_rready; end
                        if (r_grant == 2'd2) begin m2_rvalid = s3_rvalid; m2_rdata = s3_rdata; m2_rresp = s3_rresp; s3_rready = m2_rready; end
                    end
                endcase
            end
        end
    end

    // =========================================================================
    // TRỌNG TÀI KÊNH GHI (WRITE ARBITRATION PARALLEL AW & W FSM)
    // =========================================================================
    localparam W_IDLE     = 2'd0;
    localparam W_TRANSFER = 2'd1;
    localparam W_RESP     = 2'd2;

    reg [1:0] w_state;
    reg       w_grant; // 0:M1, 1:M2
    reg       w_token;
    reg [2:0] w_target;
    reg       aw_done;
    reg       w_done;

    wire [1:0] w_req = {m2_awvalid, m1_awvalid};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state  <= W_IDLE;
            w_grant  <= 1'b0;
            w_token  <= 1'b0;
            w_target <= 3'd0;
            aw_done  <= 1'b0;
            w_done   <= 1'b0;
        end else begin
            case (w_state)
                W_IDLE: begin
                    aw_done <= 1'b0;
                    w_done  <= 1'b0;
                    if (w_req != 2'b00) begin
                        w_state <= W_TRANSFER;
                        if (w_token == 1'b0) begin
                            if (w_req[0]) begin w_grant <= 1'b0; w_token <= 1'b1; w_target <= decode_addr(m1_awaddr); end
                            else          begin w_grant <= 1'b1; w_token <= 1'b0; w_target <= decode_addr(m2_awaddr); end
                        end else begin
                            if (w_req[1]) begin w_grant <= 1'b1; w_token <= 1'b0; w_target <= decode_addr(m2_awaddr); end
                            else          begin w_grant <= 1'b0; w_token <= 1'b1; w_target <= decode_addr(m1_awaddr); end
                        end
                    end
                end
                W_TRANSFER: begin
                    if (w_target == 3'd4 || w_target == 3'd0 || w_target == 3'd3) begin
                        // Error/Read-Only targets (S0 ROM & S3 SPI Flash)
                        aw_done <= 1'b1;
                        w_done  <= 1'b1;
                        w_state <= W_RESP;
                    end else begin
                        if (!aw_done) begin
                            if ((w_grant == 1'b0 && m1_awvalid && m1_awready) ||
                                (w_grant == 1'b1 && m2_awvalid && m2_awready)) begin
                                aw_done <= 1'b1;
                            end
                        end
                        if (!w_done) begin
                            if ((w_grant == 1'b0 && m1_wvalid && m1_wready) ||
                                (w_grant == 1'b1 && m2_wvalid && m2_wready)) begin
                                w_done <= 1'b1;
                            end
                        end
                        
                        // Wait cho cả AW phase & W phase độc lập kết thúc
                        if ((aw_done || ((w_grant == 1'b0 && m1_awvalid && m1_awready) || (w_grant == 1'b1 && m2_awvalid && m2_awready))) &&
                            (w_done  || ((w_grant == 1'b0 && m1_wvalid && m1_wready) || (w_grant == 1'b1 && m2_wvalid && m2_wready)))) begin
                            w_state <= W_RESP;
                        end
                    end
                end
                W_RESP: begin
                    if ((w_grant == 1'b0 && m1_bvalid && m1_bready) ||
                        (w_grant == 1'b1 && m2_bvalid && m2_bready)) begin
                        w_state <= W_IDLE;
                    end
                end
            endcase
        end
    end

    // Định tuyến Kênh Ghi Combinational
    always @(*) begin
        m1_awready = 1'b0; m1_wready = 1'b0; m1_bvalid = 1'b0; m1_bresp = 2'b00;
        m2_awready = 1'b0; m2_wready = 1'b0; m2_bvalid = 1'b0; m2_bresp = 2'b00;
        
        s1_awvalid = 1'b0; s1_awaddr = 32'h0; s1_wvalid  = 1'b0; s1_wdata  = 32'h0; s1_wstrb = 4'b0000; s1_bready  = 1'b0;
        s2_awvalid = 1'b0; s2_awaddr = 32'h0; s2_wvalid  = 1'b0; s2_wdata  = 32'h0; s2_wstrb = 4'b0000; s2_bready  = 1'b0;

        if (w_state == W_TRANSFER) begin
            if (w_target == 3'd4 || w_target == 3'd0 || w_target == 3'd3) begin
                if (w_grant == 1'b0) begin m1_awready = !aw_done; m1_wready = !w_done; end
                if (w_grant == 1'b1) begin m2_awready = !aw_done; m2_wready = !w_done; end
            end else begin
                case (w_target)
                    3'd1: begin
                        s1_awaddr  = (w_grant == 1'b0) ? m1_awaddr : m2_awaddr;
                        s1_awvalid = !aw_done ? ((w_grant == 1'b0) ? m1_awvalid : m2_awvalid) : 1'b0;
                        
                        s1_wdata   = (w_grant == 1'b0) ? m1_wdata : m2_wdata;
                        s1_wstrb   = (w_grant == 1'b0) ? m1_wstrb : m2_wstrb;
                        s1_wvalid  = !w_done  ? ((w_grant == 1'b0) ? m1_wvalid : m2_wvalid) : 1'b0;
                        
                        if (w_grant == 1'b0) begin m1_awready = s1_awready && !aw_done; m1_wready = s1_wready && !w_done; end
                        if (w_grant == 1'b1) begin m2_awready = s1_awready && !aw_done; m2_wready = s1_wready && !w_done; end
                    end
                    3'd2: begin
                        s2_awaddr  = (w_grant == 1'b0) ? m1_awaddr : m2_awaddr;
                        s2_awvalid = !aw_done ? ((w_grant == 1'b0) ? m1_awvalid : m2_awvalid) : 1'b0;
                        
                        s2_wdata   = (w_grant == 1'b0) ? m1_wdata : m2_wdata;
                        s2_wstrb   = (w_grant == 1'b0) ? m1_wstrb : m2_wstrb;
                        s2_wvalid  = !w_done  ? ((w_grant == 1'b0) ? m1_wvalid : m2_wvalid) : 1'b0;
                        
                        if (w_grant == 1'b0) begin m1_awready = s2_awready && !aw_done; m1_wready = s2_wready && !w_done; end
                        if (w_grant == 1'b1) begin m2_awready = s2_awready && !aw_done; m2_wready = s2_wready && !w_done; end
                    end
                endcase
            end
        end else if (w_state == W_RESP) begin
            if (w_target == 3'd4 || w_target == 3'd0 || w_target == 3'd3) begin
                if (w_grant == 1'b0) begin m1_bvalid = 1'b1; m1_bresp = 2'b11; end
                if (w_grant == 1'b1) begin m2_bvalid = 1'b1; m2_bresp = 2'b11; end
            end else begin
                case (w_target)
                    3'd1: begin
                        if (w_grant == 1'b0) begin m1_bvalid = s1_bvalid; m1_bresp = s1_bresp; s1_bready = m1_bready; end
                        if (w_grant == 1'b1) begin m2_bvalid = s1_bvalid; m2_bresp = s1_bresp; s1_bready = m2_bready; end
                    end
                    3'd2: begin
                        if (w_grant == 1'b0) begin m1_bvalid = s2_bvalid; m1_bresp = s2_bresp; s2_bready = m1_bready; end
                        if (w_grant == 1'b1) begin m2_bvalid = s2_bvalid; m2_bresp = s2_bresp; s2_bready = m2_bready; end
                    end
                endcase
            end
        end
    end

endmodule