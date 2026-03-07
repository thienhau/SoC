`timescale 1ns / 1ps

module axi_interconnect #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input wire clk,
    input wire rst_n,

    // =========================================================================
    // KÊNH MASTER 0: I-CACHE (Chỉ Đọc)
    // =========================================================================
    input  wire [ADDR_WIDTH-1:0]  m0_araddr,
    input  wire                   m0_arvalid,
    output reg                    m0_arready,
    output reg  [DATA_WIDTH-1:0]  m0_rdata,
    output reg  [1:0]             m0_rresp,
    output reg                    m0_rvalid,
    input  wire                   m0_rready,

    // =========================================================================
    // KÊNH MASTER 1: D-CACHE (Đọc / Ghi)
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
    // KÊNH MASTER 2: DEBUG MODULE (Đọc / Ghi)
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
    // KÊNH SLAVE 0: BOOT ROM (Chỉ Đọc, Địa chỉ: 0x0000_1000 - 0x0000_4FFF)
    // =========================================================================
    output reg  [ADDR_WIDTH-1:0]  s0_araddr,
    output reg                    s0_arvalid,
    input  wire                   s0_arready,
    input  wire [DATA_WIDTH-1:0]  s0_rdata,
    input  wire [1:0]             s0_rresp,
    input  wire                   s0_rvalid,
    output reg                    s0_rready,

    // =========================================================================
    // KÊNH SLAVE 1: SYSTEM RAM (Đọc / Ghi, Địa chỉ: 0x8000_0000 - 0x8000_FFFF)
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
    // KÊNH SLAVE 2: APB BRIDGE (Đọc / Ghi, Địa chỉ: 0x4000_0000 - 0x4FFF_FFFF)
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
    output reg                    s2_rready
);

    // =========================================================================
    // FSM QUẢN LÝ KÊNH ĐỌC (READ ARBITRATION)
    // =========================================================================
    localparam R_IDLE      = 1'b0;
    localparam R_ADDR_DATA = 1'b1;
    
    reg       r_state;
    reg [1:0] r_grant; // 0: M0, 1: M1, 2: M2
    reg [1:0] r_token; // Thẻ xoay vòng

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
                        r_state <= R_ADDR_DATA;
                        case (r_token)
                            2'd0: begin
                                if      (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; end
                                else if (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; end
                                else if (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd0; end
                            end
                            2'd1: begin
                                if      (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; end
                                else if (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd0; end
                                else if (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; end
                            end
                            2'd2: begin
                                if      (r_req[2]) begin r_grant <= 2'd2; r_token <= 2'd0; end
                                else if (r_req[0]) begin r_grant <= 2'd0; r_token <= 2'd1; end
                                else if (r_req[1]) begin r_grant <= 2'd1; r_token <= 2'd2; end
                            end
                            default: begin
                                r_grant <= 2'd0; r_token <= 2'd0;
                            end
                        endcase
                    end
                end

                R_ADDR_DATA: begin
                    if ( (r_grant == 2'd0 && m0_rvalid && m0_rready) ||
                         (r_grant == 2'd1 && m1_rvalid && m1_rready) ||
                         (r_grant == 2'd2 && m2_rvalid && m2_rready) ) begin
                        r_state <= R_IDLE;
                    end
                end
            endcase
        end
    end

    // Định tuyến dữ liệu Đọc
    wire [31:0] current_araddr = (r_grant == 2'd0) ? m0_araddr : 
                                 (r_grant == 2'd1) ? m1_araddr : m2_araddr;

    wire dec_r_rom = (current_araddr >= 32'h0000_1000) && (current_araddr <= 32'h0000_4FFF);
    wire dec_r_apb = (current_araddr >= 32'h4000_0000) && (current_araddr <= 32'h4FFF_FFFF);
    wire dec_r_ram = (current_araddr >= 32'h8000_0000) && (current_araddr <= 32'h8000_FFFF);

    always @(*) begin
        m0_arready = 1'b0; m0_rvalid = 1'b0; m0_rdata = 32'h0; m0_rresp = 2'b00;
        m1_arready = 1'b0; m1_rvalid = 1'b0; m1_rdata = 32'h0; m1_rresp = 2'b00;
        m2_arready = 1'b0; m2_rvalid = 1'b0; m2_rdata = 32'h0; m2_rresp = 2'b00;
        
        s0_arvalid = 1'b0; s0_araddr = 32'h0; s0_rready = 1'b0;
        s1_arvalid = 1'b0; s1_araddr = 32'h0; s1_rready = 1'b0;
        s2_arvalid = 1'b0; s2_araddr = 32'h0; s2_rready = 1'b0;

        if (r_state == R_ADDR_DATA) begin
            if (!(dec_r_rom || dec_r_apb || dec_r_ram)) begin
                if (r_grant == 2'd0) begin m0_arready = 1'b1; m0_rvalid = 1'b1; m0_rresp = 2'b11; end // 2'b11 = DECERR
                if (r_grant == 2'd1) begin m1_arready = 1'b1; m1_rvalid = 1'b1; m1_rresp = 2'b11; end
                if (r_grant == 2'd2) begin m2_arready = 1'b1; m2_rvalid = 1'b1; m2_rresp = 2'b11; end
            end else begin
                if (dec_r_rom) begin s0_araddr = current_araddr; s0_arvalid = 1'b1; end
                if (dec_r_ram) begin s1_araddr = current_araddr; s1_arvalid = 1'b1; end
                if (dec_r_apb) begin s2_araddr = current_araddr; s2_arvalid = 1'b1; end

                if (r_grant == 2'd0) begin
                    m0_arready = dec_r_rom ? s0_arready : dec_r_ram ? s1_arready : s2_arready;
                    m0_rdata   = dec_r_rom ? s0_rdata   : dec_r_ram ? s1_rdata   : s2_rdata;
                    m0_rresp   = dec_r_rom ? s0_rresp   : dec_r_ram ? s1_rresp   : s2_rresp;
                    m0_rvalid  = dec_r_rom ? s0_rvalid  : dec_r_ram ? s1_rvalid  : s2_rvalid;
                    s0_rready  = dec_r_rom ? m0_rready  : 1'b0;
                    s1_rready  = dec_r_ram ? m0_rready  : 1'b0;
                    s2_rready  = dec_r_apb ? m0_rready  : 1'b0;
                end else if (r_grant == 2'd1) begin
                    m1_arready = dec_r_rom ? s0_arready : dec_r_ram ? s1_arready : s2_arready;
                    m1_rdata   = dec_r_rom ? s0_rdata   : dec_r_ram ? s1_rdata   : s2_rdata;
                    m1_rresp   = dec_r_rom ? s0_rresp   : dec_r_ram ? s1_rresp   : s2_rresp;
                    m1_rvalid  = dec_r_rom ? s0_rvalid  : dec_r_ram ? s1_rvalid  : s2_rvalid;
                    s0_rready  = dec_r_rom ? m1_rready  : 1'b0;
                    s1_rready  = dec_r_ram ? m1_rready  : 1'b0;
                    s2_rready  = dec_r_apb ? m1_rready  : 1'b0;
                end else if (r_grant == 2'd2) begin
                    m2_arready = dec_r_rom ? s0_arready : dec_r_ram ? s1_arready : s2_arready;
                    m2_rdata   = dec_r_rom ? s0_rdata   : dec_r_ram ? s1_rdata   : s2_rdata;
                    m2_rresp   = dec_r_rom ? s0_rresp   : dec_r_ram ? s1_rresp   : s2_rresp;
                    m2_rvalid  = dec_r_rom ? s0_rvalid  : dec_r_ram ? s1_rvalid  : s2_rvalid;
                    s0_rready  = dec_r_rom ? m2_rready  : 1'b0;
                    s1_rready  = dec_r_ram ? m2_rready  : 1'b0;
                    s2_rready  = dec_r_apb ? m2_rready  : 1'b0;
                end
            end
        end
    end

    // =========================================================================
    // FSM QUẢN LÝ KÊNH GHI (WRITE ARBITRATION)
    // =========================================================================
    localparam W_IDLE   = 1'b0;
    localparam W_ACTIVE = 1'b1;
    
    reg w_state;
    reg w_grant; // 0: M1, 1: M2
    reg w_token;

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
                            if      (w_req[0]) begin w_grant <= 1'b0; w_token <= 1'b1; end
                            else if (w_req[1]) begin w_grant <= 1'b1; w_token <= 1'b0; end
                        end else begin
                            if      (w_req[1]) begin w_grant <= 1'b1; w_token <= 1'b0; end
                            else if (w_req[0]) begin w_grant <= 1'b0; w_token <= 1'b1; end
                        end
                    end
                end

                W_ACTIVE: begin
                    if ( (w_grant == 1'b0 && m1_bvalid && m1_bready) ||
                         (w_grant == 1'b1 && m2_bvalid && m2_bready) ) begin
                        w_state <= W_IDLE;
                    end
                end
            endcase
        end
    end

    // Định tuyến dữ liệu Ghi
    wire [31:0] current_awaddr = (w_grant == 1'b0) ? m1_awaddr : m2_awaddr;

    wire dec_w_apb = (current_awaddr >= 32'h4000_0000) && (current_awaddr <= 32'h4FFF_FFFF);
    wire dec_w_ram = (current_awaddr >= 32'h8000_0000) && (current_awaddr <= 32'h8000_FFFF);

    always @(*) begin
        m1_awready = 1'b0; m1_wready = 1'b0; m1_bvalid = 1'b0; m1_bresp = 2'b00;
        m2_awready = 1'b0; m2_wready = 1'b0; m2_bvalid = 1'b0; m2_bresp = 2'b00;

        s1_awvalid = 1'b0; s1_awaddr = 32'h0; s1_wvalid = 1'b0; s1_wdata = 32'h0; s1_wstrb = 4'h0; s1_bready = 1'b0;
        s2_awvalid = 1'b0; s2_awaddr = 32'h0; s2_wvalid = 1'b0; s2_wdata = 32'h0; s2_wstrb = 4'h0; s2_bready = 1'b0;

        if (w_state == W_ACTIVE) begin
            if (!(dec_w_apb || dec_w_ram)) begin
                if (w_grant == 1'b0) begin m1_awready = 1'b1; m1_wready = 1'b1; m1_bvalid = 1'b1; m1_bresp = 2'b11; end
                if (w_grant == 1'b1) begin m2_awready = 1'b1; m2_wready = 1'b1; m2_bvalid = 1'b1; m2_bresp = 2'b11; end
            end else begin
                if (dec_w_ram) begin
                    s1_awaddr  = current_awaddr;
                    s1_wdata   = (w_grant == 1'b0) ? m1_wdata : m2_wdata;
                    s1_wstrb   = (w_grant == 1'b0) ? m1_wstrb : m2_wstrb;
                    s1_awvalid = (w_grant == 1'b0) ? m1_awvalid : m2_awvalid;
                    s1_wvalid  = (w_grant == 1'b0) ? m1_wvalid : m2_wvalid;
                end
                if (dec_w_apb) begin
                    s2_awaddr  = current_awaddr;
                    s2_wdata   = (w_grant == 1'b0) ? m1_wdata : m2_wdata;
                    s2_wstrb   = (w_grant == 1'b0) ? m1_wstrb : m2_wstrb;
                    s2_awvalid = (w_grant == 1'b0) ? m1_awvalid : m2_awvalid;
                    s2_wvalid  = (w_grant == 1'b0) ? m1_wvalid : m2_wvalid;
                end

                if (w_grant == 1'b0) begin
                    m1_awready = dec_w_ram ? s1_awready : s2_awready;
                    m1_wready  = dec_w_ram ? s1_wready  : s2_wready;
                    m1_bresp   = dec_w_ram ? s1_bresp   : s2_bresp;
                    m1_bvalid  = dec_w_ram ? s1_bvalid  : s2_bvalid;
                    s1_bready  = dec_w_ram ? m1_bready  : 1'b0;
                    s2_bready  = dec_w_apb ? m1_bready  : 1'b0;
                end else if (w_grant == 1'b1) begin
                    m2_awready = dec_w_ram ? s1_awready : s2_awready;
                    m2_wready  = dec_w_ram ? s1_wready  : s2_wready;
                    m2_bresp   = dec_w_ram ? s1_bresp   : s2_bresp;
                    m2_bvalid  = dec_w_ram ? s1_bvalid  : s2_bvalid;
                    s1_bready  = dec_w_ram ? m2_bready  : 1'b0;
                    s2_bready  = dec_w_apb ? m2_bready  : 1'b0;
                end
            end
        end
    end

endmodule