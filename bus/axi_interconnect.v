`timescale 1ns / 1ps

module axi_interconnect #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32
)(
    input wire clk,
    input wire rst_n,

    // =========================================
    // KHAI BÁO PORT (Giữ nguyên như cũ)
    // =========================================
    // --- M0: I-Cache (Read Only) ---
    input wire [ADDR_WIDTH-1:0] m0_araddr, input wire m0_arvalid, output reg m0_arready,
    output reg [DATA_WIDTH-1:0] m0_rdata, output reg [1:0] m0_rresp, output reg m0_rvalid, input wire m0_rready,

    // --- M1: D-Cache (Read & Write) ---
    input wire [ADDR_WIDTH-1:0] m1_awaddr, input wire m1_awvalid, output reg m1_awready,
    input wire [DATA_WIDTH-1:0] m1_wdata, input wire [3:0] m1_wstrb, input wire m1_wvalid, output reg m1_wready,
    output reg [1:0] m1_bresp, output reg m1_bvalid, input wire m1_bready,
    input wire [ADDR_WIDTH-1:0] m1_araddr, input wire m1_arvalid, output reg m1_arready,
    output reg [DATA_WIDTH-1:0] m1_rdata, output reg [1:0] m1_rresp, output reg m1_rvalid, input wire m1_rready,

    // --- M2: Debug (Read & Write) ---
    input wire [ADDR_WIDTH-1:0] m2_awaddr, input wire m2_awvalid, output reg m2_awready,
    input wire [DATA_WIDTH-1:0] m2_wdata, input wire [3:0] m2_wstrb, input wire m2_wvalid, output reg m2_wready,
    output reg [1:0] m2_bresp, output reg m2_bvalid, input wire m2_bready,
    input wire [ADDR_WIDTH-1:0] m2_araddr, input wire m2_arvalid, output reg m2_arready,
    output reg [DATA_WIDTH-1:0] m2_rdata, output reg [1:0] m2_rresp, output reg m2_rvalid, input wire m2_rready,

    // --- S0: ROM (Read Only: 0x1000 - 0x4FFF) ---
    output reg [ADDR_WIDTH-1:0] s0_araddr, output reg s0_arvalid, input wire s0_arready,
    input wire [DATA_WIDTH-1:0] s0_rdata, input wire [1:0] s0_rresp, input wire s0_rvalid, output reg s0_rready,

    // --- S1: RAM (R/W: 0x8000 - 0xFFFF) ---
    output reg [ADDR_WIDTH-1:0] s1_awaddr, output reg s1_awvalid, input wire s1_awready,
    output reg [DATA_WIDTH-1:0] s1_wdata, output reg [3:0] s1_wstrb, output reg s1_wvalid, input wire s1_wready,
    input wire [1:0] s1_bresp, input wire s1_bvalid, output reg s1_bready,
    output reg [ADDR_WIDTH-1:0] s1_araddr, output reg s1_arvalid, input wire s1_arready,
    input wire [DATA_WIDTH-1:0] s1_rdata, input wire [1:0] s1_rresp, input wire s1_rvalid, output reg s1_rready,

    // --- S2: APB (R/W: 0x5000 - 0x7FFF) ---
    output reg [ADDR_WIDTH-1:0] s2_awaddr, output reg s2_awvalid, input wire s2_awready,
    output reg [DATA_WIDTH-1:0] s2_wdata, output reg [3:0] s2_wstrb, output reg s2_wvalid, input wire s2_wready,
    input wire [1:0] s2_bresp, input wire s2_bvalid, output reg s2_bready,
    output reg [ADDR_WIDTH-1:0] s2_araddr, output reg s2_arvalid, input wire s2_arready,
    input wire [DATA_WIDTH-1:0] s2_rdata, input wire [1:0] s2_rresp, input wire s2_rvalid, output reg s2_rready
);

    // =====================================================================
    // 1. KÊNH ĐỌC (READ CHANNEL) - ROUND ROBIN ARBITRATION & LOCKING
    // =====================================================================
    localparam R_IDLE = 0, R_ADDR_DATA = 1;
    reg r_state;
    reg [1:0] r_grant; // 0: M0, 1: M1, 2: M2
    reg [1:0] r_token; // Thẻ bài xoay vòng

    wire [2:0] r_req = {m2_arvalid, m1_arvalid, m0_arvalid};

    // FSM Quản lý kênh đọc
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_state <= R_IDLE;
            r_grant <= 2'b00;
            r_token <= 2'b00;
        end else begin
            case (r_state)
                R_IDLE: begin
                    if (r_req != 3'b000) begin
                        r_state <= R_ADDR_DATA; // Khóa Bus
                        // Round-Robin Logic
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
                        endcase
                    end
                end

                R_ADDR_DATA: begin
                    // Đợi đến khi giao dịch kết thúc (RVALID & RREADY)
                    if ( (r_grant == 0 && m0_rvalid && m0_rready) ||
                         (r_grant == 1 && m1_rvalid && m1_rready) ||
                         (r_grant == 2 && m2_rvalid && m2_rready) ) begin
                        r_state <= R_IDLE; // Mở khóa Bus
                    end
                end
            endcase
        end
    end

    // Định tuyến Kênh Đọc (Read Routing)
    wire [15:0] current_araddr = (r_grant == 0) ? m0_araddr : 
                                 (r_grant == 1) ? m1_araddr : m2_araddr;

    wire dec_r_rom = (current_araddr >= 16'h1000) && (current_araddr <= 16'h4FFF);
    wire dec_r_apb = (current_araddr >= 16'h5000) && (current_araddr <= 16'h7FFF);
    wire dec_r_ram = (current_araddr >= 16'h8000);

    always @(*) begin
        // Trạng thái mặc định (Default values)
        {m0_arready, m1_arready, m2_arready} = 3'b000;
        {s0_arvalid, s1_arvalid, s2_arvalid} = 3'b000;
        {s0_rready, s1_rready, s2_rready}    = 3'b000;
        
        m0_rdata = 0; m1_rdata = 0; m2_rdata = 0;
        m0_rresp = 0; m1_rresp = 0; m2_rresp = 0;
        m0_rvalid = 0; m1_rvalid = 0; m2_rvalid = 0;

        if (r_state == R_ADDR_DATA) begin
            // Địa chỉ rác -> Phản hồi lập tức lỗi
            if (!(dec_r_rom || dec_r_apb || dec_r_ram)) begin
                if (r_grant==0) begin m0_arready=1; m0_rvalid=1; m0_rresp=2'b11; end
                if (r_grant==1) begin m1_arready=1; m1_rvalid=1; m1_rresp=2'b11; end
                if (r_grant==2) begin m2_arready=1; m2_rvalid=1; m2_rresp=2'b11; end
            end 
            else begin
                // Forward Address (M -> S)
                if (dec_r_rom) begin s0_araddr = current_araddr; s0_arvalid = 1'b1; end
                if (dec_r_ram) begin s1_araddr = current_araddr; s1_arvalid = 1'b1; end
                if (dec_r_apb) begin s2_araddr = current_araddr; s2_arvalid = 1'b1; end

                // Forward Data (S -> M)
                if (r_grant == 0) begin
                    m0_arready = dec_r_rom ? s0_arready : dec_r_ram ? s1_arready : s2_arready;
                    m0_rdata   = dec_r_rom ? s0_rdata   : dec_r_ram ? s1_rdata   : s2_rdata;
                    m0_rresp   = dec_r_rom ? s0_rresp   : dec_r_ram ? s1_rresp   : s2_rresp;
                    m0_rvalid  = dec_r_rom ? s0_rvalid  : dec_r_ram ? s1_rvalid  : s2_rvalid;
                    s0_rready  = dec_r_rom ? m0_rready  : 0;
                    s1_rready  = dec_r_ram ? m0_rready  : 0;
                    s2_rready  = dec_r_apb ? m0_rready  : 0;
                end
                else if (r_grant == 1) begin
                    m1_arready = dec_r_rom ? s0_arready : dec_r_ram ? s1_arready : s2_arready;
                    m1_rdata   = dec_r_rom ? s0_rdata   : dec_r_ram ? s1_rdata   : s2_rdata;
                    m1_rresp   = dec_r_rom ? s0_rresp   : dec_r_ram ? s1_rresp   : s2_rresp;
                    m1_rvalid  = dec_r_rom ? s0_rvalid  : dec_r_ram ? s1_rvalid  : s2_rvalid;
                    s0_rready  = dec_r_rom ? m1_rready  : 0;
                    s1_rready  = dec_r_ram ? m1_rready  : 0;
                    s2_rready  = dec_r_apb ? m1_rready  : 0;
                end
                else if (r_grant == 2) begin
                    m2_arready = dec_r_rom ? s0_arready : dec_r_ram ? s1_arready : s2_arready;
                    m2_rdata   = dec_r_rom ? s0_rdata   : dec_r_ram ? s1_rdata   : s2_rdata;
                    m2_rresp   = dec_r_rom ? s0_rresp   : dec_r_ram ? s1_rresp   : s2_rresp;
                    m2_rvalid  = dec_r_rom ? s0_rvalid  : dec_r_ram ? s1_rvalid  : s2_rvalid;
                    s0_rready  = dec_r_rom ? m2_rready  : 0;
                    s1_rready  = dec_r_ram ? m2_rready  : 0;
                    s2_rready  = dec_r_apb ? m2_rready  : 0;
                end
            end
        end
    end

    // =====================================================================
    // 2. KÊNH GHI (WRITE CHANNEL) - ROUND ROBIN ARBITRATION & LOCKING
    // =====================================================================
    // Chỉ có M1 (DCache) và M2 (Debug) mới có khả năng ghi
    localparam W_IDLE = 0, W_ACTIVE = 1;
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
                        w_state <= W_ACTIVE; // Khóa Bus
                        if (w_token == 0) begin
                            if (w_req[0]) begin w_grant <= 0; w_token <= 1; end
                            else          begin w_grant <= 1; w_token <= 0; end
                        end else begin
                            if (w_req[1]) begin w_grant <= 1; w_token <= 0; end
                            else          begin w_grant <= 0; w_token <= 1; end
                        end
                    end
                end

                W_ACTIVE: begin
                    // Kết thúc khi BVALID & BREADY (Giao dịch hoàn tất 100%)
                    if ( (w_grant == 0 && m1_bvalid && m1_bready) ||
                         (w_grant == 1 && m2_bvalid && m2_bready) ) begin
                        w_state <= W_IDLE;
                    end
                end
            endcase
        end
    end

    // Định tuyến Kênh Ghi (Write Routing)
    wire [15:0] current_awaddr = (w_grant == 0) ? m1_awaddr : m2_awaddr;

    wire dec_w_apb = (current_awaddr >= 16'h5000) && (current_awaddr <= 16'h7FFF);
    wire dec_w_ram = (current_awaddr >= 16'h8000);

    always @(*) begin
        // Reset mặc định
        {m1_awready, m2_awready, m1_wready, m2_wready} = 4'b0000;
        {s1_awvalid, s2_awvalid, s1_wvalid, s2_wvalid} = 4'b0000;
        {s1_bready, s2_bready} = 2'b00;

        m1_bresp = 0; m2_bresp = 0;
        m1_bvalid = 0; m2_bvalid = 0;
        s1_awaddr = 0; s2_awaddr = 0;
        s1_wdata = 0; s2_wdata = 0;
        s1_wstrb = 0; s2_wstrb = 0;

        if (w_state == W_ACTIVE) begin
            if (!(dec_w_apb || dec_w_ram)) begin
                // Lỗi địa chỉ
                if (w_grant == 0) begin m1_awready=1; m1_wready=1; m1_bvalid=1; m1_bresp=2'b11; end
                if (w_grant == 1) begin m2_awready=1; m2_wready=1; m2_bvalid=1; m2_bresp=2'b11; end
            end else begin
                // Phân phối dữ liệu (M -> S)
                if (dec_w_ram) begin
                    s1_awaddr  = current_awaddr;
                    s1_wdata   = w_grant ? m2_wdata : m1_wdata;
                    s1_wstrb   = w_grant ? m2_wstrb : m1_wstrb;
                    s1_awvalid = w_grant ? m2_awvalid : m1_awvalid;
                    s1_wvalid  = w_grant ? m2_wvalid  : m1_wvalid;
                end
                if (dec_w_apb) begin
                    s2_awaddr  = current_awaddr;
                    s2_wdata   = w_grant ? m2_wdata : m1_wdata;
                    s2_wstrb   = w_grant ? m2_wstrb : m1_wstrb;
                    s2_awvalid = w_grant ? m2_awvalid : m1_awvalid;
                    s2_wvalid  = w_grant ? m2_wvalid  : m1_wvalid;
                end

                // Phân phối phản hồi (S -> M)
                if (w_grant == 0) begin
                    m1_awready = dec_w_ram ? s1_awready : s2_awready;
                    m1_wready  = dec_w_ram ? s1_wready  : s2_wready;
                    m1_bresp   = dec_w_ram ? s1_bresp   : s2_bresp;
                    m1_bvalid  = dec_w_ram ? s1_bvalid  : s2_bvalid;
                    s1_bready  = dec_w_ram ? m1_bready  : 0;
                    s2_bready  = dec_w_apb ? m1_bready  : 0;
                end else begin
                    m2_awready = dec_w_ram ? s1_awready : s2_awready;
                    m2_wready  = dec_w_ram ? s1_wready  : s2_wready;
                    m2_bresp   = dec_w_ram ? s1_bresp   : s2_bresp;
                    m2_bvalid  = dec_w_ram ? s1_bvalid  : s2_bvalid;
                    s1_bready  = dec_w_ram ? m2_bready  : 0;
                    s2_bready  = dec_w_apb ? m2_bready  : 0;
                end
            end
        end
    end

endmodule