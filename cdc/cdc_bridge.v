`timescale 1ns / 1ps

module native_cdc_bridge (
    // --- CPU Domain (Ví dụ: 400MHz) ---
    input  wire        cpu_clk,
    input  wire        cpu_rst_n,
    input  wire        cpu_req_val,
    input  wire        cpu_req_is_write,
    input  wire [31:0] cpu_req_addr,
    input  wire [31:0] cpu_req_wdata,
    input  wire [3:0]  cpu_req_wstrb,   // THÊM: Byte Strobe cho lệnh ghi
    input  wire [1:0]  cpu_req_size,
    output wire        cpu_req_ready,
    output wire        cpu_resp_val,
    output wire [31:0] cpu_resp_rdata,

    // --- Bus Domain (Ví dụ: 300MHz) ---
    input  wire        bus_clk,
    input  wire        bus_rst_n,
    output wire        bus_req_val,
    output wire        bus_req_is_write,
    output wire [31:0] bus_req_addr,
    output wire [31:0] bus_req_wdata,
    output wire [3:0]  bus_req_wstrb,   // THÊM: Byte Strobe
    output wire [1:0]  bus_req_size,
    input  wire        bus_req_ready,
    input  wire        bus_resp_val,
    input  wire [31:0] bus_resp_rdata
);
    // Kênh Request (CPU -> Bus)
    // Độ rộng = is_write(1) + addr(32) + wdata(32) + wstrb(4) + size(2) = 71 bits
    wire [70:0] req_wdata = {cpu_req_is_write, cpu_req_addr, cpu_req_wdata, cpu_req_wstrb, cpu_req_size};
    wire [70:0] req_rdata;
    wire req_full, req_empty;
    
    async_fifo #(.DATA_WIDTH(71), .ADDR_WIDTH(4)) REQ_FIFO (
        .wclk(cpu_clk), .wrst_n(cpu_rst_n), 
        .winc(cpu_req_val && cpu_req_ready), 
        .wdata(req_wdata), 
        .wfull(req_full),
        .rclk(bus_clk), .rrst_n(bus_rst_n), 
        .rinc(bus_req_ready && !req_empty), 
        .rdata(req_rdata), 
        .rempty(req_empty)
    );

    assign cpu_req_ready = !req_full;
    assign bus_req_val   = !req_empty;
    assign {bus_req_is_write, bus_req_addr, bus_req_wdata, bus_req_wstrb, bus_req_size} = req_rdata;

    // Kênh Response (Bus -> CPU)
    wire resp_full, resp_empty;
    async_fifo #(.DATA_WIDTH(32), .ADDR_WIDTH(4)) RESP_FIFO (
        .wclk(bus_clk), .wrst_n(bus_rst_n), 
        .winc(bus_resp_val && !resp_full), 
        .wdata(bus_resp_rdata), 
        .wfull(resp_full),
        .rclk(cpu_clk), .rrst_n(cpu_rst_n), 
        .rinc(!resp_empty), // Tự động pop khi có dữ liệu
        .rdata(cpu_resp_rdata), 
        .rempty(resp_empty)
    );

    // cpu_resp_val cần được đồng bộ hóa chính xác để Cache nhận biết dữ liệu mới
    assign cpu_resp_val = !resp_empty;

endmodule

`timescale 1ns / 1ps

// =============================================================================
// SUB-MODULE: Asynchronous FIFO (Sử dụng Gray Code)
// =============================================================================
module async_fifo_cdc #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 4 // Sâu 16 words
)(
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  winc,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire                  wfull,

    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  rinc,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire                  rempty
);
    reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
    reg [ADDR_WIDTH:0] wptr, rptr;
    reg [ADDR_WIDTH:0] wq2_rptr, wq1_rptr, rq2_wptr, rq1_wptr;

    wire [ADDR_WIDTH:0] wptr_gray = wptr ^ (wptr >> 1);
    wire [ADDR_WIDTH:0] rptr_gray = rptr ^ (rptr >> 1);

    // Đồng bộ pointer từ miền R sang miền W
    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) {wq2_rptr, wq1_rptr} <= 0;
        else         {wq2_rptr, wq1_rptr} <= {wq1_rptr, rptr_gray};
    end

    // Đồng bộ pointer từ miền W sang miền R
    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) {rq2_wptr, rq1_wptr} <= 0;
        else         {rq2_wptr, rq1_wptr} <= {rq1_wptr, wptr_gray};
    end

    assign rempty = (rptr_gray == rq2_wptr);
    assign wfull  = (wptr_gray == {~wq2_rptr[ADDR_WIDTH:ADDR_WIDTH-1], wq2_rptr[ADDR_WIDTH-2:0]});

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) wptr <= 0;
        else if (winc && !wfull) begin
            mem[wptr[ADDR_WIDTH-1:0]] <= wdata;
            wptr <= wptr + 1;
        end
    end

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) rptr <= 0;
        else if (rinc && !rempty) rptr <= rptr + 1;
    end

    assign rdata = mem[rptr[ADDR_WIDTH-1:0]];
endmodule

// =============================================================================
// MAIN MODULE: AXI4 Burst Read CDC Bridge
// =============================================================================
module axi4_read_cdc #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    // --- Miền Core (Gắn với I-Cache) ---
    input  wire clk_core,
    input  wire rst_core_n,
    input  wire [ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire [7:0]            s_axi_arlen,
    input  wire [2:0]            s_axi_arsize,
    input  wire [1:0]            s_axi_arburst,
    input  wire                  s_axi_arvalid,
    output wire                  s_axi_arready,

    output wire [DATA_WIDTH-1:0] s_axi_rdata,
    output wire [1:0]            s_axi_rresp,
    output wire                  s_axi_rlast,
    output wire                  s_axi_rvalid,
    input  wire                  s_axi_rready,

    // --- Miền Bus (Gắn với Interconnect) ---
    input  wire clk_bus,
    input  wire rst_bus_n,
    output wire [ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]            m_axi_arlen,
    output wire [2:0]            m_axi_arsize,
    output wire [1:0]            m_axi_arburst,
    output wire                  m_axi_arvalid,
    input  wire                  m_axi_arready,

    input  wire [DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output wire                  m_axi_rready
);

    // Kênh AR (Address Read): Master -> Slave
    wire ar_empty, ar_full;
    wire [44:0] ar_wdata = {s_axi_araddr, s_axi_arlen, s_axi_arsize, s_axi_arburst};
    wire [44:0] ar_rdata;
    
    assign s_axi_arready = ~ar_full;
    assign m_axi_arvalid = ~ar_empty;
    assign {m_axi_araddr, m_axi_arlen, m_axi_arsize, m_axi_arburst} = ar_rdata;

    async_fifo_cdc #(.DATA_WIDTH(45), .ADDR_WIDTH(4)) FIFO_AR (
        .wclk(clk_core), .wrst_n(rst_core_n), .winc(s_axi_arvalid & ~ar_full), .wdata(ar_wdata), .wfull(ar_full),
        .rclk(clk_bus),  .rrst_n(rst_bus_n),  .rinc(m_axi_arready & ~ar_empty), .rdata(ar_rdata), .rempty(ar_empty)
    );

    // Kênh R (Data Read): Slave -> Master
    wire r_empty, r_full;
    wire [34:0] r_wdata = {m_axi_rdata, m_axi_rresp, m_axi_rlast};
    wire [34:0] r_rdata;

    assign m_axi_rready = ~r_full;
    assign s_axi_rvalid = ~r_empty;
    assign {s_axi_rdata, s_axi_rresp, s_axi_rlast} = r_rdata;

    async_fifo_cdc #(.DATA_WIDTH(35), .ADDR_WIDTH(4)) FIFO_R (
        .wclk(clk_bus),  .wrst_n(rst_bus_n),  .winc(m_axi_rvalid & ~r_full), .wdata(r_wdata), .wfull(r_full),
        .rclk(clk_core), .rrst_n(rst_core_n), .rinc(s_axi_rready & ~r_empty), .rdata(r_rdata), .rempty(r_empty)
    );

endmodule