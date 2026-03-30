`timescale 1ns / 1ps

module top_soc (
    input  wire        clk,
    input  wire        rst_n,

    // JTAG Interface
    input  wire        tck,
    input  wire        trst_n,
    input  wire        tms,
    input  wire        tdi,
    output wire        tdo,

    // UART
    input  wire        uart_rx,
    output wire        uart_tx,

    // GPIO
    input  wire [31:0] gpio_in,
    output wire [31:0] gpio_out,
    output wire [31:0] gpio_oe,

    // SPI (Peripheral)
    output wire        spi_sck,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_ss,

    // I2C
    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    // SPI Flash Interface (Memory Mapped)
    output wire        flash_sck,
    output wire        flash_cs,
    output wire        flash_mosi,
    input  wire        flash_miso,

    // SDRAM Interface
    output wire        sdram_clk,
    output wire        sdram_cke,
    output wire        sdram_cs_n,
    output wire        sdram_ras_n,
    output wire        sdram_cas_n,
    output wire        sdram_we_n,
    output wire [1:0]  sdram_ba,
    output wire [12:0] sdram_addr,
    inout  wire [15:0] sdram_dq,
    output wire [1:0]  sdram_dqm
);

    // =========================================================================
    // 1. DÂY TÍN HIỆU GIỮA CPU VÀ CACHES
    // =========================================================================
    wire [31:0] cpu_inst_addr, cpu_inst_data;
    wire        cpu_inst_req, cpu_inst_hit, cpu_inst_stall;

    wire [31:0] cpu_data_addr, cpu_data_wdata, cpu_data_rdata;
    wire        cpu_data_rd_req, cpu_data_wr_req, cpu_data_hit, cpu_data_stall, cpu_data_unsigned;
    wire [1:0]  cpu_data_size;

    // =========================================================================
    // 2. KHỞI TẠO CPU LÕI RISC-V
    // =========================================================================
    riscv_pipeline u_core (
        .clk             (clk),
        .rst_n           (rst_n),
        .inst_req        (cpu_inst_req),
        .inst_addr       (cpu_inst_addr),
        .inst_data       (cpu_inst_data),
        .inst_hit        (cpu_inst_hit),
        .inst_stall      (cpu_inst_stall),
        .data_rd_req     (cpu_data_rd_req),
        .data_wr_req     (cpu_data_wr_req),
        .data_addr       (cpu_data_addr),
        .data_wdata      (cpu_data_wdata),
        .data_rdata      (cpu_data_rdata),
        .data_size       (cpu_data_size),
        .data_unsigned   (cpu_data_unsigned),
        .data_hit        (cpu_data_hit),
        .data_stall      (cpu_data_stall)
    );

    // =========================================================================
    // 3. KHAI BÁO CÁC KÊNH AXI (3 MASTERS, 5 SLAVES)
    // =========================================================================
    localparam MST_AMT = 3;
    localparam SLV_AMT = 5;

    // Các tín hiệu Master phẳng (Flattened arrays cho AXI Interconnect)
    wire [MST_AMT*5-1:0]  m_axi_awid;
    wire [MST_AMT*32-1:0] m_axi_awaddr;
    wire [MST_AMT*8-1:0]  m_axi_awlen;
    wire [MST_AMT*3-1:0]  m_axi_awsize;
    wire [MST_AMT*2-1:0]  m_axi_awburst;
    wire [MST_AMT*3-1:0]  m_axi_awprot;
    wire [MST_AMT-1:0]    m_axi_awvalid;
    wire [MST_AMT-1:0]    m_axi_awready;

    wire [MST_AMT*32-1:0] m_axi_wdata;
    wire [MST_AMT*4-1:0]  m_axi_wstrb;
    wire [MST_AMT-1:0]    m_axi_wlast;
    wire [MST_AMT-1:0]    m_axi_wvalid;
    wire [MST_AMT-1:0]    m_axi_wready;

    wire [MST_AMT*5-1:0]  m_axi_bid;
    wire [MST_AMT*2-1:0]  m_axi_bresp;
    wire [MST_AMT-1:0]    m_axi_bvalid;
    wire [MST_AMT-1:0]    m_axi_bready;

    wire [MST_AMT*5-1:0]  m_axi_arid;
    wire [MST_AMT*32-1:0] m_axi_araddr;
    wire [MST_AMT*8-1:0]  m_axi_arlen;
    wire [MST_AMT*3-1:0]  m_axi_arsize;
    wire [MST_AMT*2-1:0]  m_axi_arburst;
    wire [MST_AMT*3-1:0]  m_axi_arprot;
    wire [MST_AMT-1:0]    m_axi_arvalid;
    wire [MST_AMT-1:0]    m_axi_arready;

    wire [MST_AMT*5-1:0]  m_axi_rid;
    wire [MST_AMT*32-1:0] m_axi_rdata;
    wire [MST_AMT*2-1:0]  m_axi_rresp;
    wire [MST_AMT-1:0]    m_axi_rlast;
    wire [MST_AMT-1:0]    m_axi_rvalid;
    wire [MST_AMT-1:0]    m_axi_rready;

    // Dây tín hiệu từng Master đơn lẻ
    // M0: ICache
    wire [4:0]  m0_awid;    wire [31:0] m0_awaddr;  wire [7:0]  m0_awlen;   wire [2:0]  m0_awsize;  wire [1:0]  m0_awburst; wire [2:0]  m0_awprot;  wire m0_awvalid; wire m0_awready;
    wire [31:0] m0_wdata;   wire [3:0]  m0_wstrb;   wire        m0_wlast;   wire        m0_wvalid;  wire m0_wready;
    wire [4:0]  m0_bid;     wire [1:0]  m0_bresp;   wire        m0_bvalid;  wire        m0_bready;
    wire [4:0]  m0_arid;    wire [31:0] m0_araddr;  wire [7:0]  m0_arlen;   wire [2:0]  m0_arsize;  wire [1:0]  m0_arburst; wire [2:0]  m0_arprot;  wire m0_arvalid; wire m0_arready;
    wire [4:0]  m0_rid;     wire [31:0] m0_rdata;   wire [1:0]  m0_rresp;   wire        m0_rlast;   wire m0_rvalid; wire m0_rready;

    // M1: DCache
    wire [4:0]  m1_awid;    wire [31:0] m1_awaddr;  wire [7:0]  m1_awlen;   wire [2:0]  m1_awsize;  wire [1:0]  m1_awburst; wire [2:0]  m1_awprot;  wire m1_awvalid; wire m1_awready;
    wire [31:0] m1_wdata;   wire [3:0]  m1_wstrb;   wire        m1_wlast;   wire        m1_wvalid;  wire m1_wready;
    wire [4:0]  m1_bid;     wire [1:0]  m1_bresp;   wire        m1_bvalid;  wire        m1_bready;
    wire [4:0]  m1_arid;    wire [31:0] m1_araddr;  wire [7:0]  m1_arlen;   wire [2:0]  m1_arsize;  wire [1:0]  m1_arburst; wire [2:0]  m1_arprot;  wire m1_arvalid; wire m1_arready;
    wire [4:0]  m1_rid;     wire [31:0] m1_rdata;   wire [1:0]  m1_rresp;   wire        m1_rlast;   wire m1_rvalid; wire m1_rready;

    // M2: Debug DTM
    wire [4:0]  m2_awid;    wire [31:0] m2_awaddr;  wire [7:0]  m2_awlen;   wire [2:0]  m2_awsize;  wire [1:0]  m2_awburst; wire [2:0]  m2_awprot;  wire m2_awvalid; wire m2_awready;
    wire [31:0] m2_wdata;   wire [3:0]  m2_wstrb;   wire        m2_wlast;   wire        m2_wvalid;  wire m2_wready;
    wire [4:0]  m2_bid;     wire [1:0]  m2_bresp;   wire        m2_bvalid;  wire        m2_bready;
    wire [4:0]  m2_arid;    wire [31:0] m2_araddr;  wire [7:0]  m2_arlen;   wire [2:0]  m2_arsize;  wire [1:0]  m2_arburst; wire [2:0]  m2_arprot;  wire m2_arvalid; wire m2_arready;
    wire [4:0]  m2_rid;     wire [31:0] m2_rdata;   wire [1:0]  m2_rresp;   wire        m2_rlast;   wire m2_rvalid; wire m2_rready;

    // Map các Master đơn lẻ vào Bus Mảng
    assign m_axi_awid    = {m2_awid,    m1_awid,    m0_awid};
    assign m_axi_awaddr  = {m2_awaddr,  m1_awaddr,  m0_awaddr};
    assign m_axi_awlen   = {m2_awlen,   m1_awlen,   m0_awlen};
    assign m_axi_awsize  = {m2_awsize,  m1_awsize,  m0_awsize};
    assign m_axi_awburst = {m2_awburst, m1_awburst, m0_awburst};
    assign m_axi_awprot  = {m2_awprot,  m1_awprot,  m0_awprot};
    assign m_axi_awvalid = {m2_awvalid, m1_awvalid, m0_awvalid};
    assign {m2_awready, m1_awready, m0_awready} = m_axi_awready;

    assign m_axi_wdata   = {m2_wdata,  m1_wdata,  m0_wdata};
    assign m_axi_wstrb   = {m2_wstrb,  m1_wstrb,  m0_wstrb};
    assign m_axi_wlast   = {m2_wlast,  m1_wlast,  m0_wlast};
    assign m_axi_wvalid  = {m2_wvalid, m1_wvalid, m0_wvalid};
    assign {m2_wready, m1_wready, m0_wready} = m_axi_wready;

    assign m_axi_bready  = {m2_bready, m1_bready, m0_bready};
    assign {m2_bid, m1_bid, m0_bid}             = m_axi_bid;
    assign {m2_bresp, m1_bresp, m0_bresp}       = m_axi_bresp;
    assign {m2_bvalid, m1_bvalid, m0_bvalid}    = m_axi_bvalid;

    assign m_axi_arid    = {m2_arid,    m1_arid,    m0_arid};
    assign m_axi_araddr  = {m2_araddr,  m1_araddr,  m0_araddr};
    assign m_axi_arlen   = {m2_arlen,   m1_arlen,   m0_arlen};
    assign m_axi_arsize  = {m2_arsize,  m1_arsize,  m0_arsize};
    assign m_axi_arburst = {m2_arburst, m1_arburst, m0_arburst};
    assign m_axi_arprot  = {m2_arprot,  m1_arprot,  m0_arprot};
    assign m_axi_arvalid = {m2_arvalid, m1_arvalid, m0_arvalid};
    assign {m2_arready, m1_arready, m0_arready} = m_axi_arready;

    assign m_axi_rready  = {m2_rready, m1_rready, m0_rready};
    assign {m2_rid, m1_rid, m0_rid}             = m_axi_rid;
    assign {m2_rdata, m1_rdata, m0_rdata}       = m_axi_rdata;
    assign {m2_rresp, m1_rresp, m0_rresp}       = m_axi_rresp;
    assign {m2_rlast, m1_rlast, m0_rlast}       = m_axi_rlast;
    assign {m2_rvalid, m1_rvalid, m0_rvalid}    = m_axi_rvalid;

    // Tín hiệu AXI Slave phẳng
    wire [SLV_AMT*7-1:0]  s_axi_awid; // TRANS_SLV_ID_W = MST_ID_W(2) + TRANS_MST_ID_W(5) = 7
    wire [SLV_AMT*32-1:0] s_axi_awaddr;
    wire [SLV_AMT*8-1:0]  s_axi_awlen;
    wire [SLV_AMT*3-1:0]  s_axi_awsize;
    wire [SLV_AMT*2-1:0]  s_axi_awburst;
    wire [SLV_AMT*3-1:0]  s_axi_awprot;
    wire [SLV_AMT-1:0]    s_axi_awvalid;
    wire [SLV_AMT-1:0]    s_axi_awready;

    wire [SLV_AMT*32-1:0] s_axi_wdata;
    wire [SLV_AMT*4-1:0]  s_axi_wstrb;
    wire [SLV_AMT-1:0]    s_axi_wlast;
    wire [SLV_AMT-1:0]    s_axi_wvalid;
    wire [SLV_AMT-1:0]    s_axi_wready;

    wire [SLV_AMT*7-1:0]  s_axi_bid;
    wire [SLV_AMT*2-1:0]  s_axi_bresp;
    wire [SLV_AMT-1:0]    s_axi_bvalid;
    wire [SLV_AMT-1:0]    s_axi_bready;

    wire [SLV_AMT*7-1:0]  s_axi_arid;
    wire [SLV_AMT*32-1:0] s_axi_araddr;
    wire [SLV_AMT*8-1:0]  s_axi_arlen;
    wire [SLV_AMT*3-1:0]  s_axi_arsize;
    wire [SLV_AMT*2-1:0]  s_axi_arburst;
    wire [SLV_AMT*3-1:0]  s_axi_arprot;
    wire [SLV_AMT-1:0]    s_axi_arvalid;
    wire [SLV_AMT-1:0]    s_axi_arready;

    wire [SLV_AMT*7-1:0]  s_axi_rid;
    wire [SLV_AMT*32-1:0] s_axi_rdata;
    wire [SLV_AMT*2-1:0]  s_axi_rresp;
    wire [SLV_AMT-1:0]    s_axi_rlast;
    wire [SLV_AMT-1:0]    s_axi_rvalid;
    wire [SLV_AMT-1:0]    s_axi_rready;

    // S0: ROM, S1: RAM, S2: SPI Flash, S3: SDRAM, S4: APB Bridge
    wire [6:0] s0_awid; wire [31:0] s0_awaddr; wire [7:0] s0_awlen; wire [2:0] s0_awsize; wire [1:0] s0_awburst; wire [2:0] s0_awprot; wire s0_awvalid; wire s0_awready;
    wire [6:0] s1_awid; wire [31:0] s1_awaddr; wire [7:0] s1_awlen; wire [2:0] s1_awsize; wire [1:0] s1_awburst; wire [2:0] s1_awprot; wire s1_awvalid; wire s1_awready;
    wire [6:0] s2_awid; wire [31:0] s2_awaddr; wire [7:0] s2_awlen; wire [2:0] s2_awsize; wire [1:0] s2_awburst; wire [2:0] s2_awprot; wire s2_awvalid; wire s2_awready;
    wire [6:0] s3_awid; wire [31:0] s3_awaddr; wire [7:0] s3_awlen; wire [2:0] s3_awsize; wire [1:0] s3_awburst; wire [2:0] s3_awprot; wire s3_awvalid; wire s3_awready;
    wire [6:0] s4_awid; wire [31:0] s4_awaddr; wire [7:0] s4_awlen; wire [2:0] s4_awsize; wire [1:0] s4_awburst; wire [2:0] s4_awprot; wire s4_awvalid; wire s4_awready;

    wire [31:0] s0_wdata; wire [3:0] s0_wstrb; wire s0_wlast; wire s0_wvalid; wire s0_wready;
    wire [31:0] s1_wdata; wire [3:0] s1_wstrb; wire s1_wlast; wire s1_wvalid; wire s1_wready;
    wire [31:0] s2_wdata; wire [3:0] s2_wstrb; wire s2_wlast; wire s2_wvalid; wire s2_wready;
    wire [31:0] s3_wdata; wire [3:0] s3_wstrb; wire s3_wlast; wire s3_wvalid; wire s3_wready;
    wire [31:0] s4_wdata; wire [3:0] s4_wstrb; wire s4_wlast; wire s4_wvalid; wire s4_wready;

    wire [6:0] s0_bid; wire [1:0] s0_bresp; wire s0_bvalid; wire s0_bready;
    wire [6:0] s1_bid; wire [1:0] s1_bresp; wire s1_bvalid; wire s1_bready;
    wire [6:0] s2_bid; wire [1:0] s2_bresp; wire s2_bvalid; wire s2_bready;
    wire [6:0] s3_bid; wire [1:0] s3_bresp; wire s3_bvalid; wire s3_bready;
    wire [6:0] s4_bid; wire [1:0] s4_bresp; wire s4_bvalid; wire s4_bready;

    wire [6:0] s0_arid; wire [31:0] s0_araddr; wire [7:0] s0_arlen; wire [2:0] s0_arsize; wire [1:0] s0_arburst; wire [2:0] s0_arprot; wire s0_arvalid; wire s0_arready;
    wire [6:0] s1_arid; wire [31:0] s1_araddr; wire [7:0] s1_arlen; wire [2:0] s1_arsize; wire [1:0] s1_arburst; wire [2:0] s1_arprot; wire s1_arvalid; wire s1_arready;
    wire [6:0] s2_arid; wire [31:0] s2_araddr; wire [7:0] s2_arlen; wire [2:0] s2_arsize; wire [1:0] s2_arburst; wire [2:0] s2_arprot; wire s2_arvalid; wire s2_arready;
    wire [6:0] s3_arid; wire [31:0] s3_araddr; wire [7:0] s3_arlen; wire [2:0] s3_arsize; wire [1:0] s3_arburst; wire [2:0] s3_arprot; wire s3_arvalid; wire s3_arready;
    wire [6:0] s4_arid; wire [31:0] s4_araddr; wire [7:0] s4_arlen; wire [2:0] s4_arsize; wire [1:0] s4_arburst; wire [2:0] s4_arprot; wire s4_arvalid; wire s4_arready;

    wire [6:0] s0_rid; wire [31:0] s0_rdata; wire [1:0] s0_rresp; wire s0_rlast; wire s0_rvalid; wire s0_rready;
    wire [6:0] s1_rid; wire [31:0] s1_rdata; wire [1:0] s1_rresp; wire s1_rlast; wire s1_rvalid; wire s1_rready;
    wire [6:0] s2_rid; wire [31:0] s2_rdata; wire [1:0] s2_rresp; wire s2_rlast; wire s2_rvalid; wire s2_rready;
    wire [6:0] s3_rid; wire [31:0] s3_rdata; wire [1:0] s3_rresp; wire s3_rlast; wire s3_rvalid; wire s3_rready;
    wire [6:0] s4_rid; wire [31:0] s4_rdata; wire [1:0] s4_rresp; wire s4_rlast; wire s4_rvalid; wire s4_rready;

    // Map Slaves ra mảng phẳng
    assign {s4_awready, s3_awready, s2_awready, s1_awready, s0_awready} = s_axi_awready;
    assign s_axi_awid    = {s4_awid,    s3_awid,    s2_awid,    s1_awid,    s0_awid};
    assign s_axi_awaddr  = {s4_awaddr,  s3_awaddr,  s2_awaddr,  s1_awaddr,  s0_awaddr};
    assign s_axi_awlen   = {s4_awlen,   s3_awlen,   s2_awlen,   s1_awlen,   s0_awlen};
    assign s_axi_awsize  = {s4_awsize,  s3_awsize,  s2_awsize,  s1_awsize,  s0_awsize};
    assign s_axi_awburst = {s4_awburst, s3_awburst, s2_awburst, s1_awburst, s0_awburst};
    assign s_axi_awprot  = {s4_awprot,  s3_awprot,  s2_awprot,  s1_awprot,  s0_awprot};
    assign s_axi_awvalid = {s4_awvalid, s3_awvalid, s2_awvalid, s1_awvalid, s0_awvalid};

    assign {s4_wready, s3_wready, s2_wready, s1_wready, s0_wready} = s_axi_wready;
    assign s_axi_wdata   = {s4_wdata,  s3_wdata,  s2_wdata,  s1_wdata,  s0_wdata};
    assign s_axi_wstrb   = {s4_wstrb,  s3_wstrb,  s2_wstrb,  s1_wstrb,  s0_wstrb};
    assign s_axi_wlast   = {s4_wlast,  s3_wlast,  s2_wlast,  s1_wlast,  s0_wlast};
    assign s_axi_wvalid  = {s4_wvalid, s3_wvalid, s2_wvalid, s1_wvalid, s0_wvalid};

    assign s_axi_bready  = {s4_bready, s3_bready, s2_bready, s1_bready, s0_bready};
    assign {s4_bid, s3_bid, s2_bid, s1_bid, s0_bid}             = s_axi_bid;
    assign {s4_bresp, s3_bresp, s2_bresp, s1_bresp, s0_bresp}   = s_axi_bresp;
    assign {s4_bvalid, s3_bvalid, s2_bvalid, s1_bvalid, s0_bvalid} = s_axi_bvalid;

    assign {s4_arready, s3_arready, s2_arready, s1_arready, s0_arready} = s_axi_arready;
    assign s_axi_arid    = {s4_arid,    s3_arid,    s2_arid,    s1_arid,    s0_arid};
    assign s_axi_araddr  = {s4_araddr,  s3_araddr,  s2_araddr,  s1_araddr,  s0_araddr};
    assign s_axi_arlen   = {s4_arlen,   s3_arlen,   s2_arlen,   s1_arlen,   s0_arlen};
    assign s_axi_arsize  = {s4_arsize,  s3_arsize,  s2_arsize,  s1_arsize,  s0_arsize};
    assign s_axi_arburst = {s4_arburst, s3_arburst, s2_arburst, s1_arburst, s0_arburst};
    assign s_axi_arprot  = {s4_arprot,  s3_arprot,  s2_arprot,  s1_arprot,  s0_arprot};
    assign s_axi_arvalid = {s4_arvalid, s3_arvalid, s2_arvalid, s1_arvalid, s0_arvalid};

    assign s_axi_rready  = {s4_rready, s3_rready, s2_rready, s1_rready, s0_rready};
    assign {s4_rid, s3_rid, s2_rid, s1_rid, s0_rid}             = s_axi_rid;
    assign {s4_rdata, s3_rdata, s2_rdata, s1_rdata, s0_rdata}   = s_axi_rdata;
    assign {s4_rresp, s3_rresp, s2_rresp, s1_rresp, s0_rresp}   = s_axi_rresp;
    assign {s4_rlast, s3_rlast, s2_rlast, s1_rlast, s0_rlast}   = s_axi_rlast;
    assign {s4_rvalid, s3_rvalid, s2_rvalid, s1_rvalid, s0_rvalid} = s_axi_rvalid;

    // =========================================================================
    // 4. INSTANTIATE CÁC MASTER MODULES
    // =========================================================================
    // M0: I-Cache
    wire ic_uncache_en = (cpu_ic_addr >= 32'h4000_0000 && cpu_ic_addr <= 32'h4FFF_FFFF);

    icache u_icache (
        .clk             (clk),
        .rst_n           (rst_n),
        .cpu_read_req    (cpu_inst_req),
        .cpu_addr        (cpu_inst_addr),
        .uncache_en      (ic_uncache_en),
        .cpu_read_data   (cpu_inst_data),
        .icache_hit      (cpu_inst_hit),
        .icache_stall    (cpu_inst_stall),
        // M-AXI
        .m_axi_arid      (m0_arid),
        .m_axi_araddr    (m0_araddr),
        .m_axi_arlen     (m0_arlen),
        .m_axi_arsize    (m0_arsize),
        .m_axi_arburst   (m0_arburst),
        .m_axi_arvalid   (m0_arvalid),
        .m_axi_arready   (m0_arready),
        .m_axi_rdata     (m0_rdata),
        .m_axi_rresp     (m0_rresp),
        .m_axi_rlast     (m0_rlast),
        .m_axi_rvalid    (m0_rvalid),
        .m_axi_rready    (m0_rready)
    );
    // Vô hiệu hóa Write cho I-Cache
    assign m0_awvalid = 0; assign m0_wvalid = 0; assign m0_bready = 1;

    // M1: D-Cache
    wire dc_uncache_en = (cpu_dc_addr >= 32'h4000_0000 && cpu_dc_addr <= 32'h4FFF_FFFF);

    dcache u_dcache (
        .clk             (clk),
        .rst_n           (rst_n),
        .cpu_read_req    (cpu_data_rd_req),
        .cpu_write_req   (cpu_data_wr_req),
        .cpu_addr        (cpu_data_addr),
        .cpu_write_data  (cpu_data_wdata),
        .mem_unsigned    (cpu_data_unsigned),
        .mem_size        (cpu_data_size),
        .uncache_en      (dc_uncache_en),
        .cpu_read_data   (cpu_data_rdata),
        .dcache_hit      (cpu_data_hit),
        .dcache_stall    (cpu_data_stall),
        // M-AXI (Read + Write)
        .m_axi_awid      (m1_awid),
        .m_axi_awaddr    (m1_awaddr),
        .m_axi_awlen     (m1_awlen),
        .m_axi_awsize    (m1_awsize),
        .m_axi_awburst   (m1_awburst),
        .m_axi_awvalid   (m1_awvalid),
        .m_axi_awready   (m1_awready),
        .m_axi_wdata     (m1_wdata),
        .m_axi_wstrb     (m1_wstrb),
        .m_axi_wlast     (m1_wlast),
        .m_axi_wvalid    (m1_wvalid),
        .m_axi_wready    (m1_wready),
        .m_axi_bresp     (m1_bresp),
        .m_axi_bvalid    (m1_bvalid),
        .m_axi_bready    (m1_bready),
        .m_axi_arid      (m1_arid),
        .m_axi_araddr    (m1_araddr),
        .m_axi_arlen     (m1_arlen),
        .m_axi_arsize    (m1_arsize),
        .m_axi_arburst   (m1_arburst),
        .m_axi_arvalid   (m1_arvalid),
        .m_axi_arready   (m1_arready),
        .m_axi_rdata     (m1_rdata),
        .m_axi_rresp     (m1_rresp),
        .m_axi_rlast     (m1_rlast),
        .m_axi_rvalid    (m1_rvalid),
        .m_axi_rready    (m1_rready)
    );

    // M2: Debug Module (JTAG + DTM AXI Master)
    wire        dmi_req_valid, dmi_resp_valid, dmi_resp_ready;
    wire [6:0]  dmi_req_addr;
    wire [31:0] dmi_req_data, dmi_resp_data;
    wire [1:0]  dmi_req_op, dmi_resp_op;
    wire        dtm_ack;
    
    rv_jtag_dtm u_jtag_dtm (
        .tck             (tck),
        .trst_n          (trst_n),
        .tms             (tms),
        .tdi             (tdi),
        .tdo             (tdo),
        .dmi_req_valid   (dmi_req_valid),
        .dmi_req_addr    (dmi_req_addr),
        .dmi_req_data    (dmi_req_data),
        .dmi_req_op      (dmi_req_op),
        .dmi_resp_ready  (1'b1),
        .dmi_resp_valid  (dtm_ack), // Đấu qua ack
        .dmi_resp_data   (dmi_resp_data),
        .dmi_resp_op     (dmi_resp_op)
    );

    dtm_axi_master u_dtm_axi (
        .clk_sys         (clk),
        .rst_sys_n       (rst_n),
        .i_req           (dmi_req_valid),
        .i_op            (dmi_req_op),
        .i_size          (2'b10), // 32-bit
        .i_addr          ({23'b0, dmi_req_addr, 2'b0}),
        .i_wdata         (dmi_req_data),
        .o_ack           (dtm_ack),
        .o_resp          (dmi_resp_op),
        .o_rdata         (dmi_resp_data),
        // M-AXI
        .m_axi_awid      (m2_awid),
        .m_axi_awaddr    (m2_awaddr),
        .m_axi_awlen     (m2_awlen),
        .m_axi_awsize    (m2_awsize),
        .m_axi_awburst   (m2_awburst),
        .m_axi_awvalid   (m2_awvalid),
        .m_axi_awready   (m2_awready),
        .m_axi_wdata     (m2_wdata),
        .m_axi_wstrb     (m2_wstrb),
        .m_axi_wlast     (m2_wlast),
        .m_axi_wvalid    (m2_wvalid),
        .m_axi_wready    (m2_wready),
        .m_axi_bresp     (m2_bresp),
        .m_axi_bvalid    (m2_bvalid),
        .m_axi_bready    (m2_bready),
        .m_axi_arid      (m2_arid),
        .m_axi_araddr    (m2_araddr),
        .m_axi_arlen     (m2_arlen),
        .m_axi_arsize    (m2_arsize),
        .m_axi_arburst   (m2_arburst),
        .m_axi_arvalid   (m2_arvalid),
        .m_axi_arready   (m2_arready),
        .m_axi_rdata     (m2_rdata),
        .m_axi_rresp     (m2_rresp),
        .m_axi_rlast     (m2_rlast),
        .m_axi_rvalid    (m2_rvalid),
        .m_axi_rready    (m2_rready)
    );

    // =========================================================================
    // 5. KHỞI TẠO AXI INTERCONNECT
    // =========================================================================
    axi_interconnect #(
        .MST_AMT(MST_AMT), 
        .SLV_AMT(SLV_AMT),
        // Gán địa chỉ Base và Mask cho 5 Slave
        // S0: ROM (0x0000_0000 - 64KB), S1: RAM (0x2000_0000 - 128KB)
        // S2: Flash (0x3000_0000 - 16MB), S3: SDRAM (0x8000_0000 - 64MB)
        // S4: APB Bridge (0x4000_0000 - 1MB)
        .SLV_BASE_ADDR (160'h4000_0000_8000_0000_3000_0000_2000_0000_0000_0000),
        .SLV_ADDR_MASK (160'hFFF0_0000_FC00_0000_FF00_0000_FFFE_0000_FFFF_0000)
    ) u_axi_interconnect (
        .ACLK_i          (clk),
        .ARESETn_i       (rst_n),
        // Masters
        .m_AWID_i        (m_axi_awid),
        .m_AWADDR_i      (m_axi_awaddr),
        .m_AWBURST_i     (m_axi_awburst),
        .m_AWLEN_i       (m_axi_awlen),
        .m_AWSIZE_i      (m_axi_awsize),
        .m_AWVALID_i     (m_axi_awvalid),
        .m_AWREADY_o     (m_axi_awready),
        .m_WDATA_i       (m_axi_wdata),
        .m_WSTRB_i       (m_axi_wstrb),
        .m_WLAST_i       (m_axi_wlast),
        .m_WVALID_i      (m_axi_wvalid),
        .m_WREADY_o      (m_axi_wready),
        .m_BID_o         (m_axi_bid),
        .m_BRESP_o       (m_axi_bresp),
        .m_BVALID_o      (m_axi_bvalid),
        .m_BREADY_i      (m_axi_bready),
        .m_ARID_i        (m_axi_arid),
        .m_ARADDR_i      (m_axi_araddr),
        .m_ARBURST_i     (m_axi_arburst),
        .m_ARLEN_i       (m_axi_arlen),
        .m_ARSIZE_i      (m_axi_arsize),
        .m_ARVALID_i     (m_axi_arvalid),
        .m_ARREADY_o     (m_axi_arready),
        .m_RID_o         (m_axi_rid),
        .m_RDATA_o       (m_axi_rdata),
        .m_RRESP_o       (m_axi_rresp),
        .m_RLAST_o       (m_axi_rlast),
        .m_RVALID_o      (m_axi_rvalid),
        .m_RREADY_i      (m_axi_rready),
        // Slaves
        .s_AWID_o        (s_axi_awid),
        .s_AWADDR_o      (s_axi_awaddr),
        .s_AWBURST_o     (s_axi_awburst),
        .s_AWLEN_o       (s_axi_awlen),
        .s_AWSIZE_o      (s_axi_awsize),
        .s_AWVALID_o     (s_axi_awvalid),
        .s_AWREADY_i     (s_axi_awready),
        .s_WDATA_o       (s_axi_wdata),
        .s_WSTRB_o       (s_axi_wstrb),
        .s_WLAST_o       (s_axi_wlast),
        .s_WVALID_o      (s_axi_wvalid),
        .s_WREADY_i      (s_axi_wready),
        .s_BID_i         (s_axi_bid),
        .s_BRESP_i       (s_axi_bresp),
        .s_BVALID_i      (s_axi_bvalid),
        .s_BREADY_o      (s_axi_bready),
        .s_ARID_o        (s_axi_arid),
        .s_ARADDR_o      (s_axi_araddr),
        .s_ARBURST_o     (s_axi_arburst),
        .s_ARLEN_o       (s_axi_arlen),
        .s_ARSIZE_o      (s_axi_arsize),
        .s_ARVALID_o     (s_axi_arvalid),
        .s_ARREADY_i     (s_axi_arready),
        .s_RID_i         (s_axi_rid),
        .s_RDATA_i       (s_axi_rdata),
        .s_RRESP_i       (s_axi_rresp),
        .s_RLAST_i       (s_axi_rlast),
        .s_RVALID_i      (s_axi_rvalid),
        .s_RREADY_o      (s_axi_rready)
    );

    // =========================================================================
    // 6. KHỞI TẠO CÁC AXI SLAVES
    // =========================================================================
    // S0: ROM
    axi_rom u_axi_rom (
        .clk(clk), .rst_n(rst_n),
        .s_axi_arid(s0_arid), .s_axi_araddr(s0_araddr), .s_axi_arlen(s0_arlen), .s_axi_arsize(s0_arsize), .s_axi_arburst(s0_arburst), .s_axi_arvalid(s0_arvalid), .s_axi_arready(s0_arready),
        .s_axi_rid(s0_rid), .s_axi_rdata(s0_rdata), .s_axi_rresp(s0_rresp), .s_axi_rlast(s0_rlast), .s_axi_rvalid(s0_rvalid), .s_axi_rready(s0_rready)
    );
    assign s0_awready = 0; assign s0_wready = 0; assign s0_bvalid = 0; // ROM read-only

    // S1: RAM
    axi_ram u_axi_ram (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awid(s1_awid), .s_axi_awaddr(s1_awaddr), .s_axi_awlen(s1_awlen), .s_axi_awsize(s1_awsize), .s_axi_awburst(s1_awburst), .s_axi_awvalid(s1_awvalid), .s_axi_awready(s1_awready),
        .s_axi_wdata(s1_wdata), .s_axi_wstrb(s1_wstrb), .s_axi_wlast(s1_wlast), .s_axi_wvalid(s1_wvalid), .s_axi_wready(s1_wready),
        .s_axi_bid(s1_bid), .s_axi_bresp(s1_bresp), .s_axi_bvalid(s1_bvalid), .s_axi_bready(s1_bready),
        .s_axi_arid(s1_arid), .s_axi_araddr(s1_araddr), .s_axi_arlen(s1_arlen), .s_axi_arsize(s1_arsize), .s_axi_arburst(s1_arburst), .s_axi_arvalid(s1_arvalid), .s_axi_arready(s1_arready),
        .s_axi_rid(s1_rid), .s_axi_rdata(s1_rdata), .s_axi_rresp(s1_rresp), .s_axi_rlast(s1_rlast), .s_axi_rvalid(s1_rvalid), .s_axi_rready(s1_rready)
    );

    // S2: SPI Flash
    axi_spi_flash u_axi_flash (
        .clk(clk), .rst_n(rst_n),
        .flash_sck(flash_sck), .flash_cs(flash_cs), .flash_mosi(flash_mosi), .flash_miso(flash_miso),
        .s_axi_arid(s2_arid), .s_axi_araddr(s2_araddr), .s_axi_arlen(s2_arlen), .s_axi_arsize(s2_arsize), .s_axi_arburst(s2_arburst), .s_axi_arvalid(s2_arvalid), .s_axi_arready(s2_arready),
        .s_axi_rid(s2_rid), .s_axi_rdata(s2_rdata), .s_axi_rresp(s2_rresp), .s_axi_rlast(s2_rlast), .s_axi_rvalid(s2_rvalid), .s_axi_rready(s2_rready)
    );
    assign s2_awready = 0; assign s2_wready = 0; assign s2_bvalid = 0;

    // S3: SDRAM Controller
    wire [15:0] sdram_dq_i;
    wire [15:0] sdram_dq_o;
    wire        sdram_dq_oe;

    assign sdram_dq   = sdram_dq_oe ? sdram_dq_o : 16'bz;
    assign sdram_dq_i = sdram_dq;

    axi_sdram_controller u_axi_sdram (
        .clk(clk), .rst_n(rst_n),
        .sdram_clk(sdram_clk), .sdram_cke(sdram_cke), .sdram_cs_n(sdram_cs_n), .sdram_ras_n(sdram_ras_n),
        .sdram_cas_n(sdram_cas_n), .sdram_we_n(sdram_we_n), .sdram_ba(sdram_ba), .sdram_addr(sdram_addr),
        .sdram_dqm(sdram_dqm), .sdram_dq_i(sdram_dq_i), .sdram_dq_o(sdram_dq_o), .sdram_dq_oe(sdram_dq_oe),
        .s_axi_awid(s3_awid), .s_axi_awaddr(s3_awaddr), .s_axi_awlen(s3_awlen), .s_axi_awsize(s3_awsize), .s_axi_awburst(s3_awburst), .s_axi_awvalid(s3_awvalid), .s_axi_awready(s3_awready),
        .s_axi_wdata(s3_wdata), .s_axi_wstrb(s3_wstrb), .s_axi_wlast(s3_wlast), .s_axi_wvalid(s3_wvalid), .s_axi_wready(s3_wready),
        .s_axi_bid(s3_bid), .s_axi_bresp(s3_bresp), .s_axi_bvalid(s3_bvalid), .s_axi_bready(s3_bready),
        .s_axi_arid(s3_arid), .s_axi_araddr(s3_araddr), .s_axi_arlen(s3_arlen), .s_axi_arsize(s3_arsize), .s_axi_arburst(s3_arburst), .s_axi_arvalid(s3_arvalid), .s_axi_arready(s3_arready),
        .s_axi_rid(s3_rid), .s_axi_rdata(s3_rdata), .s_axi_rresp(s3_rresp), .s_axi_rlast(s3_rlast), .s_axi_rvalid(s3_rvalid), .s_axi_rready(s3_rready)
    );

    // =========================================================================
    // 7. AXI TO APB BRIDGE VÀ APB INTERCONNECT
    // =========================================================================
    wire [31:0] apb_paddr, apb_pwdata, apb_prdata;
    wire [3:0]  apb_pstrb;
    wire [2:0]  apb_pprot;
    wire        apb_psel, apb_penable, apb_pwrite, apb_pready, apb_pslverr;

    axi_to_apb_bridge u_axi_to_apb (
        .clk(clk), .rst_n(rst_n),
        .s_axi_awid(s4_awid), .s_axi_awaddr(s4_awaddr), .s_axi_awlen(s4_awlen), .s_axi_awsize(s4_awsize), .s_axi_awburst(s4_awburst), .s_axi_awprot(s4_awprot), .s_axi_awvalid(s4_awvalid), .s_axi_awready(s4_awready),
        .s_axi_wdata(s4_wdata), .s_axi_wstrb(s4_wstrb), .s_axi_wlast(s4_wlast), .s_axi_wvalid(s4_wvalid), .s_axi_wready(s4_wready),
        .s_axi_bid(s4_bid), .s_axi_bresp(s4_bresp), .s_axi_bvalid(s4_bvalid), .s_axi_bready(s4_bready),
        .s_axi_arid(s4_arid), .s_axi_araddr(s4_araddr), .s_axi_arlen(s4_arlen), .s_axi_arsize(s4_arsize), .s_axi_arburst(s4_arburst), .s_axi_arprot(s4_arprot), .s_axi_arvalid(s4_arvalid), .s_axi_arready(s4_arready),
        .s_axi_rid(s4_rid), .s_axi_rdata(s4_rdata), .s_axi_rresp(s4_rresp), .s_axi_rlast(s4_rlast), .s_axi_rvalid(s4_rvalid), .s_axi_rready(s4_rready),
        // APB Master Output
        .m_apb_paddr(apb_paddr), .m_apb_psel(apb_psel), .m_apb_penable(apb_penable), .m_apb_pwrite(apb_pwrite), .m_apb_pwdata(apb_pwdata), .m_apb_pstrb(apb_pstrb), .m_apb_pprot(apb_pprot),
        .m_apb_pready(apb_pready), .m_apb_prdata(apb_prdata), .m_apb_pslverr(apb_pslverr)
    );

    // Wires cho 8 APB Slaves
    wire [31:0] paddr_0, paddr_1, paddr_2, paddr_3, paddr_4, paddr_5, paddr_6, paddr_7;
    wire [31:0] pwdata_0, pwdata_1, pwdata_2, pwdata_3, pwdata_4, pwdata_5, pwdata_6, pwdata_7;
    wire [31:0] prdata_0, prdata_1, prdata_2, prdata_3, prdata_4, prdata_5, prdata_6, prdata_7;
    wire psel_0, psel_1, psel_2, psel_3, psel_4, psel_5, psel_6, psel_7;
    wire penable_0, penable_1, penable_2, penable_3, penable_4, penable_5, penable_6, penable_7;
    wire pwrite_0, pwrite_1, pwrite_2, pwrite_3, pwrite_4, pwrite_5, pwrite_6, pwrite_7;
    wire pready_0, pready_1, pready_2, pready_3, pready_4, pready_5, pready_6, pready_7;

    apb_interconnect u_apb_interconnect (
        .clk(clk), .rst_n(rst_n),
        .m_paddr(apb_paddr), .m_psel(apb_psel), .m_penable(apb_penable), .m_pwrite(apb_pwrite), .m_pwdata(apb_pwdata), .m_pstrb(apb_pstrb), .m_pprot(apb_pprot),
        .m_pready(apb_pready), .m_prdata(apb_prdata), .m_pslverr(apb_pslverr),
        // Cổng nối Slave (Rút gọn phần khai báo do tính lặp lại, ánh xạ trực tiếp)
        .s0_paddr(paddr_0), .s0_psel(psel_0), .s0_penable(penable_0), .s0_pwrite(pwrite_0), .s0_pwdata(pwdata_0), .s0_pready(pready_0), .s0_prdata(prdata_0), .s0_pslverr(1'b0),
        .s1_paddr(paddr_1), .s1_psel(psel_1), .s1_penable(penable_1), .s1_pwrite(pwrite_1), .s1_pwdata(pwdata_1), .s1_pready(pready_1), .s1_prdata(prdata_1), .s1_pslverr(1'b0),
        .s2_paddr(paddr_2), .s2_psel(psel_2), .s2_penable(penable_2), .s2_pwrite(pwrite_2), .s2_pwdata(pwdata_2), .s2_pready(pready_2), .s2_prdata(prdata_2), .s2_pslverr(1'b0),
        .s3_paddr(paddr_3), .s3_psel(psel_3), .s3_penable(penable_3), .s3_pwrite(pwrite_3), .s3_pwdata(pwdata_3), .s3_pready(pready_3), .s3_prdata(prdata_3), .s3_pslverr(1'b0),
        .s4_paddr(paddr_4), .s4_psel(psel_4), .s4_penable(penable_4), .s4_pwrite(pwrite_4), .s4_pwdata(pwdata_4), .s4_pready(pready_4), .s4_prdata(prdata_4), .s4_pslverr(1'b0),
        .s5_paddr(paddr_5), .s5_psel(psel_5), .s5_penable(penable_5), .s5_pwrite(pwrite_5), .s5_pwdata(pwdata_5), .s5_pready(pready_5), .s5_prdata(prdata_5), .s5_pslverr(1'b0),
        .s6_paddr(paddr_6), .s6_psel(psel_6), .s6_penable(penable_6), .s6_pwrite(pwrite_6), .s6_pwdata(pwdata_6), .s6_pready(pready_6), .s6_prdata(prdata_6), .s6_pslverr(1'b0),
        .s7_paddr(paddr_7), .s7_psel(psel_7), .s7_penable(penable_7), .s7_pwrite(pwrite_7), .s7_pwdata(pwdata_7), .s7_pready(pready_7), .s7_prdata(prdata_7), .s7_pslverr(1'b0)
    );

    // =========================================================================
    // 8. KHỞI TẠO CÁC NGOẠI VI APB
    // =========================================================================
    // S0: UART
    apb_uart u_apb_uart (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr_0), .psel(psel_0), .penable(penable_0), .pwrite(pwrite_0), .pwdata(pwdata_0), .pready(pready_0), .prdata(prdata_0),
        .uart_rx(uart_rx), .uart_tx(uart_tx)
    );

    // S1: GPIO
    apb_gpio u_apb_gpio (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr_1), .psel(psel_1), .penable(penable_1), .pwrite(pwrite_1), .pwdata(pwdata_1), .pready(pready_1), .prdata(prdata_1),
        .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_oe(gpio_oe)
    );

    // S2: Timer
    apb_timer u_apb_timer (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr_2), .psel(psel_2), .penable(penable_2), .pwrite(pwrite_2), .pwdata(pwdata_2), .pready(pready_2), .prdata(prdata_2)
    );

    // S3: SPI
    apb_spi u_apb_spi (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr_3), .psel(psel_3), .penable(penable_3), .pwrite(pwrite_3), .pwdata(pwdata_3), .pready(pready_3), .prdata(prdata_3),
        .spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_miso(spi_miso), .spi_ss(spi_ss)
    );

    // S4: I2C
    wire i2c_scl_i, i2c_scl_o, i2c_scl_oe;
    wire i2c_sda_i, i2c_sda_o, i2c_sda_oe;

    assign i2c_scl   = i2c_scl_oe ? i2c_scl_o : 1'bz;
    assign i2c_scl_i = i2c_scl;

    assign i2c_sda   = i2c_sda_oe ? i2c_sda_o : 1'bz;
    assign i2c_sda_i = i2c_sda;

    apb_i2c u_apb_i2c (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr_4), .psel(psel_4), .penable(penable_4), .pwrite(pwrite_4), .pwdata(pwdata_4), .pready(pready_4), .prdata(prdata_4),
        .i2c_scl_i(i2c_scl_i), .i2c_scl_o(i2c_scl_o), .i2c_scl_oe(i2c_scl_oe),
        .i2c_sda_i(i2c_sda_i), .i2c_sda_o(i2c_sda_o), .i2c_sda_oe(i2c_sda_oe)
    );

    // S5: Watchdog
    apb_watchdog u_apb_watchdog (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr_5), .psel(psel_5), .penable(penable_5), .pwrite(pwrite_5), .pwdata(pwdata_5), .pready(pready_5), .prdata(prdata_5)
    );

    // S6: CORDIC
    apb_cordic u_apb_cordic (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr_6), .psel(psel_6), .penable(penable_6), .pwrite(pwrite_6), .pwdata(pwdata_6), .pready(pready_6), .prdata(prdata_6)
    );

    // S7: Syscon
    apb_syscon u_apb_syscon (
        .clk(clk), .rst_n(rst_n),
        .paddr(paddr_7), .psel(psel_7), .penable(penable_7), .pwrite(pwrite_7), .pwdata(pwdata_7), .pready(pready_7), .prdata(prdata_7)
    );

endmodule