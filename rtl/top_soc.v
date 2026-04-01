`timescale 1ns / 1ps

module top_soc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rtc_clk,

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

    // PWM
    output wire        pwm_out,

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
    // TÍN HIỆU NGẮT & DEBUG MODULE
    // =========================================================================
    // Ngắt từ CLINT/PLIC
    wire [0:0] cpu_msip;
    wire [0:0] cpu_mtip;
    wire       cpu_meip;

    wire       uart_irq, gpio_irq, spi_irq, i2c_irq, wdt_irq;
    wire [3:0] dma_irq;

    // DMA Requests (Từ APB Slaves)
    wire       uart_dma_tx, uart_dma_rx;
    wire       spi_dma_tx,  spi_dma_rx;
    wire       i2c_dma_tx,  i2c_dma_rx;

    // Quản lý Reset/Năng lượng (Syscon & Watchdog)
    wire [31:0] syscon_reset_vector;
    wire        wfi_sleep_state;
    wire        wdt_rst;

    // Tổ hợp DMA Requests (Bit 0 bỏ trống theo chuẩn)
    wire [31:1] periph_dma_req = {
        25'd0,
        i2c_dma_rx, i2c_dma_tx, // [6:5]
        spi_dma_rx, spi_dma_tx, // [4:3]
        uart_dma_rx, uart_dma_tx // [2:1]
    };
    wire [31:1] periph_dma_clr;

    // Tổ hợp Nguồn Ngắt PLIC (Bit 0 luôn = 0)
    wire [31:0] plic_irq_src = {
        22'd0,         // [31:10]: Trống
        dma_irq,       // [9:6]: Ngắt của 4 kênh DMA
        wdt_irq,       // [5]: Watchdog Cảnh báo
        i2c_irq,       // [4]: I2C
        spi_irq,       // [3]: SPI
        gpio_irq,      // [2]: GPIO (Cảm biến/Nút nhấn)
        uart_irq,      // [1]: UART
        1'b0           // [0]: LUÔN BẰNG 0
    };

    // Dây kết nối giữa Debug Module (DM) và CPU
    wire        dbg_halt_req;
    wire        dbg_resume_req;
    wire        dbg_halted;
    wire [15:0] dbg_reg_read_addr;
    wire [31:0] dbg_reg_read_data;
    wire        dbg_reg_write_en;
    wire [15:0] dbg_reg_write_addr;
    wire [31:0] dbg_reg_write_data;
    wire        ndmreset_req;

    // Dây kết nối giữa DM (System Bus Access) và AXI Master wrapper
    wire        sba_req;
    wire        sba_ack;
    wire [1:0]  sba_op;
    wire [1:0]  sba_size;
    wire [1:0]  sba_resp;
    wire [31:0] sba_addr;
    wire [31:0] sba_wdata;
    wire [31:0] sba_rdata;

    // =========================================================================
    // CLOCK GATING NETWORK
    // =========================================================================
    wire clk_cpu, clk_dbg, clk_pwm, clk_uart, clk_spi, clk_i2c, clk_gpio, clk_cordic;

    clock_gate cg_cpu   (.clk_in(clk), .en(clk_en_cpu),  .test_en(1'b0), .clk_out(clk_cpu));
    clock_gate cg_dbg   (.clk_in(clk), .en(clk_en_dbg),  .test_en(1'b0), .clk_out(clk_dbg));
    clock_gate cg_pwm   (.clk_in(clk), .en(clk_en_pwm),  .test_en(1'b0), .clk_out(clk_pwm));
    clock_gate cg_uart  (.clk_in(clk), .en(clk_en_uart), .test_en(1'b0), .clk_out(clk_uart));
    clock_gate cg_spi   (.clk_in(clk), .en(clk_en_spi),  .test_en(1'b0), .clk_out(clk_spi));
    clock_gate cg_i2c   (.clk_in(clk), .en(clk_en_i2c),  .test_en(1'b0), .clk_out(clk_i2c));
    clock_gate cg_gpio  (.clk_in(clk), .en(clk_en_gpio), .test_en(1'b0), .clk_out(clk_gpio));
    clock_gate cg_cordic(.clk_in(clk), .en(clk_en_acc),  .test_en(1'b0), .clk_out(clk_cordic));

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
    wire reset_cpu_n = reset_sys_n = rst_n & ~ndmreset_req & ~wdt_rst; // Hỗ trợ Non-Debug Module Reset từ OpenOCD

    riscv_pipeline u_core (
        .clk             (clk_cpu),
        .reset_n         (reset_sys_n),
        .riscv_start     (1'b1),               // Tự động chạy khi có clock
        .meip_i          (cpu_meip),           // External Interrupt (từ PLIC)
        .msip_i          (cpu_msip[0]),        // Software Interrupt (từ CLINT)
        .mtip_i          (cpu_mtip[0]),        // Timer Interrupt (từ CLINT)
        .reset_vector_in (syscon_reset_vector),// Trỏ về ROM
        .riscv_done      (),                   // Bỏ trống nếu không dùng

        // ICache interface
        .icache_read_req (cpu_inst_req),
        .icache_addr     (cpu_inst_addr),
        .icache_read_data(cpu_inst_data),
        .icache_hit      (cpu_inst_hit),
        .icache_stall    (cpu_inst_stall),
        
        // DCache interface
        .dcache_read_req  (cpu_data_rd_req),
        .dcache_write_req (cpu_data_wr_req),
        .dcache_addr      (cpu_data_addr),
        .dcache_write_data(cpu_data_wdata),
        .dcache_read_data (cpu_data_rdata),
        .dcache_hit       (cpu_data_hit),
        .dcache_stall     (cpu_data_stall),
        .mem_size_top     (cpu_data_size),
        .mem_unsigned_top (cpu_data_unsigned),
        .wfi_sleep_out    (wfi_sleep_state),

        // Debug Module Interface
        .dbg_halt_req       (dbg_halt_req),
        .dbg_resume_req     (dbg_resume_req),
        .dbg_halted         (dbg_halted),
        .dbg_reg_read_addr  (dbg_reg_read_addr),
        .dbg_reg_read_data  (dbg_reg_read_data),
        .dbg_reg_write_en   (dbg_reg_write_en),
        .dbg_reg_write_addr (dbg_reg_write_addr),
        .dbg_reg_write_data (dbg_reg_write_data)
    );

    // =========================================================================
    // 3. KHAI BÁO CÁC KÊNH AXI (4 MASTERS, 6 SLAVES)
    // =========================================================================
    localparam MST_AMT = 4;
    localparam SLV_AMT = 6;

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

    // M3: DMA
    wire [4:0]  m3_awid;    wire [31:0] m3_awaddr;  wire [7:0]  m3_awlen;   wire [2:0]  m3_awsize;  wire [1:0]  m3_awburst; wire [2:0]  m3_awprot;  wire m3_awvalid; wire m3_awready;
    wire [31:0] m3_wdata;   wire [3:0]  m3_wstrb;   wire        m3_wlast;   wire        m3_wvalid;  wire m3_wready;
    wire [4:0]  m3_bid;     wire [1:0]  m3_bresp;   wire        m3_bvalid;  wire        m3_bready;
    wire [4:0]  m3_arid;    wire [31:0] m3_araddr;  wire [7:0]  m3_arlen;   wire [2:0]  m3_arsize;  wire [1:0]  m3_arburst; wire [2:0]  m3_arprot;  wire m3_arvalid; wire m3_arready;
    wire [4:0]  m3_rid;     wire [31:0] m3_rdata;   wire [1:0]  m3_rresp;   wire        m3_rlast;   wire m3_rvalid; wire m3_rready;

    // Map các Master đơn lẻ vào Bus Mảng
    assign m_axi_awid    = {m3_awid, m2_awid,    m1_awid,    m0_awid};
    assign m_axi_awaddr  = {m3_awaddr, m2_awaddr,  m1_awaddr,  m0_awaddr};
    assign m_axi_awlen   = {m3_awlen, m2_awlen,   m1_awlen,   m0_awlen};
    assign m_axi_awsize  = {m3_awsize, m2_awsize,  m1_awsize,  m0_awsize};
    assign m_axi_awburst = {m3_awburst, m2_awburst, m1_awburst, m0_awburst};
    assign m_axi_awprot  = {m3_awprot, m2_awprot,  m1_awprot,  m0_awprot};
    assign m_axi_awvalid = {m3_awvalid, m2_awvalid, m1_awvalid, m0_awvalid};
    assign {m3_awready, m2_awready, m1_awready, m0_awready} = m_axi_awready;

    assign m_axi_wdata   = {m3_wdata, m2_wdata,  m1_wdata,  m0_wdata};
    assign m_axi_wstrb   = {m3_wstrb, m2_wstrb,  m1_wstrb,  m0_wstrb};
    assign m_axi_wlast   = {m3_wlast, m2_wlast,  m1_wlast,  m0_wlast};
    assign m_axi_wvalid  = {m3_wvalid, m2_wvalid, m1_wvalid, m0_wvalid};
    assign {m3_wready, m2_wready, m1_wready, m0_wready} = m_axi_wready;

    assign m_axi_bready  = {m3_bready, m2_bready, m1_bready, m0_bready};
    assign {m3_bid, m2_bid, m1_bid, m0_bid}             = m_axi_bid;
    assign {m3_bresp, m2_bresp, m1_bresp, m0_bresp}       = m_axi_bresp;
    assign {m3_bvalid, m2_bvalid, m1_bvalid, m0_bvalid}    = m_axi_bvalid;

    assign m_axi_arid    = {m3_arid, m2_arid,    m1_arid,    m0_arid};
    assign m_axi_araddr  = {m3_araddr, m2_araddr,  m1_araddr,  m0_araddr};
    assign m_axi_arlen   = {m3_arlen, m2_arlen,   m1_arlen,   m0_arlen};
    assign m_axi_arsize  = {m3_arsize, m2_arsize,  m1_arsize,  m0_arsize};
    assign m_axi_arburst = {m3_arburst, m2_arburst, m1_arburst, m0_arburst};
    assign m_axi_arprot  = {m3_arprot, m2_arprot,  m1_arprot,  m0_arprot};
    assign m_axi_arvalid = {m3_arvalid, m2_arvalid, m1_arvalid, m0_arvalid};
    assign {m3_arready, m2_arready, m1_arready, m0_arready} = m_axi_arready;

    assign m_axi_rready  = {m3_rready, m2_rready, m1_rready, m0_rready};
    assign {m3_rid, m2_rid, m1_rid, m0_rid}             = m_axi_rid;
    assign {m3_rdata, m2_rdata, m1_rdata, m0_rdata}       = m_axi_rdata;
    assign {m3_rresp, m2_rresp, m1_rresp, m0_rresp}       = m_axi_rresp;
    assign {m3_rlast, m2_rlast, m1_rlast, m0_rlast}       = m_axi_rlast;
    assign {m3_rvalid, m2_rvalid, m1_rvalid, m0_rvalid}    = m_axi_rvalid;

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

    // S0: ROM, S1: RAM, S2: SPI Flash, S3: SDRAM, S4: APB Bridge, S5: CLINT
    wire [6:0] s0_awid; wire [31:0] s0_awaddr; wire [7:0] s0_awlen; wire [2:0] s0_awsize; wire [1:0] s0_awburst; wire [2:0] s0_awprot; wire s0_awvalid; wire s0_awready;
    wire [6:0] s1_awid; wire [31:0] s1_awaddr; wire [7:0] s1_awlen; wire [2:0] s1_awsize; wire [1:0] s1_awburst; wire [2:0] s1_awprot; wire s1_awvalid; wire s1_awready;
    wire [6:0] s2_awid; wire [31:0] s2_awaddr; wire [7:0] s2_awlen; wire [2:0] s2_awsize; wire [1:0] s2_awburst; wire [2:0] s2_awprot; wire s2_awvalid; wire s2_awready;
    wire [6:0] s3_awid; wire [31:0] s3_awaddr; wire [7:0] s3_awlen; wire [2:0] s3_awsize; wire [1:0] s3_awburst; wire [2:0] s3_awprot; wire s3_awvalid; wire s3_awready;
    wire [6:0] s4_awid; wire [31:0] s4_awaddr; wire [7:0] s4_awlen; wire [2:0] s4_awsize; wire [1:0] s4_awburst; wire [2:0] s4_awprot; wire s4_awvalid; wire s4_awready;
    wire [6:0] s5_awid; wire [31:0] s5_awaddr; wire [7:0] s5_awlen; wire [2:0] s5_awsize; wire [1:0] s5_awburst; wire [2:0] s5_awprot; wire s5_awvalid; wire s5_awready;
    
    wire [31:0] s0_wdata; wire [3:0] s0_wstrb; wire s0_wlast; wire s0_wvalid; wire s0_wready;
    wire [31:0] s1_wdata; wire [3:0] s1_wstrb; wire s1_wlast; wire s1_wvalid; wire s1_wready;
    wire [31:0] s2_wdata; wire [3:0] s2_wstrb; wire s2_wlast; wire s2_wvalid; wire s2_wready;
    wire [31:0] s3_wdata; wire [3:0] s3_wstrb; wire s3_wlast; wire s3_wvalid; wire s3_wready;
    wire [31:0] s4_wdata; wire [3:0] s4_wstrb; wire s4_wlast; wire s4_wvalid; wire s4_wready;
    wire [31:0] s5_wdata; wire [3:0] s5_wstrb; wire s5_wlast; wire s5_wvalid; wire s5_wready;

    wire [6:0] s0_bid; wire [1:0] s0_bresp; wire s0_bvalid; wire s0_bready;
    wire [6:0] s1_bid; wire [1:0] s1_bresp; wire s1_bvalid; wire s1_bready;
    wire [6:0] s2_bid; wire [1:0] s2_bresp; wire s2_bvalid; wire s2_bready;
    wire [6:0] s3_bid; wire [1:0] s3_bresp; wire s3_bvalid; wire s3_bready;
    wire [6:0] s4_bid; wire [1:0] s4_bresp; wire s4_bvalid; wire s4_bready;
    wire [6:0] s5_bid; wire [1:0] s5_bresp; wire s5_bvalid; wire s5_bready;

    wire [6:0] s0_arid; wire [31:0] s0_araddr; wire [7:0] s0_arlen; wire [2:0] s0_arsize; wire [1:0] s0_arburst; wire [2:0] s0_arprot; wire s0_arvalid; wire s0_arready;
    wire [6:0] s1_arid; wire [31:0] s1_araddr; wire [7:0] s1_arlen; wire [2:0] s1_arsize; wire [1:0] s1_arburst; wire [2:0] s1_arprot; wire s1_arvalid; wire s1_arready;
    wire [6:0] s2_arid; wire [31:0] s2_araddr; wire [7:0] s2_arlen; wire [2:0] s2_arsize; wire [1:0] s2_arburst; wire [2:0] s2_arprot; wire s2_arvalid; wire s2_arready;
    wire [6:0] s3_arid; wire [31:0] s3_araddr; wire [7:0] s3_arlen; wire [2:0] s3_arsize; wire [1:0] s3_arburst; wire [2:0] s3_arprot; wire s3_arvalid; wire s3_arready;
    wire [6:0] s4_arid; wire [31:0] s4_araddr; wire [7:0] s4_arlen; wire [2:0] s4_arsize; wire [1:0] s4_arburst; wire [2:0] s4_arprot; wire s4_arvalid; wire s4_arready;
    wire [6:0] s5_arid; wire [31:0] s5_araddr; wire [7:0] s5_arlen; wire [2:0] s5_arsize; wire [1:0] s5_arburst; wire [2:0] s5_arprot; wire s5_arvalid; wire s5_arready;

    wire [6:0] s0_rid; wire [31:0] s0_rdata; wire [1:0] s0_rresp; wire s0_rlast; wire s0_rvalid; wire s0_rready;
    wire [6:0] s1_rid; wire [31:0] s1_rdata; wire [1:0] s1_rresp; wire s1_rlast; wire s1_rvalid; wire s1_rready;
    wire [6:0] s2_rid; wire [31:0] s2_rdata; wire [1:0] s2_rresp; wire s2_rlast; wire s2_rvalid; wire s2_rready;
    wire [6:0] s3_rid; wire [31:0] s3_rdata; wire [1:0] s3_rresp; wire s3_rlast; wire s3_rvalid; wire s3_rready;
    wire [6:0] s4_rid; wire [31:0] s4_rdata; wire [1:0] s4_rresp; wire s4_rlast; wire s4_rvalid; wire s4_rready;
    wire [6:0] s5_rid; wire [31:0] s5_rdata; wire [1:0] s5_rresp; wire s5_rlast; wire s5_rvalid; wire s5_rready;

    // Map Slaves ra mảng phẳng
    assign {s5_awready, s4_awready, s3_awready, s2_awready, s1_awready, s0_awready} = s_axi_awready;
    assign s_axi_awid    = {s5_awid, s4_awid,    s3_awid,    s2_awid,    s1_awid,    s0_awid};
    assign s_axi_awaddr  = {s5_awaddr, s4_awaddr,  s3_awaddr,  s2_awaddr,  s1_awaddr,  s0_awaddr};
    assign s_axi_awlen   = {s5_awlen, s4_awlen,   s3_awlen,   s2_awlen,   s1_awlen,   s0_awlen};
    assign s_axi_awsize  = {s5_awsize, s4_awsize,  s3_awsize,  s2_awsize,  s1_awsize,  s0_awsize};
    assign s_axi_awburst = {s5_awburst, s4_awburst, s3_awburst, s2_awburst, s1_awburst, s0_awburst};
    assign s_axi_awprot  = {s5_awprot, s4_awprot,  s3_awprot,  s2_awprot,  s1_awprot,  s0_awprot};
    assign s_axi_awvalid = {s5_awvalid, s4_awvalid, s3_awvalid, s2_awvalid, s1_awvalid, s0_awvalid};

    assign {s5_wready, s4_wready, s3_wready, s2_wready, s1_wready, s0_wready} = s_axi_wready;
    assign s_axi_wdata   = {s5_wdata, s4_wdata,  s3_wdata,  s2_wdata,  s1_wdata,  s0_wdata};
    assign s_axi_wstrb   = {s5_wstrb, s4_wstrb,  s3_wstrb,  s2_wstrb,  s1_wstrb,  s0_wstrb};
    assign s_axi_wlast   = {s5_wlast, s4_wlast,  s3_wlast,  s2_wlast,  s1_wlast,  s0_wlast};
    assign s_axi_wvalid  = {s5_wvalid, s4_wvalid, s3_wvalid, s2_wvalid, s1_wvalid, s0_wvalid};

    assign s_axi_bready  = {s5_bready, s4_bready, s3_bready, s2_bready, s1_bready, s0_bready};
    assign {s5_bid, s4_bid, s3_bid, s2_bid, s1_bid, s0_bid}             = s_axi_bid;
    assign {s5_bresp, s4_bresp, s3_bresp, s2_bresp, s1_bresp, s0_bresp}   = s_axi_bresp;
    assign {s5_bvalid, s4_bvalid, s3_bvalid, s2_bvalid, s1_bvalid, s0_bvalid} = s_axi_bvalid;

    assign {s5_arready, s4_arready, s3_arready, s2_arready, s1_arready, s0_arready} = s_axi_arready;
    assign s_axi_arid    = {s5_arid, s4_arid,    s3_arid,    s2_arid,    s1_arid,    s0_arid};
    assign s_axi_araddr  = {s5_araddr, s4_araddr,  s3_araddr,  s2_araddr,  s1_araddr,  s0_araddr};
    assign s_axi_arlen   = {s5_arlen, s4_arlen,   s3_arlen,   s2_arlen,   s1_arlen,   s0_arlen};
    assign s_axi_arsize  = {s5_arsize, s4_arsize,  s3_arsize,  s2_arsize,  s1_arsize,  s0_arsize};
    assign s_axi_arburst = {s5_arburst, s4_arburst, s3_arburst, s2_arburst, s1_arburst, s0_arburst};
    assign s_axi_arprot  = {s5_arprot, s4_arprot,  s3_arprot,  s2_arprot,  s1_arprot,  s0_arprot};
    assign s_axi_arvalid = {s5_arvalid, s4_arvalid, s3_arvalid, s2_arvalid, s1_arvalid, s0_arvalid};

    assign s_axi_rready  = {s5_rready, s4_rready, s3_rready, s2_rready, s1_rready, s0_rready};
    assign {s5_rid, s4_rid, s3_rid, s2_rid, s1_rid, s0_rid}             = s_axi_rid;
    assign {s5_rdata, s4_rdata, s3_rdata, s2_rdata, s1_rdata, s0_rdata}   = s_axi_rdata;
    assign {s5_rresp, s4_rresp, s3_rresp, s2_rresp, s1_rresp, s0_rresp}   = s_axi_rresp;
    assign {s5_rlast, s4_rlast, s3_rlast, s2_rlast, s1_rlast, s0_rlast}   = s_axi_rlast;
    assign {s5_rvalid, s4_rvalid, s3_rvalid, s2_rvalid, s1_rvalid, s0_rvalid} = s_axi_rvalid;

    // =========================================================================
    // 4. INSTANTIATE CÁC MASTER MODULES
    // =========================================================================
    // M0: I-Cache
    wire ic_uncache_en = (cpu_ic_addr >= 32'h4000_0000 && cpu_ic_addr <= 32'h4FFF_FFFF);

    icache u_icache (
        .clk             (clk_cpu),
        .rst_n           (reset_sys_n),
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
        .clk             (clk_cpu),
        .rst_n           (reset_sys_n),
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
    
    // Tầng 1: JTAG DTM (Giao tiếp vật lý JTAG)
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
        .dmi_resp_ready  (dmi_resp_ready),
        .dmi_resp_valid  (dmi_resp_valid),
        .dmi_resp_data   (dmi_resp_data),
        .dmi_resp_op     (dmi_resp_op)
    );

    // Tầng 2: Debug Module (Xử lý lệnh Halt/Resume/Regs và System Bus)
    rv_debug_module_sba u_debug_module (
        .clk_sys            (clk_dbg),
        .rst_sys_n          (rst_n),
        
        // Nối với JTAG DTM
        .dmi_req_valid      (dmi_req_valid),
        .dmi_req_addr       (dmi_req_addr),
        .dmi_req_data       (dmi_req_data),
        .dmi_req_op         (dmi_req_op),
        .dmi_resp_ready     (dmi_resp_ready),
        .dmi_resp_valid     (dmi_resp_valid),
        .dmi_resp_data      (dmi_resp_data),
        .dmi_resp_op        (dmi_resp_op),

        // Nối System Bus Access (SBA) sang AXI Wrapper
        .axi_req            (sba_req),
        .axi_op             (sba_op),
        .axi_size           (sba_size),
        .axi_addr           (sba_addr),
        .axi_wdata          (sba_wdata),
        .axi_ack            (sba_ack),
        .axi_rdata          (sba_rdata),
        .axi_resp           (sba_resp),

        // Nối sang CPU (Điều khiển Core & Registers)
        .cpu_halt_req       (dbg_halt_req),
        .cpu_resume_req     (dbg_resume_req),
        .cpu_halted         (dbg_halted),
        .cpu_reg_read_addr  (dbg_reg_read_addr),
        .cpu_reg_read_data  (dbg_reg_read_data),
        .cpu_reg_write_en   (dbg_reg_write_en),
        .cpu_reg_write_addr (dbg_reg_write_addr),
        .cpu_reg_write_data (dbg_reg_write_data),
        .ndmreset_req       (ndmreset_req)
    );

    // Tầng 3: AXI Master Wrapper (Đưa SBA vào bus hệ thống làm M2)
    dtm_axi_master u_dtm_axi (
        .clk_sys         (clk),
        .rst_sys_n       (rst_n),
        
        // Nhận lệnh từ SBA của Debug Module thay vì JTAG
        .i_req           (sba_req),
        .i_op            (sba_op),
        .i_size          (sba_size),
        .i_addr          (sba_addr),
        .i_wdata         (sba_wdata),
        .o_ack           (sba_ack),
        .o_resp          (sba_resp),
        .o_rdata         (sba_rdata),
        
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
        // Trọng số ưu tiên (MST_WEIGHT): 4 Master tương ứng 4 x 32-bit = 128 bit
        // Quy tắc: Số càng to ưu tiên càng cao.
        // Ví dụ: DMA (5) > DCache (4) > ICache (3) > Debug (1)
        .MST_WEIGHT (128'h00000005_00000004_00000003_00000001),
        // Gán địa chỉ Base và Mask cho 5 Slave
        // S0: ROM (0x0001_0000 - 64KB), S1: RAM (0x2000_0000 - 128KB)
        // S2: Flash (0x3000_0000 - 16MB), S3: SDRAM (0x8000_0000 - 64MB)
        // S4: APB Bridge (0x4000_0000 - 1MB)
        // S5: CLINT (0x0200_0000 - 64KB)
        .SLV_BASE_ADDR (192'h0200_0000_4000_0000_8000_0000_3000_0000_2000_0000_0001_0000),
        .SLV_ADDR_MASK (192'hFFFF_0000_FFF0_0000_FC00_0000_FF00_0000_FFFE_0000_FFFF_0000)
    ) u_axi_interconnect (
        .ACLK_i          (clk),
        .ARESETn_i       (reset_sys_n),
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
        .clk(clk), .rst_n(reset_sys_n),
        .s_axi_arid(s0_arid), .s_axi_araddr(s0_araddr), .s_axi_arlen(s0_arlen), .s_axi_arsize(s0_arsize), .s_axi_arburst(s0_arburst), .s_axi_arvalid(s0_arvalid), .s_axi_arready(s0_arready),
        .s_axi_rid(s0_rid), .s_axi_rdata(s0_rdata), .s_axi_rresp(s0_rresp), .s_axi_rlast(s0_rlast), .s_axi_rvalid(s0_rvalid), .s_axi_rready(s0_rready)
    );
    assign s0_awready = 0; assign s0_wready = 0; assign s0_bvalid = 0; // ROM read-only

    // S1: RAM
    axi_ram u_axi_ram (
        .clk(clk), .rst_n(reset_sys_n),
        .s_axi_awid(s1_awid), .s_axi_awaddr(s1_awaddr), .s_axi_awlen(s1_awlen), .s_axi_awsize(s1_awsize), .s_axi_awburst(s1_awburst), .s_axi_awvalid(s1_awvalid), .s_axi_awready(s1_awready),
        .s_axi_wdata(s1_wdata), .s_axi_wstrb(s1_wstrb), .s_axi_wlast(s1_wlast), .s_axi_wvalid(s1_wvalid), .s_axi_wready(s1_wready),
        .s_axi_bid(s1_bid), .s_axi_bresp(s1_bresp), .s_axi_bvalid(s1_bvalid), .s_axi_bready(s1_bready),
        .s_axi_arid(s1_arid), .s_axi_araddr(s1_araddr), .s_axi_arlen(s1_arlen), .s_axi_arsize(s1_arsize), .s_axi_arburst(s1_arburst), .s_axi_arvalid(s1_arvalid), .s_axi_arready(s1_arready),
        .s_axi_rid(s1_rid), .s_axi_rdata(s1_rdata), .s_axi_rresp(s1_rresp), .s_axi_rlast(s1_rlast), .s_axi_rvalid(s1_rvalid), .s_axi_rready(s1_rready)
    );

    // S2: SPI Flash
    axi_spi_flash u_axi_flash (
        .clk(clk), .rst_n(reset_sys_n),
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
        .clk(clk), .rst_n(reset_sys_n),
        .sdram_clk(sdram_clk), .sdram_cke(sdram_cke), .sdram_cs_n(sdram_cs_n), .sdram_ras_n(sdram_ras_n),
        .sdram_cas_n(sdram_cas_n), .sdram_we_n(sdram_we_n), .sdram_ba(sdram_ba), .sdram_addr(sdram_addr),
        .sdram_dqm(sdram_dqm), .sdram_dq_i(sdram_dq_i), .sdram_dq_o(sdram_dq_o), .sdram_dq_oe(sdram_dq_oe),
        .s_axi_awid(s3_awid), .s_axi_awaddr(s3_awaddr), .s_axi_awlen(s3_awlen), .s_axi_awsize(s3_awsize), .s_axi_awburst(s3_awburst), .s_axi_awvalid(s3_awvalid), .s_axi_awready(s3_awready),
        .s_axi_wdata(s3_wdata), .s_axi_wstrb(s3_wstrb), .s_axi_wlast(s3_wlast), .s_axi_wvalid(s3_wvalid), .s_axi_wready(s3_wready),
        .s_axi_bid(s3_bid), .s_axi_bresp(s3_bresp), .s_axi_bvalid(s3_bvalid), .s_axi_bready(s3_bready),
        .s_axi_arid(s3_arid), .s_axi_araddr(s3_araddr), .s_axi_arlen(s3_arlen), .s_axi_arsize(s3_arsize), .s_axi_arburst(s3_arburst), .s_axi_arvalid(s3_arvalid), .s_axi_arready(s3_arready),
        .s_axi_rid(s3_rid), .s_axi_rdata(s3_rdata), .s_axi_rresp(s3_rresp), .s_axi_rlast(s3_rlast), .s_axi_rvalid(s3_rvalid), .s_axi_rready(s3_rready)
    );

    // S4: AXI to APB Bridge
    wire [31:0] apb_paddr, apb_pwdata, apb_prdata;
    wire [3:0]  apb_pstrb;
    wire [2:0]  apb_pprot;
    wire        apb_psel, apb_penable, apb_pwrite, apb_pready, apb_pslverr;

    axi_to_apb_bridge u_axi_to_apb (
        .clk(clk), .rst_n(reset_sys_n),
        .s_axi_awid(s4_awid), .s_axi_awaddr(s4_awaddr), .s_axi_awlen(s4_awlen), .s_axi_awsize(s4_awsize), .s_axi_awburst(s4_awburst), .s_axi_awprot(s4_awprot), .s_axi_awvalid(s4_awvalid), .s_axi_awready(s4_awready),
        .s_axi_wdata(s4_wdata), .s_axi_wstrb(s4_wstrb), .s_axi_wlast(s4_wlast), .s_axi_wvalid(s4_wvalid), .s_axi_wready(s4_wready),
        .s_axi_bid(s4_bid), .s_axi_bresp(s4_bresp), .s_axi_bvalid(s4_bvalid), .s_axi_bready(s4_bready),
        .s_axi_arid(s4_arid), .s_axi_araddr(s4_araddr), .s_axi_arlen(s4_arlen), .s_axi_arsize(s4_arsize), .s_axi_arburst(s4_arburst), .s_axi_arprot(s4_arprot), .s_axi_arvalid(s4_arvalid), .s_axi_arready(s4_arready),
        .s_axi_rid(s4_rid), .s_axi_rdata(s4_rdata), .s_axi_rresp(s4_rresp), .s_axi_rlast(s4_rlast), .s_axi_rvalid(s4_rvalid), .s_axi_rready(s4_rready),
        // APB Master Output
        .m_apb_paddr(apb_paddr), .m_apb_psel(apb_psel), .m_apb_penable(apb_penable), .m_apb_pwrite(apb_pwrite), .m_apb_pwdata(apb_pwdata), .m_apb_pstrb(apb_pstrb), .m_apb_pprot(apb_pprot),
        .m_apb_pready(apb_pready), .m_apb_prdata(apb_prdata), .m_apb_pslverr(apb_pslverr)
    );

    wire [0:0] cpu_msip;
    wire [0:0] cpu_mtip;

    // S5: CLINT
    axi_clint #(
        .NUM_HARTS(1) // SoC của bạn hiện có 1 lõi
    ) u_axi_clint (
        .clk_i           (clk),
        .rst_ni          (reset_sys_n),
        .rtc_clk_i       (rtc_clk),

        // AXI4 Slave Interface (Nối với S5)
        .s_axi_awid      (s5_awid),
        .s_axi_awaddr    (s5_awaddr),
        .s_axi_awlen     (s5_awlen),
        .s_axi_awsize    (s5_awsize),
        .s_axi_awburst   (s5_awburst),
        .s_axi_awprot    (s5_awprot),
        .s_axi_awvalid   (s5_awvalid),
        .s_axi_awready   (s5_awready),

        .s_axi_wdata     (s5_wdata),
        .s_axi_wstrb     (s5_wstrb),
        .s_axi_wlast     (s5_wlast),
        .s_axi_wvalid    (s5_wvalid),
        .s_axi_wready    (s5_wready),

        .s_axi_bid       (s5_bid),
        .s_axi_bresp     (s5_bresp),
        .s_axi_bvalid    (s5_bvalid),
        .s_axi_bready    (s5_bready),

        .s_axi_arid      (s5_arid),
        .s_axi_araddr    (s5_araddr),
        .s_axi_arlen     (s5_arlen),
        .s_axi_arsize    (s5_arsize),
        .s_axi_arburst   (s5_arburst),
        .s_axi_arprot    (s5_arprot),
        .s_axi_arvalid   (s5_arvalid),
        .s_axi_arready   (s5_arready),

        .s_axi_rid       (s5_rid),
        .s_axi_rdata     (s5_rdata),
        .s_axi_rresp     (s5_rresp),
        .s_axi_rlast     (s5_rlast),
        .s_axi_rvalid    (s5_rvalid),
        .s_axi_rready    (s5_rready),

        // Xuất tín hiệu ngắt ra CPU
        .msip_o          (cpu_msip),
        .mtip_o          (cpu_mtip)
    );

    // =========================================================================
    // 7. APB INTERCONNECT
    // =========================================================================
    // Wires cho 10 APB Slaves
    wire [31:0] paddr_0, paddr_1, paddr_2, paddr_3, paddr_4, paddr_5, paddr_6, paddr_7, paddr_8, paddr_9;
    wire [31:0] pwdata_0, pwdata_1, pwdata_2, pwdata_3, pwdata_4, pwdata_5, pwdata_6, pwdata_7, pwdata_8, pwdata_9;
    wire [31:0] prdata_0, prdata_1, prdata_2, prdata_3, prdata_4, prdata_5, prdata_6, prdata_7, prdata_8, prdata_9;
    wire psel_0, psel_1, psel_2, psel_3, psel_4, psel_5, psel_6, psel_7, psel_8, psel_9;
    wire penable_0, penable_1, penable_2, penable_3, penable_4, penable_5, penable_6, penable_7, penable_8, penable_9;
    wire pwrite_0, pwrite_1, pwrite_2, pwrite_3, pwrite_4, pwrite_5, pwrite_6, pwrite_7, pwrite_8, pwrite_9;
    wire pready_0, pready_1, pready_2, pready_3, pready_4, pready_5, pready_6, pready_7, pready_8, pready_9;

    apb_interconnect u_apb_interconnect (
        .clk(clk), .rst_n(reset_sys_n),
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
        .s7_paddr(paddr_7), .s7_psel(psel_7), .s7_penable(penable_7), .s7_pwrite(pwrite_7), .s7_pwdata(pwdata_7), .s7_pready(pready_7), .s7_prdata(prdata_7), .s7_pslverr(1'b0),
        .s8_paddr(paddr_8), .s8_psel(psel_8), .s8_penable(penable_8), .s8_pwrite(pwrite_8), .s8_pwdata(pwdata_8), .s8_pready(pready_8), .s8_prdata(prdata_8), .s8_pslverr(1'b0)
    );

    // =========================================================================
    // 8. KHỞI TẠO CÁC NGOẠI VI APB
    // =========================================================================
    / S0: UART (Có IRQ, DMA_TX, DMA_RX)
    apb_uart u_apb_uart (
        .pclk(clk_uart), .presetn(reset_sys_n),
        .paddr(paddr_0[11:0]), .psel(psel_0), .penable(penable_0), .pwrite(pwrite_0), 
        .pwdata(pwdata_0), .prdata(prdata_0), .pready(pready_0), .pslverr(pslverr_0),
        .uart_clk(clk_uart), .uart_rst_n(rst_n), 
        .rxd(uart_rx), .txd(uart_tx),
        .uart_irq(uart_irq), .dma_tx_req(uart_dma_tx), .dma_rx_req(uart_dma_rx)
    );

    // S1: GPIO (Có IRQ)
    apb_gpio u_apb_gpio (
        .pclk(clk_gpio), .presetn(reset_sys_n),
        .paddr(paddr_1[11:0]), .psel(psel_1), .penable(penable_1), .pwrite(pwrite_1), 
        .pwdata(pwdata_1), .prdata(prdata_1), .pready(pready_1), .pslverr(pslverr_1),
        .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_dir(gpio_oe), 
        .gpio_irq(gpio_irq)
    );

    // S2: PWM (Thay thế Timer cũ)
    apb_pwm u_apb_pwm (
        .pclk(clk_pwm), .presetn(reset_sys_n),
        .paddr(paddr_2[11:0]), .psel(psel_2), .penable(penable_2), .pwrite(pwrite_2), 
        .pwdata(pwdata_2), .pstrb(apb_pstrb), .prdata(prdata_2), .pready(pready_2), .pslverr(pslverr_2),
        .pwm_out(pwm_out)
    );

    // S3: SPI (Có IRQ, DMA_TX, DMA_RX)
    apb_spi u_apb_spi (
        .pclk(clk_spi), .presetn(reset_sys_n),
        .paddr(paddr_3[11:0]), .psel(psel_3), .penable(penable_3), .pwrite(pwrite_3), 
        .pwdata(pwdata_3), .pstrb(apb_pstrb), .prdata(prdata_3), .pready(pready_3), .pslverr(pslverr_3),
        .sclk(spi_sck), .mosi(spi_mosi), .miso(spi_miso), .cs_n(spi_ss),
        .spi_irq(spi_irq), .dma_tx_req(spi_dma_tx), .dma_rx_req(spi_dma_rx)
    );

    // S4: I2C (Có IRQ, DMA_TX, DMA_RX)
    wire i2c_scl_i, i2c_scl_o, i2c_scl_oe;
    wire i2c_sda_i, i2c_sda_o, i2c_sda_oe;
    assign i2c_scl   = i2c_scl_oe ? i2c_scl_o : 1'bz;
    assign i2c_scl_i = i2c_scl;
    assign i2c_sda   = i2c_sda_oe ? i2c_sda_o : 1'bz;
    assign i2c_sda_i = i2c_sda;

    apb_i2c u_apb_i2c (
        .pclk(clk_i2c), .presetn(reset_sys_n),
        .paddr(paddr_4[11:0]), .psel(psel_4), .penable(penable_4), .pwrite(pwrite_4), 
        .pwdata(pwdata_4), .pstrb(apb_pstrb), .prdata(prdata_4), .pready(pready_4), .pslverr(pslverr_4),
        .scl_o(i2c_scl_o), .scl_oen(i2c_scl_oe), .scl_i(i2c_scl_i),
        .sda_o(i2c_sda_o), .sda_oen(i2c_sda_oe), .sda_i(i2c_sda_i),
        .i2c_irq(i2c_irq), .dma_tx_req(i2c_dma_tx), .dma_rx_req(i2c_dma_rx)
    );

    // S5: Watchdog (Có IRQ, RST)
    apb_watchdog u_apb_watchdog (
        .pclk(clk), .presetn(reset_sys_n),
        .paddr(paddr_5[11:0]), .psel(psel_5), .penable(penable_5), .pwrite(pwrite_5), 
        .pwdata(pwdata_5), .pstrb(apb_pstrb), .prdata(prdata_5), .pready(pready_5), .pslverr(pslverr_5),
        .wdt_irq(wdt_irq), .wdt_rst(wdt_rst)
    );

    // S6: CORDIC (Thuần APB)
    apb_cordic u_apb_cordic (
        .pclk(clk_cordic), .presetn(reset_sys_n),
        .paddr(paddr_6[11:0]), .psel(psel_6), .penable(penable_6), .pwrite(pwrite_6), 
        .pwdata(pwdata_6), .pstrb(apb_pstrb), .prdata(prdata_6), .pready(pready_6), .pslverr(pslverr_6)
    );

    // S7: Syscon (Quản lý Clock/Reset/Power)
    wire clk_en_cpu, clk_en_dbg, clk_en_pwm, clk_en_uart;
    wire clk_en_spi, clk_en_i2c, clk_en_gpio, clk_en_acc;

    apb_syscon u_apb_syscon (
        .pclk(clk), .presetn(rst_n),
        .paddr(paddr_7[11:0]), .psel(psel_7), .penable(penable_7), .pwrite(pwrite_7), 
        .pwdata(pwdata_7), .prdata(prdata_7), .pready(pready_7), .pslverr(pslverr_7),
        
        .o_reset_vector(syscon_reset_vector),
        .i_wfi_sleep   (wfi_sleep_state),
        .i_ext_irq     (cpu_meip),
        
        .o_cpu_clk_en  (clk_en_cpu), 
        .o_dbg_clk_en  (clk_en_dbg),
        .o_pwm_clk_en  (clk_en_pwm),
        .o_urt_clk_en  (clk_en_uart),
        .o_spi_clk_en  (clk_en_spi),
        .o_i2c_clk_en  (clk_en_i2c),
        .o_gpo_clk_en  (clk_en_gpio),
        .o_acc_clk_en  (clk_en_acc)
    );

    // S8: PLIC (Quản lý Ngắt Tập Trung)
    apb_plic #(.ALGORITHM("BINARY_TREE")) u_apb_plic (
        .clk_i      (clk),
        .rst_ni     (reset_sys_n),
        .paddr      (paddr_8),
        .psel       (psel_8),
        .penable    (penable_8),
        .pwrite     (pwrite_8),
        .pwdata     (pwdata_8),
        .pready     (pready_8),
        .prdata     (prdata_8),
        .pslverr    (pslverr_8),
        
        .irq_src_i  (plic_irq_src), // Tổ hợp ngắt từ các Peripheral
        .irq_o      (cpu_meip)
    );

    // =========================================================================
    // DMA CONTROLLER (AXI Master 3 / APB Slave 9)
    // =========================================================================
    axi_apb_dma u_axi_apb_dma (
        .clk_bus        (clk),
        .rst_bus_n      (reset_sys_n),

        // --- GIAO TIẾP APB (Cấu hình) ---
        .s_apb_psel     (s9_psel),
        .s_apb_penable  (s9_penable),
        .s_apb_pwrite   (s9_pwrite),
        .s_apb_paddr    (s9_paddr[12:0]),
        .s_apb_pwdata   (s9_pwdata),
        .s_apb_prdata   (s9_prdata),
        .s_apb_pslverr  (s9_pslverr),
        .s_apb_pready   (s9_pready),

        // --- NGẮT & HARDWARE TRIGGER ---
        .dma_irq        (dma_irq),
        .periph_req     (periph_dma_req),
        .periph_clr     (periph_dma_clr),

        // --- GIAO TIẾP AXI MASTER (Đọc/Ghi bộ nhớ) ---
        // Write Address
        .AWADDR         (m3_awaddr),
        .AWLEN          (m3_awlen),
        .AWSIZE         (m3_awsize),
        .AWID           (m3_awid),
        .AWVALID        (m3_awvalid),
        .AWREADY        (m3_awready),
        // Write Data
        .WDATA          (m3_wdata),
        .WSTRB          (m3_wstrb),
        .WLAST          (m3_wlast),
        .WVALID         (m3_wvalid),
        .WREADY         (m3_wready),
        // Write Response
        .BID            (m3_bid),
        .BRESP          (m3_bresp),
        .BVALID         (m3_bvalid),
        .BREADY         (m3_bready),
        // Read Address
        .ARADDR         (m3_araddr),
        .ARLEN          (m3_arlen),
        .ARSIZE         (m3_arsize),
        .ARID           (m3_arid),
        .ARVALID        (m3_arvalid),
        .ARREADY        (m3_arready),
        // Read Data
        .RDATA          (m3_rdata),
        .RID            (m3_rid),
        .RRESP          (m3_rresp),
        .RLAST          (m3_rlast),
        .RVALID         (m3_rvalid),
        .RREADY         (m3_rready)
    );

endmodule