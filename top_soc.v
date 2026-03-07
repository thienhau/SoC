`timescale 1ns / 1ps

module top_soc (
    // --- Hệ thống Clock và Reset ---
    input  wire        clk_sys,      // 300MHz - 400MHz
    input  wire        rst_n_pad,    // Reset cứng từ Pad (Active Low)

    // --- Giao tiếp JTAG (4-5 pins) ---
    input  wire        jtag_tck,
    input  wire        jtag_trst_n,
    input  wire        jtag_tms,
    input  wire        jtag_tdi,
    output wire        jtag_tdo,

    // --- UART Interface ---
    input  wire        uart_rx,
    output wire        uart_tx,

    // --- SPI Interface ---
    output wire        spi_sclk,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire        spi_cs_n,

    // --- I2C Interface (Open-Drain) ---
    output wire        i2c_scl_o,
    output wire        i2c_scl_oen,
    input  wire        i2c_scl_i,
    output wire        i2c_sda_o,
    output wire        i2c_sda_oen,
    input  wire        i2c_sda_i,

    // --- GPIO Interface ---
    input  wire [31:0] gpio_in,
    output wire [31:0] gpio_out,
    output wire [31:0] gpio_dir
);

    // -------------------------------------------------------------------------
    // 1. RESET SYNCHRONIZER (Chống Metastability khi thoát Reset)
    // -------------------------------------------------------------------------
    reg [2:0] rst_sync;
    always @(posedge clk_sys or negedge rst_n_pad) begin
        if (!rst_n_pad) rst_sync <= 3'b000;
        else            rst_sync <= {rst_sync[1:0], 1'b1};
    end
    wire sys_rst_n = rst_sync[2];

    // -------------------------------------------------------------------------
    // 2. KHAI BÁO CÁC TÍN HIỆU KẾT NỐI (WIRE HARNESS)
    // -------------------------------------------------------------------------
    
    // Tín hiệu từ Core ra Cache
    wire [15:0] sys_reset_vector;
    wire        plic_ext_irq;
    wire        cpu_wfi_sleep;
    wire        cpu_ic_req, cpu_dc_rd, cpu_dc_wr;
    wire [15:0] cpu_ic_addr, cpu_dc_addr;
    wire [31:0] cpu_ic_rdata, cpu_dc_wdata, cpu_dc_rdata;
    wire        cpu_ic_stall, cpu_dc_stall;
    wire [1:0]  cpu_dc_size;

    // Tín hiệu từ Cache ra Adapter
    wire        ic_mem_req, dc_mem_rd, dc_mem_wr;
    wire [15:0] ic_mem_addr, dc_mem_addr;
    wire [31:0] ic_mem_rdata, dc_mem_wdata, dc_mem_rdata;
    wire [1:0]  dc_mem_size;
    wire        ic_mem_ready, dc_mem_ready;

    // Tín hiệu Debug (JTAG <-> Debug Module <-> AXI Master)
    wire        j_shift, j_update, j_capture, j_sel_dmi, j_tdo_dmi;
    wire [4:0]  j_ir;
    wire        dtm_req, dtm_ack;
    wire [1:0]  dtm_op, dtm_resp;
    wire [15:0] dtm_addr;
    wire [31:0] dtm_wdata, dtm_rdata;

    // Tín hiệu Clock Gating từ System Controller
    wire clk_en_cpu, clk_en_dbg;
    wire clk_en_tmr, clk_en_urt, clk_en_spi, clk_en_i2c, clk_en_gpo, clk_en_acc;
    
    wire clk_cpu_gated;
    wire clk_dbg_gated;
    wire clk_tmr_gated;
    wire clk_urt_gated;
    wire clk_spi_gated;
    wire clk_i2c_gated;
    wire clk_gpo_gated;
    wire clk_acc_gated;

    // Instantiate ICG Cells
    clock_gate CG_CPU   (.clk_in(clk_sys), .en(clk_en_cpu), .test_en(1'b0), .clk_out(clk_cpu_gated));
    clock_gate CG_DBG   (.clk_in(clk_sys), .en(clk_en_dbg), .test_en(1'b0), .clk_out(clk_dbg_gated));
    clock_gate CG_TMR   (.clk_in(clk_sys), .en(clk_en_tmr), .test_en(1'b0), .clk_out(clk_tmr_gated));
    clock_gate CG_URT   (.clk_in(clk_sys), .en(clk_en_urt), .test_en(1'b0), .clk_out(clk_urt_gated));
    clock_gate CG_SPI   (.clk_in(clk_sys), .en(clk_en_spi), .test_en(1'b0), .clk_out(clk_spi_gated));
    clock_gate CG_I2C   (.clk_in(clk_sys), .en(clk_en_i2c), .test_en(1'b0), .clk_out(clk_i2c_gated));
    clock_gate CG_GPO   (.clk_in(clk_sys), .en(clk_en_gpo), .test_en(1'b0), .clk_out(clk_gpo_gated));
    clock_gate CG_ACC   (.clk_in(clk_sys), .en(clk_en_acc), .test_en(1'b0), .clk_out(clk_acc_gated));

    // -------------------------------------------------------------------------
    // 3. KHỐI ĐIỀU KHIỂN TRUNG TÂM (RISC-V CORE & CACHE)
    // -------------------------------------------------------------------------
    riscv_pipeline CPU_CORE (
        .clk(clk_cpu_gated), .reset(!sys_rst_n), .riscv_start(1'b1),
        .external_irq_in(plic_ext_irq), .reset_vector_in(sys_reset_vector), .wfi_sleep_out(cpu_wfi_sleep),
        .icache_read_req(cpu_ic_req), .icache_addr(cpu_ic_addr), .icache_read_data(cpu_ic_rdata), .icache_stall(cpu_ic_stall),
        .dcache_read_req(cpu_dc_rd), .dcache_write_req(cpu_dc_wr), .dcache_addr(cpu_dc_addr),
        .dcache_write_data(cpu_dc_wdata), .dcache_read_data(cpu_dc_rdata), .dcache_stall(cpu_dc_stall),
        .mem_size_top(cpu_dc_size)
    );

    icache I_CACHE (
        .clk(clk_sys), .reset(!sys_rst_n), .flush(1'b0),
        .cpu_read_req(cpu_ic_req), .cpu_addr(cpu_ic_addr), .cpu_read_data(cpu_ic_rdata), .icache_stall(cpu_ic_stall),
        .mem_read_req(ic_mem_req), .mem_addr(ic_mem_addr), .mem_read_data(ic_mem_rdata), .mem_read_valid(ic_mem_ready)
    );

    dcache D_CACHE (
        .clk(clk_sys), .reset(!sys_rst_n),
        .cpu_read_req(cpu_dc_rd), .cpu_write_req(cpu_dc_wr), .cpu_addr(cpu_dc_addr), .cpu_write_data(cpu_dc_wdata), 
        .cpu_read_data(cpu_dc_rdata), .dcache_stall(cpu_dc_stall), .mem_size(cpu_dc_size),
        .mem_read_req(dc_mem_rd), .mem_write_req(dc_mem_wr), .mem_addr(dc_mem_addr), 
        .mem_write_data(dc_mem_wdata), .mem_read_data(dc_mem_rdata), .mem_size_out(dc_mem_size), .mem_ready(dc_mem_ready)
    );

    // -------------------------------------------------------------------------
    // 4. HỆ THỐNG DEBUG (JTAG TAP -> DEBUG MODULE -> DTM AXI MASTER)
    // -------------------------------------------------------------------------
    jtag_tap JTAG_TAP_INST (
        .tck(jtag_tck), .trst_n(jtag_trst_n), .tms(jtag_tms), .tdi(jtag_tdi), .tdo(jtag_tdo),
        .o_ir(j_ir), .o_shift_dr(j_shift), .o_capture_dr(j_capture), .o_update_dr(j_update),
        .o_sel_dmi(j_sel_dmi), .i_tdo_dmi(j_tdo_dmi)
    );

    debug_module DBG_MODULE_INST (
        .tck(jtag_tck), .trst_n(jtag_trst_n), .i_shift_dr(j_shift), .i_capture_dr(j_capture),
        .i_update_dr(j_update), .i_sel_dmi(j_sel_dmi), .i_tdi(jtag_tdi), .o_tdo(j_tdo_dmi),
        .clk_sys(clk_dbg_gated), .rst_sys_n(sys_rst_n),
        .req_sys(dtm_req), .op_sys(dtm_op), .addr_sys(dtm_addr), .wdata_sys(dtm_wdata),
        .ack_sys(dtm_ack), .resp_sys(dtm_resp), .rdata_sys(dtm_rdata)
    );

    // -------------------------------------------------------------------------
    // 5. ĐƯỜNG TRUYỀN CHÍNH AXI (AXI BUS WIRES)
    // -------------------------------------------------------------------------
    // Wires cho 3 Master
    wire [15:0] m0_araddr, m1_araddr, m1_awaddr, m2_araddr, m2_awaddr;
    wire m0_arvalid, m0_arready, m0_rvalid, m0_rready;
    wire m1_awvalid, m1_awready, m1_wvalid, m1_wready, m1_bvalid, m1_bready, m1_arvalid, m1_arready, m1_rvalid, m1_rready;
    wire m2_awvalid, m2_awready, m2_wvalid, m2_wready, m2_bvalid, m2_bready, m2_arvalid, m2_arready, m2_rvalid, m2_rready;
    wire [31:0] m0_rdata, m1_wdata, m1_rdata, m2_wdata, m2_rdata;
    wire [3:0]  m1_wstrb, m2_wstrb;
    wire [1:0]  m0_rresp, m1_bresp, m1_rresp, m2_bresp, m2_rresp;

    // --- M0: I-Cache AXI Adapter ---
    axi_master_adapter M0_ICACHE_ADAPT (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .cpu_read_req(ic_mem_req), .cpu_write_req(1'b0), .cpu_addr(ic_mem_addr), .cpu_wdata(32'b0), .cpu_mem_size(2'b00),
        .cpu_rdata(ic_mem_rdata), .cpu_ready(ic_mem_ready),
        .m_axi_araddr(m0_araddr), .m_axi_arvalid(m0_arvalid), .m_axi_arready(m0_arready),
        .m_axi_rdata(m0_rdata), .m_axi_rresp(m0_rresp), .m_axi_rvalid(m0_rvalid), .m_axi_rready(m0_rready)
    );

    // --- M1: D-Cache AXI Adapter ---
    axi_master_adapter M1_DCACHE_ADAPT (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .cpu_read_req(dc_mem_rd), .cpu_write_req(dc_mem_wr), .cpu_addr(dc_mem_addr), .cpu_wdata(dc_mem_wdata), .cpu_mem_size(dc_mem_size),
        .cpu_rdata(dc_mem_rdata), .cpu_ready(dc_mem_ready),
        .m_axi_awaddr(m1_awaddr), .m_axi_awvalid(m1_awvalid), .m_axi_awready(m1_awready),
        .m_axi_wdata(m1_wdata), .m_axi_wstrb(m1_wstrb), .m_axi_wvalid(m1_wvalid), .m_axi_wready(m1_wready),
        .m_axi_bresp(m1_bresp), .m_axi_bvalid(m1_bvalid), .m_axi_bready(m1_bready),
        .m_axi_araddr(m1_araddr), .m_axi_arvalid(m1_arvalid), .m_axi_arready(m1_arready),
        .m_axi_rdata(m1_rdata), .m_axi_rresp(m1_rresp), .m_axi_rvalid(m1_rvalid), .m_axi_rready(m1_rready)
    );

    // --- M2: DTM AXI Master (Khối nạp code/Debug) ---
    dtm_axi_master M2_DTM_AXI_INST (
        .clk_sys(clk_sys), .rst_sys_n(sys_rst_n),
        .i_req(dtm_req), .i_op(dtm_op), .i_addr(dtm_addr), .i_wdata(dtm_wdata),
        .o_ack(dtm_ack), .o_resp(dtm_resp), .o_rdata(dtm_rdata),
        .m_axi_awaddr(m2_awaddr), .m_axi_awvalid(m2_awvalid), .m_axi_awready(m2_awready),
        .m_axi_wdata(m2_wdata), .m_axi_wstrb(m2_wstrb), .m_axi_wvalid(m2_wvalid), .m_axi_wready(m2_wready),
        .m_axi_bresp(m2_bresp), .m_axi_bvalid(m2_bvalid), .m_axi_bready(m2_bready),
        .m_axi_araddr(m2_araddr), .m_axi_arvalid(m2_arvalid), .m_axi_arready(m2_arready),
        .m_axi_rdata(m2_rdata), .m_axi_rresp(m2_rresp), .m_axi_rvalid(m2_rvalid), .m_axi_rready(m2_rready)
    );

    // -------------------------------------------------------------------------
    // 6. AXI INTERCONNECT (Bộ định tuyến dữ liệu chính)
    // -------------------------------------------------------------------------
    wire [15:0] s0_araddr, s1_awaddr, s1_araddr, s2_awaddr, s2_araddr;
    wire s0_arvalid, s0_arready, s0_rvalid, s0_rready;
    wire s1_awvalid, s1_awready, s1_wvalid, s1_wready, s1_bvalid, s1_bready, s1_arvalid, s1_arready, s1_rvalid, s1_rready;
    wire s2_awvalid, s2_awready, s2_wvalid, s2_wready, s2_bvalid, s2_bready, s2_arvalid, s2_arready, s2_rvalid, s2_rready;
    wire [31:0] s0_rdata, s1_wdata, s1_rdata, s2_wdata, s2_rdata;
    wire [3:0]  s1_wstrb, s2_wstrb;
    wire [1:0]  s0_rresp, s1_bresp, s1_rresp, s2_bresp, s2_rresp;

    axi_interconnect MAIN_BUS_MATRIX (
        .clk(clk_sys), .rst_n(sys_rst_n),
        // Kết nối Master 0 (I-Cache)
        .m0_araddr(m0_araddr), .m0_arvalid(m0_arvalid), .m0_arready(m0_arready), .m0_rdata(m0_rdata), .m0_rresp(m0_rresp), .m0_rvalid(m0_rvalid), .m0_rready(m0_rready),
        // Kết nối Master 1 (D-Cache)
        .m1_awaddr(m1_awaddr), .m1_awvalid(m1_awvalid), .m1_awready(m1_awready), .m1_wdata(m1_wdata), .m1_wstrb(m1_wstrb), .m1_wvalid(m1_wvalid), .m1_wready(m1_wready), .m1_bresp(m1_bresp), .m1_bvalid(m1_bvalid), .m1_bready(m1_bready),
        .m1_araddr(m1_araddr), .m1_arvalid(m1_arvalid), .m1_arready(m1_arready), .m1_rdata(m1_rdata), .m1_rresp(m1_rresp), .m1_rvalid(m1_rvalid), .m1_rready(m1_rready),
        // Kết nối Master 2 (Debug Module)
        .m2_awaddr(m2_awaddr), .m2_awvalid(m2_awvalid), .m2_awready(m2_awready), .m2_wdata(m2_wdata), .m2_wstrb(m2_wstrb), .m2_wvalid(m2_wvalid), .m2_wready(m2_wready), .m2_bresp(m2_bresp), .m2_bvalid(m2_bvalid), .m2_bready(m2_bready),
        .m2_araddr(m2_araddr), .m2_arvalid(m2_arvalid), .m2_arready(m2_arready), .m2_rdata(m2_rdata), .m2_rresp(m2_rresp), .m2_rvalid(m2_rvalid), .m2_rready(m2_rready),
        
        // Kết nối Slave 0 (ROM: 0x1000 - 0x4FFF)
        .s0_araddr(s0_araddr), .s0_arvalid(s0_arvalid), .s0_arready(s0_arready), .s0_rdata(s0_rdata), .s0_rresp(s0_rresp), .s0_rvalid(s0_rvalid), .s0_rready(s0_rready),
        // Kết nối Slave 1 (RAM: 0x8000 - 0xFFFF)
        .s1_awaddr(s1_awaddr), .s1_awvalid(s1_awvalid), .s1_awready(s1_awready), .s1_wdata(s1_wdata), .s1_wstrb(s1_wstrb), .s1_wvalid(s1_wvalid), .s1_wready(s1_wready), .s1_bresp(s1_bresp), .s1_bvalid(s1_bvalid), .s1_bready(s1_bready),
        .s1_araddr(s1_araddr), .s1_arvalid(s1_arvalid), .s1_arready(s1_arready), .s1_rdata(s1_rdata), .s1_rresp(s1_rresp), .s1_rvalid(s1_rvalid), .s1_rready(s1_rready),
        // Kết nối Slave 2 (APB: 0x5000 - 0x7FFF)
        .s2_awaddr(s2_awaddr), .s2_awvalid(s2_awvalid), .s2_awready(s2_awready), .s2_wdata(s2_wdata), .s2_wstrb(s2_wstrb), .s2_wvalid(s2_wvalid), .s2_wready(s2_wready), .s2_bresp(s2_bresp), .s2_bvalid(s2_bvalid), .s2_bready(s2_bready),
        .s2_araddr(s2_araddr), .s2_arvalid(s2_arvalid), .s2_arready(s2_arready), .s2_rdata(s2_rdata), .s2_rresp(s2_rresp), .s2_rvalid(s2_rvalid), .s2_rready(s2_rready)
    );

    // --- S0: BootROM ---
    axi_rom BOOT_ROM_INST (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .s_axi_araddr(s0_araddr), .s_axi_arvalid(s0_arvalid), .s_axi_arready(s0_arready),
        .s_axi_rdata(s0_rdata), .s_axi_rresp(s0_rresp), .s_axi_rvalid(s0_rvalid), .s_axi_rready(s0_rready)
    );

    // --- S1: System RAM ---
    axi_ram SYSTEM_RAM_INST (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .s_axi_awaddr(s1_awaddr), .s_axi_awvalid(s1_awvalid), .s_axi_awready(s1_awready), .s_axi_wdata(s1_wdata), .s_axi_wstrb(s1_wstrb), .s_axi_wvalid(s1_wvalid), .s_axi_wready(s1_wready), .s_axi_bresp(s1_bresp), .s_axi_bvalid(s1_bvalid), .s_axi_bready(s1_bready),
        .s_axi_araddr(s1_araddr), .s_axi_arvalid(s1_arvalid), .s_axi_arready(s1_arready), .s_axi_rdata(s1_rdata), .s_axi_rresp(s1_rresp), .s_axi_rvalid(s1_rvalid), .s_axi_rready(s1_rready)
    );

    // -------------------------------------------------------------------------
    // 7. PHÂN HỆ NGOẠI VI APB (APB SUBSYSTEM)
    // -------------------------------------------------------------------------
    wire [15:0] apb_paddr;
    wire [31:0] apb_pwdata, apb_prdata;
    wire [3:0]  apb_pstrb;
    wire        apb_psel, apb_penable, apb_pwrite, apb_pready, apb_pslverr;

    // --- AXI to APB Bridge ---
    axi_to_apb_bridge BRIDGE_INST (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .s_axi_awaddr(s2_awaddr), .s_axi_awvalid(s2_awvalid), .s_axi_awready(s2_awready), .s_axi_wdata(s2_wdata), .s_axi_wstrb(s2_wstrb), .s_axi_wvalid(s2_wvalid), .s_axi_wready(s2_wready), .s_axi_bresp(s2_bresp), .s_axi_bvalid(s2_bvalid), .s_axi_bready(s2_bready),
        .s_axi_araddr(s2_araddr), .s_axi_arvalid(s2_arvalid), .s_axi_arready(s2_arready), .s_axi_rdata(s2_rdata), .s_axi_rresp(s2_rresp), .s_axi_rvalid(s2_rvalid), .s_axi_rready(s2_rready),
        .m_apb_paddr(apb_paddr), .m_apb_psel(apb_psel), .m_apb_penable(apb_penable), .m_apb_pwrite(apb_pwrite), .m_apb_pwdata(apb_pwdata), .m_apb_pstrb(apb_pstrb), .m_apb_pready(apb_pready), .m_apb_prdata(apb_prdata), .m_apb_pslverr(apb_pslverr)
    );

    // --- APB Interconnect ---
    wire sel_syscon, sel_plic, sel_timer, sel_uart, sel_spi, sel_i2c, sel_gpio, sel_accel;
    wire [31:0] rdata_syscon, rdata_plic, rdata_timer, rdata_uart, rdata_spi, rdata_i2c, rdata_gpio, rdata_accel;
    wire ready_syscon, ready_plic, ready_timer, ready_uart, ready_spi, ready_i2c, ready_gpio, ready_accel;
    wire err_syscon, err_plic, err_timer, err_uart, err_spi, err_i2c, err_gpio, err_accel;

    apb_interconnect APB_BUS_MATRIX (
        .m_paddr(apb_paddr), .m_psel(apb_psel), .m_penable(apb_penable), .m_pwrite(apb_pwrite), .m_pwdata(apb_pwdata), .m_pstrb(apb_pstrb),
        .m_prdata(apb_prdata), .m_pready(apb_pready), .m_pslverr(apb_pslverr),
        
        .s0_psel(sel_syscon), .s0_prdata(rdata_syscon), .s0_pready(ready_syscon), .s0_pslverr(err_syscon),
        .s1_psel(sel_plic),   .s1_prdata(rdata_plic),   .s1_pready(ready_plic),   .s1_pslverr(err_plic),
        .s2_psel(sel_timer),  .s2_prdata(rdata_timer),  .s2_pready(ready_timer),  .s2_pslverr(err_timer),
        .s3_psel(sel_uart),   .s3_prdata(rdata_uart),   .s3_pready(ready_uart),   .s3_pslverr(err_uart),
        .s4_psel(sel_spi),    .s4_prdata(rdata_spi),    .s4_pready(ready_spi),    .s4_pslverr(err_spi),
        .s5_psel(sel_i2c),    .s5_prdata(rdata_i2c),    .s5_pready(ready_i2c),    .s5_pslverr(err_i2c),
        .s6_psel(sel_gpio),   .s6_prdata(rdata_gpio),   .s6_pready(ready_gpio),   .s6_pslverr(err_gpio),
        .s7_psel(sel_accel),  .s7_prdata(rdata_accel),  .s7_pready(ready_accel),  .s7_pslverr(err_accel)
    );

    // -------------------------------------------------------------------------
    // 8. KHAI BÁO CỤ THỂ TỪNG NGOẠI VI (FULL INSTANTIATION)
    // -------------------------------------------------------------------------
    wire irq_timer, irq_uart, irq_spi, irq_i2c, irq_gpio, irq_accel;

    // --- SYSCON: Quản lý Reset Vector ---
    apb_syscon SYSCON_INST (
        .pclk(clk_sys), .presetn(sys_rst_n), 
        .psel(sel_syscon), .penable(apb_penable), .pwrite(apb_pwrite), 
        .paddr(apb_paddr), .pwdata(apb_pwdata),
        .pready(ready_syscon), .prdata(rdata_syscon), .pslverr(err_syscon), 
        .o_reset_vector(sys_reset_vector),
        
        // Quản lý năng lượng WFI
        .i_wfi_sleep(cpu_wfi_sleep),
        .i_ext_irq(plic_ext_irq),    // Báo thức CPU khi có ngắt
        
        // Xuất tín hiệu Enable Clock tới các ICG
        .o_cpu_clk_en(clk_en_cpu),
        .o_dbg_clk_en(clk_en_dbg),
        .o_tmr_clk_en(clk_en_tmr),
        .o_urt_clk_en(clk_en_urt),
        .o_spi_clk_en(clk_en_spi),
        .o_i2c_clk_en(clk_en_i2c),
        .o_gpo_clk_en(clk_en_gpo),
        .o_acc_clk_en(clk_en_acc)
    );

    // --- PLIC: Interrupt Controller ---
    apb_interrupt_controller #(
        .ADDR_WIDTH(12),
        .DATA_WIDTH(32),
        .NUM_IRQ(6)
    ) PLIC_INST (
        // APB Bus Interface
        .pclk(clk_sys), 
        .presetn(sys_rst_n), 
        .paddr(apb_paddr[11:0]), 
        .psel(sel_plic), 
        .penable(apb_penable), 
        .pwrite(apb_pwrite), 
        .pwdata(apb_pwdata), 
        .pstrb(apb_pstrb), 
        .pready(ready_plic), 
        .prdata(rdata_plic), 
        .pslverr(err_plic),

        // 6 Interrupt Sources
        .irq_timer(irq_timer),  // ID 1
        .irq_uart(irq_uart),    // ID 2
        .irq_spi(irq_spi),      // ID 3
        .irq_i2c(irq_i2c),      // ID 4
        .irq_gpio(irq_gpio),    // ID 5
        .irq_accel(irq_accel),  // ID 6

        // Output to CPU
        .cpu_ext_irq(plic_ext_irq)
    );
    assign ready_plic = 1'b1; assign err_plic = 1'b0;

    // --- Timer ---
    apb_timer TIMER_INST (.pclk(clk_tmr_gated), .presetn(sys_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_timer), .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_timer), .prdata(rdata_timer), .pslverr(err_timer), .timer_irq(irq_timer));
    
    // --- UART ---
    apb_uart  UART_INST  (.pclk(clk_urt_gated), .presetn(sys_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_uart),  .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_uart),  .prdata(rdata_uart),  .pslverr(err_uart),  .rx(uart_rx), .tx(uart_tx), .uart_irq(irq_uart));
    
    // --- SPI ---
    apb_spi   SPI_INST   (.pclk(clk_spi_gated), .presetn(sys_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_spi),   .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_spi),   .prdata(rdata_spi),   .pslverr(err_spi),   .sclk(spi_sclk), .mosi(spi_mosi), .miso(spi_miso), .cs_n(spi_cs_n), .spi_irq(irq_spi));
    
    // --- I2C ---
    apb_i2c   I2C_INST   (.pclk(clk_i2c_gated), .presetn(sys_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_i2c),   .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_i2c),   .prdata(rdata_i2c),   .pslverr(err_i2c),   .scl_o(i2c_scl_o), .scl_oen(i2c_scl_oen), .scl_i(i2c_scl_i), .sda_o(i2c_sda_o), .sda_oen(i2c_sda_oen), .sda_i(i2c_sda_i), .i2c_irq(irq_i2c));
    
    // --- GPIO ---
    apb_gpio  GPIO_INST  (.pclk(clk_gpo_gated), .presetn(sys_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_gpio),  .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_gpio),  .prdata(rdata_gpio),  .pslverr(err_gpio),  .gpio_in(gpio_in), .gpio_out(gpio_out), .gpio_dir(gpio_dir), .gpio_irq(irq_gpio));
    
    // --- Accelerator (Dự phòng) ---
    apb_accel ACCEL_INST (.pclk(clk_acc_gated), .presetn(sys_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_accel), .penable(apb_penable), .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pstrb(apb_pstrb), .pready(ready_accel), .prdata(rdata_accel), .pslverr(err_accel), .accel_irq(irq_accel));

endmodule