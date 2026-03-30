`timescale 1ns / 1ps

// =====================================================================
// MOCK SPI FLASH MEMORY (Giữ nguyên như cũ)
// =====================================================================
module mock_spi_flash (
    input  wire cs_n,
    input  wire sck,
    input  wire mosi,
    output reg  miso
);
    reg [7:0] flash_mem [0:255];
    reg [31:0] shift_reg;
    reg [23:0] addr;
    integer bit_cnt;
    integer state; 
    integer i;
    
    initial begin
        for (i = 0; i < 256; i = i + 1) flash_mem[i] = 8'h00;
        flash_mem[0]=8'h93; flash_mem[1]=8'h02; flash_mem[2]=8'h00; flash_mem[3]=8'h00;
        flash_mem[4]=8'h13; flash_mem[5]=8'h03; flash_mem[6]=8'h10; flash_mem[7]=8'h00;
        flash_mem[8]=8'h93; flash_mem[9]=8'h03; flash_mem[10]=8'hb0; flash_mem[11]=8'h00;
        flash_mem[12]=8'hb3; flash_mem[13]=8'h82; flash_mem[14]=8'h62; flash_mem[15]=8'h00;
        flash_mem[16]=8'h13; flash_mem[17]=8'h03; flash_mem[18]=8'h13; flash_mem[19]=8'h00;
        flash_mem[20]=8'he3; flash_mem[21]=8'h6c; flash_mem[22]=8'h73; flash_mem[23]=8'hfe;
        flash_mem[24]=8'h13; flash_mem[25]=8'h0e; flash_mem[26]=8'h70; flash_mem[27]=8'h03;
        flash_mem[28]=8'h63; flash_mem[29]=8'h96; flash_mem[30]=8'hc2; flash_mem[31]=8'h01;
        flash_mem[32]=8'hb7; flash_mem[33]=8'h05; flash_mem[34]=8'h00; flash_mem[35]=8'h40;
        flash_mem[36]=8'h23; flash_mem[37]=8'ha2; flash_mem[38]=8'h55; flash_mem[39]=8'h00;
        flash_mem[40]=8'h37; flash_mem[41]=8'h06; flash_mem[42]=8'h00; flash_mem[43]=8'h80;
        flash_mem[44]=8'h23; flash_mem[45]=8'h20; flash_mem[46]=8'h56; flash_mem[47]=8'h10;
        flash_mem[48]=8'h73; flash_mem[49]=8'h00; flash_mem[50]=8'h00; flash_mem[51]=8'h00;
        bit_cnt = 0; state = 0;
        miso = 1'bz;
    end
    
    always @(negedge cs_n) begin 
        bit_cnt = 0; state = 0; miso = 1'bz;
    end
    
    always @(posedge sck) begin
        if (!cs_n) begin
            shift_reg = {shift_reg[30:0], mosi};
            if (state != 3) bit_cnt = bit_cnt + 1;
            
            if (state == 0 && bit_cnt == 8) begin 
                state = 1;
                bit_cnt = 0; 
            end 
            else if (state == 1 && bit_cnt == 24) begin 
                addr = shift_reg[23:0];
                state = 2; bit_cnt = 0; 
            end
            else if (state == 2 && bit_cnt == 8) begin 
                state = 3;
                bit_cnt = 0; 
            end
        end
    end
    
    always @(negedge sck) begin
        if (!cs_n && state == 3) begin
            miso = flash_mem[addr][7 - bit_cnt];
            if (bit_cnt == 7) begin 
                bit_cnt = 0; addr = addr + 1; 
            end else bit_cnt = bit_cnt + 1;
        end else miso = 1'bz;
    end
endmodule


// =====================================================================
// MAIN TESTBENCH FLOW (TÍCH HỢP MASTER 2 CHO JTAG)
// =====================================================================

// Khai báo kết nối C++ DPI
import "DPI-C" context function void jtag_server_init(input int port);
import "DPI-C" context function void jtag_tick(output byte tms, output byte tdi, output byte tck, input byte tdo);

module tb_jtag();
    reg clk_sys, rst_n;
    initial begin 
        clk_sys = 0; rst_n = 0; #50 rst_n = 1;
    end
    always #5 clk_sys = ~clk_sys; 

    // --- Khai báo Tín hiệu JTAG DPI ---
    reg [7:0] tms_dpi, tdi_dpi, tck_dpi;
    wire tdo_dpi;
    
    initial begin
        jtag_server_init(12345);
        tms_dpi = 0; tdi_dpi = 0;
        tck_dpi = 0;
    end
    
    // Chạy loop DPI mỗi 1ns
    always #1 begin
        jtag_tick(tms_dpi, tdi_dpi, tck_dpi, tdo_dpi);
    end
    
    wire jtag_tck = tck_dpi[0];
    wire jtag_tms = tms_dpi[0];
    wire jtag_tdi = tdi_dpi[0];
    
    // --- Khai báo các bus AXI Master 0 (I), Master 1 (D) --- 
    wire [4:0]  m0_awid, m0_arid, m0_bid, m0_rid;
    wire [31:0] m0_awaddr, m0_araddr, m0_wdata, m0_rdata;
    wire [7:0]  m0_awlen, m0_arlen;
    wire [2:0]  m0_awsize, m0_arsize, m0_awprot, m0_arprot;
    wire [1:0]  m0_awburst, m0_arburst, m0_bresp, m0_rresp;
    wire [3:0]  m0_awcache, m0_arcache, m0_awqos, m0_arqos, m0_awregion, m0_arregion, m0_wstrb;
    wire        m0_awlock, m0_awvalid, m0_awready, m0_wlast, m0_wvalid, m0_wready;
    wire        m0_bvalid, m0_bready, m0_arlock, m0_arvalid, m0_arready, m0_rlast, m0_rvalid, m0_rready;
    
    wire [4:0]  m1_awid, m1_arid, m1_bid, m1_rid;
    wire [31:0] m1_awaddr, m1_araddr, m1_wdata, m1_rdata;
    wire [7:0]  m1_awlen, m1_arlen;
    wire [2:0]  m1_awsize, m1_arsize, m1_awprot, m1_arprot;
    wire [1:0]  m1_awburst, m1_arburst, m1_bresp, m1_rresp;
    wire [3:0]  m1_awcache, m1_arcache, m1_awqos, m1_arqos, m1_awregion, m1_arregion, m1_wstrb;
    wire        m1_awlock, m1_awvalid, m1_awready, m1_wlast, m1_wvalid, m1_wready;
    wire        m1_bvalid, m1_bready, m1_arlock, m1_arvalid, m1_arready, m1_rlast, m1_rvalid, m1_rready;
    
    // --- Khai báo AXI Master 2 (JTAG DTM) ---
    wire [4:0]  m2_awid, m2_arid, m2_bid, m2_rid;
    wire [31:0] m2_awaddr, m2_araddr, m2_wdata, m2_rdata;
    wire [7:0]  m2_awlen, m2_arlen;
    wire [2:0]  m2_awsize, m2_arsize, m2_awprot, m2_arprot;
    wire [1:0]  m2_awburst, m2_arburst, m2_bresp, m2_rresp;
    wire [3:0]  m2_awcache, m2_arcache, m2_awqos, m2_arqos, m2_awregion, m2_arregion, m2_wstrb;
    wire        m2_awlock, m2_awvalid, m2_awready, m2_wlast, m2_wvalid, m2_wready;
    wire        m2_bvalid, m2_bready, m2_arlock, m2_arvalid, m2_arready, m2_rlast, m2_rvalid, m2_rready;
    
    // --- Khai báo các Slaves AXI (s0 -> s3) ---
    wire [6:0]  s0_awid, s0_arid, s0_bid, s0_rid;
    wire [31:0] s0_awaddr, s0_araddr, s0_wdata, s0_rdata; wire [7:0] s0_awlen, s0_arlen; wire [2:0] s0_awsize, s0_arsize, s0_awprot, s0_arprot;
    wire [1:0] s0_awburst, s0_arburst, s0_bresp, s0_rresp; wire [3:0] s0_awcache, s0_arcache, s0_awqos, s0_arqos, s0_awregion, s0_arregion, s0_wstrb;
    wire s0_awlock, s0_awvalid, s0_awready, s0_wlast, s0_wvalid, s0_wready, s0_bvalid, s0_bready, s0_arlock, s0_arvalid, s0_arready, s0_rlast, s0_rvalid, s0_rready;
    
    wire [6:0]  s1_awid, s1_arid, s1_bid, s1_rid; wire [31:0] s1_awaddr, s1_araddr, s1_wdata, s1_rdata; wire [7:0] s1_awlen, s1_arlen;
    wire [2:0] s1_awsize, s1_arsize, s1_awprot, s1_arprot; wire [1:0] s1_awburst, s1_arburst, s1_bresp, s1_rresp;
    wire [3:0] s1_awcache, s1_arcache, s1_awqos, s1_arqos, s1_awregion, s1_arregion, s1_wstrb; wire s1_awlock, s1_awvalid, s1_awready, s1_wlast, s1_wvalid, s1_wready, s1_bvalid, s1_bready, s1_arlock, s1_arvalid, s1_arready, s1_rlast, s1_rvalid, s1_rready;
    
    wire [6:0]  s2_awid, s2_arid, s2_bid, s2_rid; wire [31:0] s2_awaddr, s2_araddr, s2_wdata, s2_rdata; wire [7:0] s2_awlen, s2_arlen;
    wire [2:0] s2_awsize, s2_arsize, s2_awprot, s2_arprot; wire [1:0] s2_awburst, s2_arburst, s2_bresp, s2_rresp;
    wire [3:0] s2_awcache, s2_arcache, s2_awqos, s2_arqos, s2_awregion, s2_arregion, s2_wstrb; wire s2_awlock, s2_awvalid, s2_awready, s2_wlast, s2_wvalid, s2_wready, s2_bvalid, s2_bready, s2_arlock, s2_arvalid, s2_arready, s2_rlast, s2_rvalid, s2_rready;
    
    wire [6:0]  s3_awid, s3_arid, s3_bid, s3_rid; wire [31:0] s3_awaddr, s3_araddr, s3_wdata, s3_rdata; wire [7:0] s3_awlen, s3_arlen;
    wire [2:0] s3_awsize, s3_arsize, s3_awprot, s3_arprot; wire [1:0] s3_awburst, s3_arburst, s3_bresp, s3_rresp;
    wire [3:0] s3_awcache, s3_arcache, s3_awqos, s3_arqos, s3_awregion, s3_arregion, s3_wstrb; wire s3_awlock, s3_awvalid, s3_awready, s3_wlast, s3_wvalid, s3_wready, s3_bvalid, s3_bready, s3_arlock, s3_arvalid, s3_arready, s3_rlast, s3_rvalid, s3_rready;
    
    // --- CPU Core & Memory --- 
    wire [31:0] cpu_ic_addr, cpu_ic_rdata, cpu_dc_addr, cpu_dc_wdata, cpu_dc_rdata;
    wire        cpu_ic_req, cpu_ic_hit, cpu_ic_stall, cpu_dc_rd, cpu_dc_wr, cpu_dc_hit, cpu_dc_stall, cpu_mem_unsigned;
    wire [1:0]  cpu_dc_size;

    wire [31:0] apb_paddr, apb_pwdata, apb_prdata, rdata_gpio, gpio_out;
    wire [3:0]  apb_pstrb;
    wire        apb_psel, apb_penable, apb_pwrite, apb_pready, apb_pslverr, sel_gpio, ready_gpio, err_gpio;
    wire        spi_cs_n, spi_sck, spi_mosi, spi_miso;
    
    // --- JTAG -> DMI -> SBA -> AXI Tín hiệu kết nối ---
    wire        dmi_req_valid, dmi_resp_ready, dmi_resp_valid;
    wire [6:0]  dmi_req_addr;
    wire [31:0] dmi_req_data, dmi_resp_data;
    wire [1:0]  dmi_req_op, dmi_resp_op;
    
    wire        dbg_axi_req, dbg_axi_ack;
    wire [1:0]  dbg_axi_op, dbg_axi_size, dbg_axi_resp;
    wire [31:0] dbg_axi_addr, dbg_axi_wdata, dbg_axi_rdata;

    // --- BỔ SUNG: DÂY KẾT NỐI DEBUG MODULE & CPU CORE ---
    wire        dbg_halt_req;
    wire        dbg_resume_req;
    wire        dbg_step_req;
    wire        dbg_halted;
    wire [4:0]  dbg_reg_read_addr;
    wire [31:0] dbg_reg_read_data;
    wire        dbg_reg_write_en;
    wire [4:0]  dbg_reg_write_addr;
    wire [31:0] dbg_reg_write_data;
    wire        dbg_ndmreset_req;
    wire        sys_rst_n = rst_n & (~dbg_ndmreset_req);

    // =================================================================
    // INSTANTIATE MODULES
    // =================================================================

    // 1. JTAG DTM
    rv_jtag_dtm JTAG_DTM (
        .tck(jtag_tck), .trst_n(rst_n), .tms(jtag_tms), .tdi(jtag_tdi), .tdo(tdo_dpi),
        .dmi_req_valid(dmi_req_valid), .dmi_req_addr(dmi_req_addr), .dmi_req_data(dmi_req_data), .dmi_req_op(dmi_req_op),
        .dmi_resp_ready(dmi_resp_ready), .dmi_resp_valid(dmi_resp_valid), .dmi_resp_data(dmi_resp_data), .dmi_resp_op(dmi_resp_op)
    );
    
    // Khai báo wire 16-bit để nhận tín hiệu từ Output của Debug Module
    wire [15:0] dbg_reg_read_addr_dump;
    wire [15:0] dbg_reg_write_addr_dump;

    // // Trích xuất 5-bit LSB để lái tín hiệu Input vào CPU Core
    // assign dbg_reg_read_addr  = dbg_reg_read_addr_dump[4:0];
    // assign dbg_reg_write_addr = dbg_reg_write_addr_dump[4:0];

    // 2. Debug Module
    rv_debug_module_sba DBG_MODULE (
        .clk_sys(clk_sys), .rst_sys_n(rst_n),
        .dmi_req_valid(dmi_req_valid), .dmi_req_addr(dmi_req_addr), .dmi_req_data(dmi_req_data), .dmi_req_op(dmi_req_op),
        .dmi_resp_ready(dmi_resp_ready), .dmi_resp_valid(dmi_resp_valid), .dmi_resp_data(dmi_resp_data), .dmi_resp_op(dmi_resp_op),
        
        .axi_req(dbg_axi_req), .axi_op(dbg_axi_op), 
        .axi_size(dbg_axi_size), // Đã map port size
        .axi_addr(dbg_axi_addr), .axi_wdata(dbg_axi_wdata), 
        .axi_ack(dbg_axi_ack), .axi_rdata(dbg_axi_rdata), .axi_resp(dbg_axi_resp),
        
        // Map các port điều khiển CPU
        .cpu_halt_req(dbg_halt_req),
        .cpu_resume_req(dbg_resume_req),
        .cpu_halted(dbg_halted),
        
        // Nhận địa chỉ 16-bit từ DM xuất ra
        .cpu_reg_read_addr(dbg_reg_read_addr_dump),  
        .cpu_reg_read_data(dbg_reg_read_data),
        .cpu_reg_write_en(dbg_reg_write_en),
        .cpu_reg_write_addr(dbg_reg_write_addr_dump),
        .cpu_reg_write_data(dbg_reg_write_data),

        .ndmreset_req(dbg_ndmreset_req)
    );
    
    // 3. Debug AXI Master (Master 2)
    dtm_axi_master DTM_AXI_MST (
        .clk_sys(clk_sys), .rst_sys_n(rst_n),
        .i_req(dbg_axi_req), .i_op(dbg_axi_op), .i_size(dbg_axi_size), .i_addr(dbg_axi_addr), .i_wdata(dbg_axi_wdata),
        .o_ack(dbg_axi_ack), .o_resp(dbg_axi_resp), .o_rdata(dbg_axi_rdata),
        
        // Cắm thẳng vào các port m2_ của AXI
        .m_axi_awid(m2_awid), .m_axi_awaddr(m2_awaddr), .m_axi_awlen(m2_awlen), .m_axi_awsize(m2_awsize), .m_axi_awburst(m2_awburst), .m_axi_awlock(m2_awlock), .m_axi_awcache(m2_awcache), .m_axi_awprot(m2_awprot), .m_axi_awqos(m2_awqos), .m_axi_awregion(m2_awregion), .m_axi_awvalid(m2_awvalid), .m_axi_awready(m2_awready),
        .m_axi_wdata(m2_wdata), .m_axi_wstrb(m2_wstrb), .m_axi_wlast(m2_wlast), .m_axi_wvalid(m2_wvalid), .m_axi_wready(m2_wready),
        .m_axi_bid(m2_bid), .m_axi_bresp(m2_bresp), .m_axi_bvalid(m2_bvalid), .m_axi_bready(m2_bready),
        .m_axi_arid(m2_arid), .m_axi_araddr(m2_araddr), .m_axi_arlen(m2_arlen), .m_axi_arsize(m2_arsize), .m_axi_arburst(m2_arburst), .m_axi_arlock(m2_arlock), .m_axi_arcache(m2_arcache), .m_axi_arprot(m2_arprot), .m_axi_arqos(m2_arqos), .m_axi_arregion(m2_arregion), .m_axi_arvalid(m2_arvalid), .m_axi_arready(m2_arready),
        .m_axi_rid(m2_rid), .m_axi_rdata(m2_rdata), .m_axi_rresp(m2_rresp), .m_axi_rlast(m2_rlast), .m_axi_rvalid(m2_rvalid), .m_axi_rready(m2_rready)
    );
    
    // 4. CPU CORE 
    riscv_pipeline CPU_CORE (
        .clk(clk_sys), .reset_n(sys_rst_n), .riscv_start(1'b1),
        .meip_i(1'b0), .msip_i(1'b0), .mtip_i(1'b0),
        .reset_vector_in(32'h0001_0000), 
        .riscv_done(), .wfi_sleep_out(),
        
        .icache_read_req(cpu_ic_req), .icache_addr(cpu_ic_addr), .icache_read_data(cpu_ic_rdata), .icache_hit(cpu_ic_hit), .icache_stall(cpu_ic_stall),
        .dcache_read_req(cpu_dc_rd), .dcache_write_req(cpu_dc_wr), .dcache_addr(cpu_dc_addr), .dcache_write_data(cpu_dc_wdata), .dcache_read_data(cpu_dc_rdata), .dcache_hit(cpu_dc_hit), .dcache_stall(cpu_dc_stall),
        .mem_size_top(cpu_dc_size), .mem_unsigned_top(cpu_mem_unsigned),
        
        .dbg_halt_req(dbg_halt_req),
        .dbg_resume_req(dbg_resume_req),
        .dbg_step_req(dbg_step_req),
        .dbg_halted(dbg_halted),
        .dbg_reg_read_addr(dbg_reg_read_addr_dump),
        .dbg_reg_read_data(dbg_reg_read_data),
        .dbg_reg_write_en(dbg_reg_write_en),
        .dbg_reg_write_addr(dbg_reg_write_addr_dump),
        .dbg_reg_write_data(dbg_reg_write_data)
    );
    
    // 5. CACHES
    wire ic_uncache_en = (cpu_ic_addr >= 32'h4000_0000 && cpu_ic_addr <= 32'h4FFF_FFFF);
    
    instruction_cache I_CACHE (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .cpu_read_req(cpu_ic_req), .cpu_addr(cpu_ic_addr), .uncache_en(ic_uncache_en), .cpu_read_data(cpu_ic_rdata), .icache_hit(cpu_ic_hit), .icache_stall(cpu_ic_stall),
        // Connect to m0
        .m_axi_awid(m0_awid), .m_axi_awaddr(m0_awaddr), .m_axi_awlen(m0_awlen), .m_axi_awsize(m0_awsize), .m_axi_awburst(m0_awburst), .m_axi_awlock(m0_awlock), .m_axi_awcache(m0_awcache), .m_axi_awprot(m0_awprot), .m_axi_awqos(m0_awqos), .m_axi_awregion(m0_awregion), .m_axi_awvalid(m0_awvalid), .m_axi_awready(m0_awready),
        .m_axi_wdata(m0_wdata), .m_axi_wstrb(m0_wstrb), .m_axi_wlast(m0_wlast), .m_axi_wvalid(m0_wvalid), .m_axi_wready(m0_wready),
        .m_axi_bid(m0_bid), .m_axi_bresp(m0_bresp), .m_axi_bvalid(m0_bvalid), .m_axi_bready(m0_bready),
        .m_axi_arid(m0_arid), .m_axi_araddr(m0_araddr), .m_axi_arlen(m0_arlen), .m_axi_arsize(m0_arsize), .m_axi_arburst(m0_arburst), .m_axi_arlock(m0_arlock), .m_axi_arcache(m0_arcache), .m_axi_arprot(m0_arprot), .m_axi_arqos(m0_arqos), .m_axi_arregion(m0_arregion), .m_axi_arvalid(m0_arvalid), .m_axi_arready(m0_arready),
        .m_axi_rid(m0_rid), .m_axi_rdata(m0_rdata), .m_axi_rresp(m0_rresp), .m_axi_rlast(m0_rlast), .m_axi_rvalid(m0_rvalid), .m_axi_rready(m0_rready)
    );
    
    wire dc_uncache_en = (cpu_dc_addr >= 32'h4000_0000 && cpu_dc_addr <= 32'h4FFF_FFFF);
    
    data_cache D_CACHE (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .cpu_read_req(cpu_dc_rd), .cpu_write_req(cpu_dc_wr), .cpu_addr(cpu_dc_addr), .cpu_write_data(cpu_dc_wdata),
        .mem_unsigned(cpu_mem_unsigned), .mem_size(cpu_dc_size), .uncache_en(dc_uncache_en), .cpu_read_data(cpu_dc_rdata), .dcache_hit(cpu_dc_hit), .dcache_stall(cpu_dc_stall),
        // Connect to m1
        .m_axi_awid(m1_awid), .m_axi_awaddr(m1_awaddr), .m_axi_awlen(m1_awlen), .m_axi_awsize(m1_awsize), .m_axi_awburst(m1_awburst), .m_axi_awlock(m1_awlock), .m_axi_awcache(m1_awcache), .m_axi_awprot(m1_awprot), .m_axi_awqos(m1_awqos), .m_axi_awregion(m1_awregion), .m_axi_awvalid(m1_awvalid), .m_axi_awready(m1_awready),
        .m_axi_wdata(m1_wdata), .m_axi_wstrb(m1_wstrb), .m_axi_wlast(m1_wlast), .m_axi_wvalid(m1_wvalid), .m_axi_wready(m1_wready),
        .m_axi_bid(m1_bid), .m_axi_bresp(m1_bresp), .m_axi_bvalid(m1_bvalid), .m_axi_bready(m1_bready),
        .m_axi_arid(m1_arid), .m_axi_araddr(m1_araddr), .m_axi_arlen(m1_arlen), .m_axi_arsize(m1_arsize), 
        .m_axi_arburst(m1_arburst), .m_axi_arlock(m1_arlock), .m_axi_arcache(m1_arcache), .m_axi_arprot(m1_arprot), .m_axi_arqos(m1_arqos), .m_axi_arregion(m1_arregion), .m_axi_arvalid(m1_arvalid), .m_axi_arready(m1_arready),
        .m_axi_rid(m1_rid), .m_axi_rdata(m1_rdata), .m_axi_rresp(m1_rresp), .m_axi_rlast(m1_rlast), .m_axi_rvalid(m1_rvalid), .m_axi_rready(m1_rready)
    );
    
    // 6. MAIN BUS AXI INTERCONNECT (3 MASTERS)
    axi_interconnect #(
        .MST_AMT(3), // <-- TĂNG LÊN 3 CHO JTAG
        .SLV_AMT(4), 
        .DATA_WIDTH(32), .ADDR_WIDTH(32),
        .SLV_BASE_ADDR({32'h5000_0000, 32'h8000_0000, 32'h4000_0000, 32'h0001_0000}),
        .SLV_ADDR_MASK({32'hFF00_0000, 32'hFFFF_0000, 32'hFFFF_0000, 32'hFFFF_F000})
    ) MAIN_BUS (
        .ACLK_i(clk_sys), .ARESETn_i(sys_rst_n),
        // Nối m2 vào bus
        .m_AWID_i({m2_awid, m1_awid, m0_awid}), .m_AWADDR_i({m2_awaddr, m1_awaddr, m0_awaddr}), .m_AWBURST_i({m2_awburst, m1_awburst, m0_awburst}), .m_AWLEN_i({m2_awlen, m1_awlen, m0_awlen}), .m_AWSIZE_i({m2_awsize, m1_awsize, m0_awsize}), .m_AWLOCK_i({m2_awlock, m1_awlock, m0_awlock}), .m_AWCACHE_i({m2_awcache, m1_awcache, m0_awcache}), .m_AWPROT_i({m2_awprot, m1_awprot, m0_awprot}), .m_AWQOS_i({m2_awqos, m1_awqos, m0_awqos}), .m_AWREGION_i({m2_awregion, m1_awregion, m0_awregion}), .m_AWVALID_i({m2_awvalid, m1_awvalid, m0_awvalid}), .m_AWREADY_o({m2_awready, m1_awready, m0_awready}),
        .m_WDATA_i({m2_wdata, m1_wdata, m0_wdata}), .m_WSTRB_i({m2_wstrb, m1_wstrb, m0_wstrb}), .m_WLAST_i({m2_wlast, m1_wlast, m0_wlast}), .m_WVALID_i({m2_wvalid, m1_wvalid, m0_wvalid}), .m_WREADY_o({m2_wready, m1_wready, m0_wready}),
        .m_BID_o({m2_bid, m1_bid, m0_bid}), .m_BRESP_o({m2_bresp, m1_bresp, m0_bresp}), .m_BVALID_o({m2_bvalid, m1_bvalid, m0_bvalid}), .m_BREADY_i({m2_bready, m1_bready, m0_bready}),
        .m_ARID_i({m2_arid, m1_arid, m0_arid}), .m_ARADDR_i({m2_araddr, m1_araddr, m0_araddr}), .m_ARBURST_i({m2_arburst, m1_arburst, m0_arburst}), .m_ARLEN_i({m2_arlen, m1_arlen, m0_arlen}), .m_ARSIZE_i({m2_arsize, m1_arsize, m0_arsize}), .m_ARLOCK_i({m2_arlock, m1_arlock, m0_arlock}), .m_ARCACHE_i({m2_arcache, m1_arcache, m0_arcache}), .m_ARPROT_i({m2_arprot, m1_arprot, m0_arprot}), .m_ARQOS_i({m2_arqos, m1_arqos, m0_arqos}), .m_ARREGION_i({m2_arregion, m1_arregion, m0_arregion}), .m_ARVALID_i({m2_arvalid, m1_arvalid, m0_arvalid}), .m_ARREADY_o({m2_arready, m1_arready, m0_arready}),
        .m_RID_o({m2_rid, m1_rid, m0_rid}), .m_RDATA_o({m2_rdata, m1_rdata, m0_rdata}), .m_RRESP_o({m2_rresp, m1_rresp, m0_rresp}), .m_RLAST_o({m2_rlast, m1_rlast, m0_rlast}), .m_RVALID_o({m2_rvalid, m1_rvalid, m0_rvalid}), .m_RREADY_i({m2_rready, m1_rready, m0_rready}),
        
        // SLAVE PORTs GIỮ NGUYÊN
        .s_AWID_o({s3_awid, s2_awid, s1_awid, s0_awid}), .s_AWADDR_o({s3_awaddr, s2_awaddr, s1_awaddr, s0_awaddr}), .s_AWBURST_o({s3_awburst, s2_awburst, s1_awburst, s0_awburst}), .s_AWLEN_o({s3_awlen, s2_awlen, s1_awlen, s0_awlen}), .s_AWSIZE_o({s3_awsize, s2_awsize, s1_awsize, s0_awsize}), .s_AWLOCK_o({s3_awlock, s2_awlock, s1_awlock, s0_awlock}), .s_AWCACHE_o({s3_awcache, s2_awcache, s1_awcache, s0_awcache}), .s_AWPROT_o({s3_awprot, s2_awprot, s1_awprot, s0_awprot}), .s_AWQOS_o({s3_awqos, s2_awqos, s1_awqos, s0_awqos}), .s_AWREGION_o({s3_awregion, s2_awregion, s1_awregion, s0_awregion}), .s_AWVALID_o({s3_awvalid, s2_awvalid, s1_awvalid, s0_awvalid}), .s_AWREADY_i({s3_awready, s2_awready, s1_awready, s0_awready}),
        .s_WDATA_o({s3_wdata, s2_wdata, s1_wdata, s0_wdata}), .s_WSTRB_o({s3_wstrb, s2_wstrb, s1_wstrb, s0_wstrb}), .s_WLAST_o({s3_wlast, s2_wlast, s1_wlast, s0_wlast}), .s_WVALID_o({s3_wvalid, s2_wvalid, s1_wvalid, s0_wvalid}), .s_WREADY_i({s3_wready, s2_wready, s1_wready, s0_wready}),
        .s_BID_i({s3_bid, s2_bid, s1_bid, s0_bid}), .s_BRESP_i({s3_bresp, s2_bresp, s1_bresp, s0_bresp}), .s_BVALID_i({s3_bvalid, s2_bvalid, s1_bvalid, s0_bvalid}), .s_BREADY_o({s3_bready, s2_bready, s1_bready, s0_bready}),
        .s_ARID_o({s3_arid, s2_arid, s1_arid, s0_arid}), .s_ARADDR_o({s3_araddr, s2_araddr, s1_araddr, s0_araddr}), .s_ARBURST_o({s3_arburst, s2_arburst, s1_arburst, s0_arburst}), .s_ARLEN_o({s3_arlen, s2_arlen, s1_arlen, s0_arlen}), .s_ARSIZE_o({s3_arsize, s2_arsize, s1_arsize, s0_arsize}), .s_ARLOCK_o({s3_arlock, s2_arlock, s1_arlock, s0_arlock}), .s_ARCACHE_o({s3_arcache, s2_arcache, s1_arcache, s0_arcache}), .s_ARPROT_o({s3_arprot, s2_arprot, s1_arprot, s0_arprot}), .s_ARQOS_o({s3_arqos, s2_arqos, s1_arqos, s0_arqos}), .s_ARREGION_o({s3_arregion, s2_arregion, s1_arregion, s0_arregion}), .s_ARVALID_o({s3_arvalid, s2_arvalid, s1_arvalid, s0_arvalid}), .s_ARREADY_i({s3_arready, s2_arready, s1_arready, s0_arready}),
        .s_RID_i({s3_rid, s2_rid, s1_rid, s0_rid}), .s_RDATA_i({s3_rdata, s2_rdata, s1_rdata, s0_rdata}), .s_RRESP_i({s3_rresp, s2_rresp, s1_rresp, s0_rresp}), .s_RLAST_i({s3_rlast, s2_rlast, s1_rlast, s0_rlast}), .s_RVALID_i({s3_rvalid, s2_rvalid, s1_rvalid, s0_rvalid}), .s_RREADY_o({s3_rready, s2_rready, s1_rready, s0_rready})
    );
    
    // =================================================================
    // CÁC PERIPHERAL VÀ BỘ NHỚ GIỮ NGUYÊN (ROM, RAM, APB, FLASH...)
    // =================================================================
    axi_rom #(.ID_WIDTH(7), .ADDR_MASK(32'h0000_0FFF)) BOOT_ROM_INST (
        .clk(clk_sys), .rst_n(sys_rst_n), 
        .s_axi_awid(s0_awid), .s_axi_awaddr(s0_awaddr), .s_axi_awlen(s0_awlen), .s_axi_awsize(s0_awsize), .s_axi_awburst(s0_awburst), .s_axi_awlock(s0_awlock), .s_axi_awcache(s0_awcache), .s_axi_awprot(s0_awprot), .s_axi_awqos(s0_awqos), .s_axi_awregion(s0_awregion), .s_axi_awvalid(s0_awvalid), .s_axi_awready(s0_awready),
        .s_axi_wdata(s0_wdata), .s_axi_wstrb(s0_wstrb), .s_axi_wlast(s0_wlast), .s_axi_wvalid(s0_wvalid), .s_axi_wready(s0_wready), 
        .s_axi_bid(s0_bid), .s_axi_bresp(s0_bresp), .s_axi_bvalid(s0_bvalid), .s_axi_bready(s0_bready),
        .s_axi_arid(s0_arid), .s_axi_araddr(s0_araddr), .s_axi_arlen(s0_arlen), .s_axi_arsize(s0_arsize), .s_axi_arburst(s0_arburst), .s_axi_arlock(s0_arlock), .s_axi_arcache(s0_arcache), .s_axi_arprot(s0_arprot), .s_axi_arqos(s0_arqos), .s_axi_arregion(s0_arregion), .s_axi_arvalid(s0_arvalid), .s_axi_arready(s0_arready), 
        .s_axi_rid(s0_rid), .s_axi_rdata(s0_rdata), .s_axi_rresp(s0_rresp), .s_axi_rlast(s0_rlast), .s_axi_rvalid(s0_rvalid), .s_axi_rready(s0_rready)
    );
    
    axi_to_apb_bridge #(.ID_WIDTH(7)) APB_BRIDGE (
        .clk(clk_sys), .rst_n(sys_rst_n), 
        .s_axi_awid(s1_awid), .s_axi_awaddr(s1_awaddr), .s_axi_awlen(s1_awlen), .s_axi_awsize(s1_awsize), .s_axi_awburst(s1_awburst), .s_axi_awprot(s1_awprot), .s_axi_awvalid(s1_awvalid), .s_axi_awready(s1_awready), 
        .s_axi_wdata(s1_wdata), .s_axi_wstrb(s1_wstrb), .s_axi_wlast(s1_wlast), .s_axi_wvalid(s1_wvalid), .s_axi_wready(s1_wready), 
        .s_axi_bid(s1_bid), .s_axi_bresp(s1_bresp), .s_axi_bvalid(s1_bvalid), .s_axi_bready(s1_bready),
        .s_axi_arid(s1_arid), .s_axi_araddr(s1_araddr), .s_axi_arlen(s1_arlen), .s_axi_arsize(s1_arsize), .s_axi_arburst(s1_arburst), .s_axi_arprot(s1_arprot), .s_axi_arvalid(s1_arvalid), .s_axi_arready(s1_arready), 
        .s_axi_rid(s1_rid), .s_axi_rdata(s1_rdata), .s_axi_rresp(s1_rresp), .s_axi_rlast(s1_rlast), .s_axi_rvalid(s1_rvalid), .s_axi_rready(s1_rready),
        .m_apb_paddr(apb_paddr), .m_apb_pprot(), .m_apb_psel(apb_psel), .m_apb_penable(apb_penable), .m_apb_pwrite(apb_pwrite), .m_apb_pwdata(apb_pwdata), .m_apb_pstrb(apb_pstrb), .m_apb_pready(apb_pready), .m_apb_prdata(apb_prdata), .m_apb_pslverr(apb_pslverr)
    );

    apb_interconnect APB_BUS (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .m_paddr(apb_paddr), .m_pprot(3'b0), .m_psel(apb_psel), .m_penable(apb_penable), .m_pwrite(apb_pwrite), .m_pwdata(apb_pwdata), .m_pstrb(apb_pstrb), .m_prdata(apb_prdata), .m_pready(apb_pready), .m_pslverr(apb_pslverr),
        .s0_paddr(), .s0_pprot(), .s0_psel(sel_gpio), .s0_penable(), .s0_pwrite(), .s0_pwdata(), .s0_pstrb(), .s0_prdata(rdata_gpio), .s0_pready(ready_gpio), .s0_pslverr(err_gpio),
        .s1_paddr(), .s1_pprot(), .s1_psel(), .s1_penable(), .s1_pwrite(), .s1_pwdata(), .s1_pstrb(), .s1_prdata(32'h0), .s1_pready(1'b1), .s1_pslverr(1'b0),
        .s2_paddr(), .s2_pprot(), .s2_psel(), .s2_penable(), .s2_pwrite(), .s2_pwdata(), .s2_pstrb(), .s2_prdata(32'h0), .s2_pready(1'b1), .s2_pslverr(1'b0),
        .s3_paddr(), .s3_pprot(), .s3_psel(), .s3_penable(), .s3_pwrite(), .s3_pwdata(), .s3_pstrb(), .s3_prdata(32'h0), .s3_pready(1'b1), .s3_pslverr(1'b0)
    );

    apb_gpio GPIO_INST (
        .pclk(clk_sys), .presetn(sys_rst_n), .paddr(apb_paddr[11:0]), .psel(sel_gpio), .penable(apb_penable), 
        .pwrite(apb_pwrite), .pwdata(apb_pwdata), .pready(ready_gpio), .prdata(rdata_gpio), .pslverr(err_gpio), 
        .gpio_in(32'h0), .gpio_out(gpio_out), .gpio_dir(), .gpio_irq()
    );
    
    axi_ram #(.ID_WIDTH(7), .ADDR_MASK(32'h0000_FFFF)) SYSTEM_RAM_INST (
        .clk(clk_sys), .rst_n(sys_rst_n), 
        .s_axi_awid(s2_awid), .s_axi_awaddr(s2_awaddr), .s_axi_awlen(s2_awlen), .s_axi_awsize(s2_awsize), .s_axi_awburst(s2_awburst), .s_axi_awlock(s2_awlock), .s_axi_awcache(s2_awcache), .s_axi_awprot(s2_awprot), .s_axi_awqos(s2_awqos), .s_axi_awregion(s2_awregion), .s_axi_awvalid(s2_awvalid), .s_axi_awready(s2_awready), 
        .s_axi_wdata(s2_wdata), .s_axi_wstrb(s2_wstrb), .s_axi_wlast(s2_wlast), .s_axi_wvalid(s2_wvalid), .s_axi_wready(s2_wready), 
        .s_axi_bid(s2_bid), .s_axi_bresp(s2_bresp), .s_axi_bvalid(s2_bvalid), .s_axi_bready(s2_bready),
        .s_axi_arid(s2_arid), .s_axi_araddr(s2_araddr), .s_axi_arlen(s2_arlen), .s_axi_arsize(s2_arsize), .s_axi_arburst(s2_arburst), .s_axi_arlock(s2_arlock), .s_axi_arcache(s2_arcache), .s_axi_arprot(s2_arprot), .s_axi_arqos(s2_arqos), .s_axi_arregion(s2_arregion), .s_axi_arvalid(s2_arvalid), .s_axi_arready(s2_arready), 
        .s_axi_rid(s2_rid), .s_axi_rdata(s2_rdata), .s_axi_rresp(s2_rresp), .s_axi_rlast(s2_rlast), .s_axi_rvalid(s2_rvalid), .s_axi_rready(s2_rready)
    );
    
    axi_spi_flash #(.ID_WIDTH(7), .ADDR_MASK(32'h00FF_FFFF)) SPI_FLASH_INST (
        .clk(clk_sys), .rst_n(sys_rst_n), 
        .s_axi_awid(s3_awid), .s_axi_awaddr(s3_awaddr), .s_axi_awlen(s3_awlen), .s_axi_awsize(s3_awsize), .s_axi_awburst(s3_awburst), .s_axi_awvalid(s3_awvalid), .s_axi_awready(s3_awready), 
        .s_axi_wdata(s3_wdata), .s_axi_wstrb(s3_wstrb), .s_axi_wlast(s3_wlast), .s_axi_wvalid(s3_wvalid), .s_axi_wready(s3_wready), 
        .s_axi_bid(s3_bid), .s_axi_bresp(s3_bresp), .s_axi_bvalid(s3_bvalid), .s_axi_bready(s3_bready),
        .s_axi_arid(s3_arid), .s_axi_araddr(s3_araddr), .s_axi_arlen(s3_arlen), .s_axi_arsize(s3_arsize), .s_axi_arburst(s3_arburst), .s_axi_arvalid(s3_arvalid), .s_axi_arready(s3_arready), 
        .s_axi_rid(s3_rid), .s_axi_rdata(s3_rdata), .s_axi_rresp(s3_rresp), .s_axi_rlast(s3_rlast), .s_axi_rvalid(s3_rvalid), .s_axi_rready(s3_rready),
        .spi_cs_n_o(spi_cs_n), .spi_clk_o(spi_sck), .spi_io0_o(spi_mosi), .spi_io0_i(1'b0), .spi_io0_oe(), .spi_io1_o(), .spi_io1_i(spi_miso), .spi_io1_oe(), .spi_io2_o(), .spi_io2_i(1'b1), .spi_io2_oe(), 
        .spi_io3_o(), .spi_io3_i(1'b1), .spi_io3_oe()
    );

    mock_spi_flash EXT_FLASH (.cs_n(spi_cs_n), .sck(spi_sck), .mosi(spi_mosi), .miso(spi_miso));
    
    initial begin
        BOOT_ROM_INST.rom_memory[0] = 32'h500002b7; // lui t0, 0x50000
        BOOT_ROM_INST.rom_memory[1] = 32'h80000337; // lui t1, 0x80000
        BOOT_ROM_INST.rom_memory[2] = 32'h04030393; // addi t2, t1, 64
        BOOT_ROM_INST.rom_memory[3] = 32'h0002ae03; // lw t3, 0(t0)
        BOOT_ROM_INST.rom_memory[4] = 32'h01c32023; // sw t3, 0(t1)
        BOOT_ROM_INST.rom_memory[5] = 32'h00428293; // addi t0, t0, 4
        BOOT_ROM_INST.rom_memory[6] = 32'h00430313; // addi t1, t1, 4
        BOOT_ROM_INST.rom_memory[7] = 32'hfe7368e3; // bltu t1, t2, -16
        BOOT_ROM_INST.rom_memory[8] = 32'h80000337; // lui t1, 0x80000
        BOOT_ROM_INST.rom_memory[9] = 32'h00030067; // jalr x0, t1, 0
    end

    // =================================================================
    // SIMULATION FLOW
    // =================================================================
    integer phase;
    
    initial begin
        $display("===============================================================");
        $display("   STARTING SOC MAIN FLOW TEST WITH JTAG & OPENOCD             ");
        $display("===============================================================");
        
        phase = 1; $display("[PHASE 1] System Reset...");
        wait(sys_rst_n == 1); #100;
        $display("          [PASS] Reset Released.");
        
        phase = 2;
        $display("[PHASE 2] Waiting for CPU to finish BootROM...");
        wait(CPU_CORE.pc_reg >= 32'h8000_0000);
        $display("          [PASS] Boot successful! Jumped to Main RAM.");
        
        // Chạy vô tận để bạn có thời gian tương tác qua OpenOCD.
        wait(CPU_CORE.pc_reg == 32'h8000_0030);
        // Ecall ở addr 0x30 của RAM
        $display("          [DONE] Testbench reached ECALL. Simulation stays alive for OpenOCD.");
    end
endmodule