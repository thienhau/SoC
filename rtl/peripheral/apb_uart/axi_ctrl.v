module axi4lite_ctrl #(
    parameter AXI4_CTRL_CONF     = 1,
    parameter AXI4_CTRL_STAT     = 0,
    parameter AXI4_CTRL_MEM      = 0,
    parameter AXI4_CTRL_WR_ST    = 1,
    parameter AXI4_CTRL_RD_ST    = 1,
    
    // AXI4-Lite compliance options
    parameter CHECK_ALIGNMENT    = 1,          // Enable address alignment check
    parameter CHECK_PROTECTION   = 0,          // Enable AWPROT/ARPROT checking
    parameter PRIVILEGED_ONLY    = 0,          // Only allow privileged access

    parameter CONF_BASE_ADDR     = 32'h2000_0000,
    parameter CONF_OFFSET        = 32'h04,          // AXI addr là byte-address; 32-bit reg => stride 4
    parameter CONF_REG_NUM       = 2,

    parameter ST_WR_BASE_ADDR    = 32'h2000_0010,
    parameter ST_WR_OFFSET       = 32'h04,
    parameter ST_WR_FIFO_NUM     = 1,

    parameter ST_RD_BASE_ADDR    = 32'h2000_0020,
    parameter ST_RD_OFFSET       = 32'h04,
    parameter ST_RD_FIFO_NUM     = 1,

    parameter DATA_W             = 32,
    parameter ADDR_W             = 32,
    parameter TRANS_RESP_W       = 2,
    
    // Timeout for blocking reads (in clock cycles)
    parameter READ_TIMEOUT       = 16'hFFFF
) (
    input                          clk,
    input                          rst_n,

    // AXI4-Lite Slave Interface
    // Write address
    input  [ADDR_W-1:0]            s_axi_awaddr,
    input  [2:0]                   s_axi_awprot,
    input                          s_axi_awvalid,
    output                         s_axi_awready,
    // Write data
    input  [DATA_W-1:0]            s_axi_wdata,
    input  [(DATA_W/8)-1:0]        s_axi_wstrb,
    input                          s_axi_wvalid,
    output                         s_axi_wready,
    // Write response
    output reg [TRANS_RESP_W-1:0]  s_axi_bresp,
    output reg                     s_axi_bvalid,
    input                          s_axi_bready,
    // Read address
    input  [ADDR_W-1:0]            s_axi_araddr,
    input  [2:0]                   s_axi_arprot,
    input                          s_axi_arvalid,
    output                         s_axi_arready,
    // Read data
    output reg [DATA_W-1:0]        s_axi_rdata,
    output reg [TRANS_RESP_W-1:0]  s_axi_rresp,
    output reg                     s_axi_rvalid,
    input                          s_axi_rready,

    // External Interfaces
    input  [DATA_W-1:0]            stat_reg_i,
    input                          mem_wr_rdy_i,
    input  [DATA_W-1:0]            mem_rd_data_i,
    input                          mem_rd_rdy_i,

    // TX stream sink ready 
    input  [ST_WR_FIFO_NUM-1:0]    wr_st_can_accept_i,

    // RX stream source
    input  [DATA_W-1:0]            rd_st_data_i,
    input  [ST_RD_FIFO_NUM-1:0]    rd_st_has_data_i,

    // Outputs
    output reg [DATA_W*CONF_REG_NUM-1:0] conf_reg_o,
    output reg [DATA_W-1:0]        mem_wr_data_o,
    output reg [ADDR_W-1:0]        mem_wr_addr_o,
    output reg [(DATA_W/8)-1:0]    mem_wr_strb_o,     // WSTRB for memory write
    output reg                     mem_wr_vld_o,
    output reg [ADDR_W-1:0]        mem_rd_addr_o,
    output reg                     mem_rd_vld_o,

    // Stream Interfaces
    output reg [DATA_W-1:0]        wr_st_data_o,
    output reg [ST_WR_FIFO_NUM-1:0] wr_st_push_o,     // pulse when pushing
    output reg [ST_RD_FIFO_NUM-1:0] rd_st_pop_o       // pulse when popping
);

    localparam integer STRB_W = (DATA_W/8);
    
    // Address alignment check (AXI4-Lite requirement)
    // 32-bit data width requires 4-byte alignment
    localparam integer ALIGN_BITS = $clog2(DATA_W/8);

    // Indexing variables with explicit width to avoid synthesis warnings
    integer i;  // Loop counter
    reg [7:0] wr_index;  // Write index (max 256 registers/FIFOs)
    reg [7:0] rd_index;  // Read index
    
    // Timeout counter for blocking reads
    reg [15:0] rd_timeout_cnt;

    // 1-entry "skid buffers" to decouple AW and W (AXI allows re-order) buffering
    reg               aw_hold;
    reg [ADDR_W-1:0]  awaddr_hold;

    reg               w_hold;
    reg [DATA_W-1:0]  wdata_hold;
    reg [STRB_W-1:0]  wstrb_hold;

    wire aw_hs = s_axi_awvalid && s_axi_awready;
    wire  w_hs = s_axi_wvalid  && s_axi_wready;

    // We can accept AW if we are not holding one and not busy with BVALID
    assign s_axi_awready = (!aw_hold) && (!s_axi_bvalid);

    // We can accept W if we are not holding one and not busy with BVALID
    // Backpressure strategy: 
    // - If AW already received: decode address and check if writing to stream
    // - If AW not yet received: optimistically accept W (cannot decode address yet)
    wire [ADDR_W-1:0] wr_addr = aw_hold ? awaddr_hold : s_axi_awaddr;

    wire wr_to_stream = AXI4_CTRL_WR_ST && 
                    (wr_addr >= ST_WR_BASE_ADDR) &&
                    (wr_addr < (ST_WR_BASE_ADDR + ST_WR_OFFSET*ST_WR_FIFO_NUM));
    wire wr_st_ok = (ST_WR_FIFO_NUM == 0) ? 1'b1 : (|wr_st_can_accept_i);
    
    // Check if AW channel has valid address (either held or on bus)
    wire aw_is_valid = aw_hold || s_axi_awvalid;
    
    // Can accept W when: not holding W data AND not busy with response AND destination ready
    // When AW not yet valid (W arrives before AW), accept W into buffer without checking destination
    // When AW is valid, check if destination (stream) can accept
    wire can_take_w = (!w_hold) && (!s_axi_bvalid) && 
                      (!aw_is_valid || !wr_to_stream || wr_st_ok);

    assign s_axi_wready = can_take_w;

    // Address decode (byte addressing)
    // AXI4-Lite compliance: Check alignment
// 1. Kiểm tra căn lề (Đã OK)
    wire wr_addr_aligned = CHECK_ALIGNMENT ? (awaddr_hold[ALIGN_BITS-1:0] == {ALIGN_BITS{1'b0}}) : 1'b1;
    wire rd_addr_aligned = CHECK_ALIGNMENT ? (araddr_hold[ALIGN_BITS-1:0] == {ALIGN_BITS{1'b0}}) : 1'b1;
    
    // 2. Logic Write (Giữ nguyên, cái này đã chạy đúng)
    wire wr_conf_sel = (awaddr_hold >= CONF_BASE_ADDR) &&
                       (awaddr_hold <  (CONF_BASE_ADDR + CONF_OFFSET*CONF_REG_NUM));

    wire wr_st_sel   = (awaddr_hold >= ST_WR_BASE_ADDR) &&
                       (awaddr_hold <  (ST_WR_BASE_ADDR + ST_WR_OFFSET*ST_WR_FIFO_NUM));
    
    wire wr_addr_valid = (AXI4_CTRL_CONF && wr_conf_sel) ||
                         (AXI4_CTRL_WR_ST && wr_st_sel) ||
                         (AXI4_CTRL_MEM);
    
    wire rd_conf_sel = (araddr_hold >= CONF_BASE_ADDR) && 
                       (araddr_hold <  (CONF_BASE_ADDR + CONF_OFFSET*CONF_REG_NUM));

    wire rd_st_sel   = (araddr_hold >= ST_RD_BASE_ADDR) && 
                       (araddr_hold <  (ST_RD_BASE_ADDR + ST_RD_OFFSET*ST_RD_FIFO_NUM));
    // Detect unmapped read addresses
    wire rd_addr_valid = (AXI4_CTRL_CONF && rd_conf_sel) ||
                         (AXI4_CTRL_RD_ST && rd_st_sel) ||
                         (AXI4_CTRL_STAT) ||
                         (AXI4_CTRL_MEM);

    // Write commit: only when we have BOTH AW and W, and no pending BVALID
    // Must generate exactly 1 write response per write (Lite = single-beat)
    // BVALID must stay high until BREADY (spec)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_bvalid   <= 1'b0;
            s_axi_bresp    <= 2'b00;
            conf_reg_o     <= {DATA_W*CONF_REG_NUM{1'b0}};
            mem_wr_vld_o   <= 1'b0;
            mem_wr_addr_o  <= {ADDR_W{1'b0}};
            mem_wr_data_o  <= {DATA_W{1'b0}};
            mem_wr_strb_o  <= {STRB_W{1'b0}};
            wr_st_push_o   <= {ST_WR_FIFO_NUM{1'b0}};
            wr_st_data_o   <= {DATA_W{1'b0}};
            aw_hold        <= 1'b0;
            w_hold         <= 1'b0;
            awaddr_hold    <= {ADDR_W{1'b0}};
            wdata_hold     <= {DATA_W{1'b0}};
            wstrb_hold     <= {STRB_W{1'b0}};
        end else begin
            // defaults: pulses low
            mem_wr_vld_o <= 1'b0;
            wr_st_push_o <= {ST_WR_FIFO_NUM{1'b0}};

            // Latch AW when handshake occurs
            if (aw_hs) begin
                aw_hold     <= 1'b1;
                awaddr_hold <= s_axi_awaddr;
            end

            // Latch W when handshake occurs
            if (w_hs) begin
                w_hold     <= 1'b1;
                wdata_hold <= s_axi_wdata;
                wstrb_hold <= s_axi_wstrb;
            end

            // hold BVALID until handshake
            if (s_axi_bvalid) begin
                if (s_axi_bready) begin
                    s_axi_bvalid <= 1'b0;
                end
            end else begin
                // If no pending response, and have both address+data => commit
                if (aw_hold && w_hold) begin
                    //$display("[AXI_CTRL] Write Commit: addr=0x%h, data=0x%h, AXI4_CTRL_WR_ST=%b, wr_st_sel=%b", 
                    //         awaddr_hold, wdata_hold, AXI4_CTRL_WR_ST, wr_st_sel);
                    // default OKAY
                    s_axi_bresp  <= 2'b00;
                    // AXI4-Lite compliance checks
                    // Check address alignment
                    if (!wr_addr_aligned) begin
                        s_axi_bresp <= 2'b10; // SLVERR - misaligned access
                    end
                    // Check for unmapped address (must respond with DECERR)
                    else if (!wr_addr_valid) begin
                        s_axi_bresp <= 2'b11; // DECERR - no slave at address
                    end
                    // Optional: Check protection (if enabled)
                    else if (CHECK_PROTECTION && PRIVILEGED_ONLY && s_axi_awprot[0] == 1'b0) begin
                        s_axi_bresp <= 2'b10; // SLVERR - unprivileged access denied
                    end
                    // Proceed with normal write operations
                    else begin
                        // CONF write
                        if (AXI4_CTRL_CONF && wr_conf_sel) begin
                            // Use shift instead of division (CONF_OFFSET = 4 = 2^2)
                            wr_index = (awaddr_hold - CONF_BASE_ADDR) >> 2;
                            if (wr_index < CONF_REG_NUM) begin
                                // Apply WSTRB per byte lane
                                for (i = 0; i < STRB_W; i = i + 1) begin
                                    if (wstrb_hold[i]) begin
                                        conf_reg_o[wr_index*DATA_W + (8*i) +: 8] <= wdata_hold[(8*i) +: 8];
                                    end
                                end
                            end else begin
                                // decode error
                                s_axi_bresp <= 2'b11; // DECERR
                            end
                        end
                        // Stream write (push)
                        else if (AXI4_CTRL_WR_ST && wr_st_sel) begin
                            //$display("[AXI_CTRL] Stream Write detected: addr=0x%h, data=0x%h, can_accept=%b", 
                            //         awaddr_hold, wdata_hold, wr_st_can_accept_i);
                            // For UART TX: typically only use lower 8 bits
                            // Apply WSTRB mask: only send bytes with strobe asserted
                            wr_st_data_o <= wdata_hold;  // Full word (application can select bytes)
                            // Select specific FIFO based on address (ST_WR_OFFSET = 4 = 2^2)
                            wr_index = (awaddr_hold - ST_WR_BASE_ADDR) >> 2;
                            if (wr_index < ST_WR_FIFO_NUM) begin
                                // Only push if stream can accept (FIFO not full)
                                if (wr_st_can_accept_i[wr_index]) begin
                                    wr_st_push_o[wr_index] <= 1'b1;
                                //    $display("[AXI_CTRL] PUSH OK: index=%0d, data=0x%02h", wr_index, wdata_hold[7:0]);
                                end else begin
                                    // FIFO full - return error
                                    s_axi_bresp <= 2'b10; // SLVERR
                                //    $display("[AXI_CTRL] PUSH FAILED: FIFO FULL");
                                end
                            end else begin
                                s_axi_bresp <= 2'b11; // DECERR
                            end
                        end
                        // Memory write
                        else if (AXI4_CTRL_MEM) begin
                            mem_wr_addr_o <= awaddr_hold;
                            mem_wr_data_o <= wdata_hold;
                            mem_wr_strb_o <= wstrb_hold;  // Pass WSTRB to memory subsystem
                            // only assert valid when mem ready, else return SLVERR
                            if (mem_wr_rdy_i) begin
                                mem_wr_vld_o <= 1'b1;
                            end else begin
                                s_axi_bresp <= 2'b10; // SLVERR (slave not ready)
                            end
                        end
                    end
                    // produce response
                    s_axi_bvalid <= 1'b1;
                    // consume buffered AW/W
                    aw_hold <= 1'b0;
                    w_hold  <= 1'b0;
                end else begin
                    // Clear 1-cycle pulse signals
                    wr_st_push_o <= {ST_WR_FIFO_NUM{1'b0}};
                end
            end
        end
    end

    // Read channel: 1 outstanding read
    // ARREADY only when not holding a pending RVALID
    // RVALID must stay asserted until RREADY, and RDATA/RRESP stable while RVALID=1
    reg              ar_hold;
    reg [ADDR_W-1:0] araddr_hold;

    assign s_axi_arready = (!ar_hold) && (!s_axi_rvalid);

    wire ar_hs = s_axi_arvalid && s_axi_arready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar_hold      <= 1'b0;
            araddr_hold  <= {ADDR_W{1'b0}};

            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= {DATA_W{1'b0}};
            s_axi_rresp  <= 2'b00;

            mem_rd_vld_o <= 1'b0;
            mem_rd_addr_o<= {ADDR_W{1'b0}};

            rd_st_pop_o  <= {ST_RD_FIFO_NUM{1'b0}};
            rd_timeout_cnt <= 16'h0000;
        end else begin
            // default pulses
            mem_rd_vld_o <= 1'b0;
            rd_st_pop_o  <= {ST_RD_FIFO_NUM{1'b0}};
            
            // Reset timeout when no pending read
            if (!ar_hold) begin
                rd_timeout_cnt <= 16'h0000;
            end

            // accept AR
            if (ar_hs) begin
                ar_hold     <= 1'b1;
                araddr_hold <= s_axi_araddr;

                // if memory mode, issue read request pulse
                if (AXI4_CTRL_MEM) begin
                    mem_rd_addr_o <= s_axi_araddr;
                    mem_rd_vld_o  <= 1'b1;
                end
            end

            // if we are holding a read request and not yet presenting RVALID, decide when data ready
            if (ar_hold && !s_axi_rvalid) begin
                // default OKAY
                s_axi_rresp <= 2'b00;
                // AXI4-Lite compliance checks
                // Check address alignment
                if (!rd_addr_aligned) begin
                    s_axi_rdata  <= {DATA_W{1'b0}};
                    s_axi_rresp  <= 2'b10; // SLVERR - misaligned access
                    s_axi_rvalid <= 1'b1;
                    ar_hold      <= 1'b0;
                end
                // Check for unmapped address
                else if (!rd_addr_valid) begin
                    s_axi_rdata  <= {DATA_W{1'b0}};
                    s_axi_rresp  <= 2'b11; // DECERR - no slave at address
                    s_axi_rvalid <= 1'b1;
                    ar_hold      <= 1'b0;
                end
                // Optional: Check protection
                else if (CHECK_PROTECTION && PRIVILEGED_ONLY && s_axi_arprot[0] == 1'b0) begin
                    s_axi_rdata  <= {DATA_W{1'b0}};
                    s_axi_rresp  <= 2'b10; // SLVERR - unprivileged access denied
                    s_axi_rvalid <= 1'b1;
                    ar_hold      <= 1'b0;
                end
                // Proceed with normal read operations
                else begin

                    if (AXI4_CTRL_CONF && ((araddr_hold >= CONF_BASE_ADDR) &&
                        (araddr_hold < (CONF_BASE_ADDR + CONF_OFFSET*CONF_REG_NUM)))) begin

                        // Use shift instead of division (CONF_OFFSET = 4 = 2^2)
                        rd_index = (araddr_hold - CONF_BASE_ADDR) >> 2;
                        if (rd_index < CONF_REG_NUM) begin
                            s_axi_rdata  <= conf_reg_o[rd_index*DATA_W +: DATA_W];
                            s_axi_rvalid <= 1'b1;
                            ar_hold      <= 1'b0;
                        end else begin
                            s_axi_rdata  <= {DATA_W{1'b0}};
                            s_axi_rresp  <= 2'b11; // DECERR
                            s_axi_rvalid <= 1'b1;
                            ar_hold      <= 1'b0;
                        end

                    end else if (AXI4_CTRL_RD_ST && ((araddr_hold >= ST_RD_BASE_ADDR) &&
                        (araddr_hold < (ST_RD_BASE_ADDR + ST_RD_OFFSET*ST_RD_FIFO_NUM)))) begin

                        // RX FIFO: only complete when data available
                        if (|rd_st_has_data_i) begin
                            s_axi_rdata  <= rd_st_data_i;
                            s_axi_rvalid <= 1'b1;
                            s_axi_rresp  <= 2'b00;
                            // pop when master will accept (same cycle if RREADY=1)
                            // Here we assert pop when we assert RVALID; actual "consume" occurs on R handshake below
                            // We'll generate pop at handshake time instead (safer)
                            ar_hold      <= 1'b0;
                            rd_timeout_cnt <= 16'h0000;  // Clear timeout
                        end else begin
                            // Waiting for data: increment timeout counter
                            rd_timeout_cnt <= rd_timeout_cnt + 1'b1;
                            // Check for timeout to prevent deadlock
                            if (rd_timeout_cnt >= READ_TIMEOUT) begin
                                s_axi_rdata  <= {DATA_W{1'b0}};
                                s_axi_rresp  <= 2'b10;  // SLVERR - timeout (no data available)
                                s_axi_rvalid <= 1'b1;
                                ar_hold      <= 1'b0;
                                rd_timeout_cnt <= 16'h0000;
                            end
                        end

                    end else if (AXI4_CTRL_MEM) begin
                        // wait for memory returning ready
                        if (mem_rd_rdy_i) begin
                            s_axi_rdata  <= mem_rd_data_i;
                            s_axi_rvalid <= 1'b1;
                            ar_hold      <= 1'b0;
                        end

                    end else if (AXI4_CTRL_STAT) begin
                        s_axi_rdata  <= stat_reg_i;
                        s_axi_rvalid <= 1'b1;
                        ar_hold      <= 1'b0;
                    end
                end
            end

            // Hold RVALID until RREADY; on handshake, drop RVALID.
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;

                // If the read was from stream region, pop FIFO exactly on handshake
                if (AXI4_CTRL_RD_ST && ((araddr_hold >= ST_RD_BASE_ADDR) &&
                    (araddr_hold < (ST_RD_BASE_ADDR + ST_RD_OFFSET*ST_RD_FIFO_NUM)))) begin
                    // Select specific FIFO based on address (ST_RD_OFFSET = 4 = 2^2)
                    rd_index = (araddr_hold - ST_RD_BASE_ADDR) >> 2;
                    if (rd_index < ST_RD_FIFO_NUM) begin
                        rd_st_pop_o <= (1'b1 << rd_index);
                    end else begin
                        rd_st_pop_o <= {ST_RD_FIFO_NUM{1'b0}};
                    end
                end
            end
        end
    end

endmodule
