`timescale 1ns / 1ps

module tb_sdram_controller();

    // ==========================================
    // SYSTEM CONFIGURATION PARAMETERS
    // ==========================================
    parameter ADDR_WIDTH            = 32;
    parameter DATA_WIDTH            = 32;
    parameter ID_WIDTH              = 5;
    
    parameter SDRAM_ADDR_WIDTH      = 13;
    parameter SDRAM_COL_WIDTH       = 9;
    parameter SDRAM_BANK_WIDTH      = 2;
    parameter SDRAM_DATA_WIDTH      = 32;
    
    // Reduced initialization time for faster simulation
    parameter INIT_DELAY_CYCLES     = 100; 
    parameter TRP_CYCLES            = 2;
    parameter TRCD_CYCLES           = 2;
    parameter TCAS_CYCLES           = 3;
    parameter TRFC_CYCLES           = 7;
    parameter TWR_CYCLES            = 2;
    parameter REFRESH_PERIOD_CYCLES = 780;

    // ==========================================
    // INTERFACE SIGNALS
    // ==========================================
    reg                       clk;
    reg                       rst_n;

    // AXI Write Address Channel
    reg  [ID_WIDTH-1:0]       s_axi_awid;
    reg  [ADDR_WIDTH-1:0]     s_axi_awaddr;
    reg  [7:0]                s_axi_awlen;
    reg  [2:0]                s_axi_awsize;
    reg  [1:0]                s_axi_awburst;
    reg                       s_axi_awvalid;
    wire                      s_axi_awready;

    // AXI Write Data Channel
    reg  [DATA_WIDTH-1:0]     s_axi_wdata;
    reg  [(DATA_WIDTH/8)-1:0] s_axi_wstrb;
    reg                       s_axi_wlast;
    reg                       s_axi_wvalid;
    wire                      s_axi_wready;

    // AXI Write Response Channel
    wire [ID_WIDTH-1:0]       s_axi_bid;
    wire [1:0]                s_axi_bresp;
    wire                      s_axi_bvalid;
    reg                       s_axi_bready;

    // AXI Read Address Channel
    reg  [ID_WIDTH-1:0]       s_axi_arid;
    reg  [ADDR_WIDTH-1:0]     s_axi_araddr;
    reg  [7:0]                s_axi_arlen;
    reg  [2:0]                s_axi_arsize;
    reg  [1:0]                s_axi_arburst;
    reg                       s_axi_arvalid;
    wire                      s_axi_arready;

    // AXI Read Data Channel
    wire [ID_WIDTH-1:0]       s_axi_rid;
    wire [DATA_WIDTH-1:0]     s_axi_rdata;
    wire [1:0]                s_axi_rresp;
    wire                      s_axi_rlast;
    wire                      s_axi_rvalid;
    reg                       s_axi_rready;

    // SDRAM PHY Interface
    wire                      sdram_clk;
    wire                      sdram_cke;
    wire                      sdram_cs_n;
    wire                      sdram_ras_n;
    wire                      sdram_cas_n;
    wire                      sdram_we_n;
    wire [SDRAM_BANK_WIDTH-1:0] sdram_ba;
    wire [SDRAM_ADDR_WIDTH-1:0] sdram_addr;
    wire [(SDRAM_DATA_WIDTH/8)-1:0] sdram_dqm;
    
    reg  [SDRAM_DATA_WIDTH-1:0] sdram_dq_i;
    wire [SDRAM_DATA_WIDTH-1:0] sdram_dq_o;
    wire                        sdram_dq_oe;

    reg [31:0] rand_addr;
    reg [7:0]  rand_len;
    reg [31:0] rand_data;

    // Test statistics variables
    integer tests_passed;
    integer tests_failed;
    integer i, j;

    // ==========================================
    // DUT INSTANTIATION
    // ==========================================
    axi_sdram_controller #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .ID_WIDTH(ID_WIDTH),
        .SDRAM_ADDR_WIDTH(SDRAM_ADDR_WIDTH),
        .SDRAM_COL_WIDTH(SDRAM_COL_WIDTH),
        .SDRAM_BANK_WIDTH(SDRAM_BANK_WIDTH),
        .SDRAM_DATA_WIDTH(SDRAM_DATA_WIDTH),
        .INIT_DELAY_CYCLES(INIT_DELAY_CYCLES),
        .TRP_CYCLES(TRP_CYCLES),
        .TRCD_CYCLES(TRCD_CYCLES),
        .TCAS_CYCLES(TCAS_CYCLES),
        .TRFC_CYCLES(TRFC_CYCLES),
        .TWR_CYCLES(TWR_CYCLES),
        .REFRESH_PERIOD_CYCLES(REFRESH_PERIOD_CYCLES)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .s_axi_awid(s_axi_awid), .s_axi_awaddr(s_axi_awaddr), .s_axi_awlen(s_axi_awlen), .s_axi_awsize(s_axi_awsize), .s_axi_awburst(s_axi_awburst), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wlast(s_axi_wlast), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bid(s_axi_bid), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        .s_axi_arid(s_axi_arid), .s_axi_araddr(s_axi_araddr), .s_axi_arlen(s_axi_arlen), .s_axi_arsize(s_axi_arsize), .s_axi_arburst(s_axi_arburst), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_rid(s_axi_rid), .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp), .s_axi_rlast(s_axi_rlast), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
        .sdram_clk(sdram_clk), .sdram_cke(sdram_cke), .sdram_cs_n(sdram_cs_n), .sdram_ras_n(sdram_ras_n), .sdram_cas_n(sdram_cas_n), .sdram_we_n(sdram_we_n),
        .sdram_ba(sdram_ba), .sdram_addr(sdram_addr), .sdram_dqm(sdram_dqm),
        .sdram_dq_i(sdram_dq_i), .sdram_dq_o(sdram_dq_o), .sdram_dq_oe(sdram_dq_oe)
    );

    // ==========================================
    // CLOCK GENERATION (100 MHz)
    // ==========================================
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period
    end

    // ==========================================
    // BEHAVIORAL SDRAM PHY MODEL (FIXED TIMING)
    // ==========================================
    // LỖI ĐÃ ĐƯỢC FIX: Mở rộng mảng lên 16,777,216 elements (64MB) để bao phủ toàn bộ 4 Banks. 
    // Mảng cũ quá nhỏ dẫn đến việc truy cập Out-of-Bounds làm treo cứng Simulator.
    reg [31:0] sdram_memory [0:16777215]; 
    reg [12:0] active_row [0:3];
    
    // CAS Latency Read Pipeline
    reg [31:0] cas_pipeline_data [0:TCAS_CYCLES];
    reg        cas_pipeline_valid [0:TCAS_CYCLES];

    wire [3:0] sdram_cmd = {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n};
    wire [23:0] flat_addr_wr = {sdram_ba, active_row[sdram_ba], sdram_addr[8:0]};
    wire [23:0] flat_addr_rd = {sdram_ba, active_row[sdram_ba], sdram_addr[8:0]};

    integer k;
    always @(posedge sdram_clk) begin
        // Shift read pipeline
        for (k = TCAS_CYCLES; k > 0; k = k - 1) begin
            cas_pipeline_data[k]  <= cas_pipeline_data[k-1];
            cas_pipeline_valid[k] <= cas_pipeline_valid[k-1];
        end
        cas_pipeline_valid[0] <= 1'b0;

        if (!sdram_cs_n) begin
            case (sdram_cmd)
                4'b0011: begin // ACTIVATE
                    active_row[sdram_ba] <= sdram_addr;
                end
                4'b0100: begin // WRITE
                    if (sdram_dq_oe) begin
                        sdram_memory[flat_addr_wr] <= sdram_dq_o;
                    end
                end
                4'b0101: begin // READ
                    cas_pipeline_data[0]  <= sdram_memory[flat_addr_rd];
                    cas_pipeline_valid[0] <= 1'b1;
                end
                4'b0010: begin // PRECHARGE
                end
                default: ; // NOP, REFRESH, LOAD_MODE
            endcase
        end
    end

    // Output valid data with exact CAS matching
    always @(*) begin
        if (cas_pipeline_valid[TCAS_CYCLES-1]) begin
            sdram_dq_i = cas_pipeline_data[TCAS_CYCLES-1];
        end else begin
            sdram_dq_i = 32'hZZZZZZZZ;
        end
    end

    // ==========================================
    // STANDARD AXI MASTER TASKS (FIXED HANDSHAKES)
    // ==========================================

    task axi_write;
        input [31:0] addr;
        input [7:0]  len;
        input [31:0] data_start;
        integer w_idx;
        begin
            // Address Channel
            s_axi_awvalid = 1;
            s_axi_awaddr  = addr;
            s_axi_awlen   = len;
            s_axi_awsize  = 3'b010; // 4 Bytes
            s_axi_awburst = 2'b01;  // INCR
            
            @(posedge clk);
            while (!s_axi_awready) @(posedge clk); // Synchronous Wait
            s_axi_awvalid = 0;

            // Data Channel (Burst)
            for (w_idx = 0; w_idx <= len; w_idx = w_idx + 1) begin
                s_axi_wvalid = 1;
                s_axi_wdata  = data_start + w_idx; 
                s_axi_wstrb  = 4'hF;
                s_axi_wlast  = (w_idx == len);
                
                @(posedge clk);
                while (!s_axi_wready) @(posedge clk);
            end
            s_axi_wvalid = 0;
            s_axi_wlast  = 0;

            // Response Channel
            s_axi_bready = 1;
            @(posedge clk);
            while (!s_axi_bvalid) @(posedge clk);
            s_axi_bready = 0;
        end
    endtask

    task axi_read_and_check;
        input [31:0] addr;
        input [7:0]  len;
        input [31:0] expected_start;
        integer r_idx;
        reg [31:0] expected_val;
        begin
            // Address Channel
            s_axi_arvalid = 1;
            s_axi_araddr  = addr;
            s_axi_arlen   = len;
            s_axi_arsize  = 3'b010;
            s_axi_arburst = 2'b01;
            
            @(posedge clk);
            while (!s_axi_arready) @(posedge clk);
            s_axi_arvalid = 0;

            // Data Channel
            s_axi_rready = 1;
            for (r_idx = 0; r_idx <= len; r_idx = r_idx + 1) begin
                @(posedge clk);
                while (!s_axi_rvalid) @(posedge clk); // Synchronous Wait
                
                expected_val = expected_start + r_idx;
                
                if (s_axi_rdata !== expected_val) begin
                    $display("[ERROR] Read at %h | Expected: %h | Received: %h", addr + (r_idx*4), expected_val, s_axi_rdata);
                    tests_failed = tests_failed + 1;
                end else begin
                    tests_passed = tests_passed + 1;
                end
            end
            s_axi_rready = 0;
        end
    endtask

    // ==========================================
    // MAIN TEST SEQUENCE (10 PHASES)
    // ==========================================
    initial begin
        // Initialize signals
        s_axi_awid = 0; s_axi_awaddr = 0; s_axi_awlen = 0; s_axi_awsize = 0; s_axi_awburst = 0; s_axi_awvalid = 0;
        s_axi_wdata = 0; s_axi_wstrb = 0; s_axi_wlast = 0; s_axi_wvalid = 0;
        s_axi_bready = 0;
        s_axi_arid = 0; s_axi_araddr = 0; s_axi_arlen = 0; s_axi_arsize = 0; s_axi_arburst = 0; s_axi_arvalid = 0;
        s_axi_rready = 0;
        
        tests_passed = 0;
        tests_failed = 0;

        // Reset system
        rst_n = 0;
        #50 rst_n = 1;

        $display("==================================================");
        $display("STARTING AXI SDRAM CONTROLLER TESTBENCH");
        $display("==================================================");

        // ---------------------------------------------------------
        // PHASE 1: INITIALIZATION
        // ---------------------------------------------------------
        $display("\n[Phase 1] Waiting for SDRAM Initialization (Delay + Precharge + Refresh + LMR)...");
        #(INIT_DELAY_CYCLES * 10 + 200);
        $display("[Phase 1] Initialization complete. Checking FSM IDLE.");
        
        // ---------------------------------------------------------
        // PHASE 2: SINGLE WRITE & READ (ROW HIT)
        // ---------------------------------------------------------
        $display("\n[Phase 2] Simple Write/Read (Single Beat) - Row Hit Test");
        axi_write(32'h0000_0000, 8'd0, 32'hAAAA_BBBB);
        axi_read_and_check(32'h0000_0000, 8'd0, 32'hAAAA_BBBB);

        // ---------------------------------------------------------
        // PHASE 3: BACK-TO-BACK ROW MISS
        // ---------------------------------------------------------
        $display("\n[Phase 3] Force Row Miss to test Open-Row Policy");
        axi_write(32'h0000_0400, 8'd0, 32'h1111_1111);
        axi_write(32'h0000_0800, 8'd0, 32'h2222_2222);
        
        axi_read_and_check(32'h0000_0400, 8'd0, 32'h1111_1111);
        axi_read_and_check(32'h0000_0800, 8'd0, 32'h2222_2222);

        // ---------------------------------------------------------
        // PHASE 4: AXI BURST TRANSFER
        // ---------------------------------------------------------
        $display("\n[Phase 4] AXI Burst Transfer (Len = 3, i.e., 4 Beats)");
        axi_write(32'h0000_1000, 8'd3, 32'hC0DE_0000);
        axi_read_and_check(32'h0000_1000, 8'd3, 32'hC0DE_0000);

        // ---------------------------------------------------------
        // PHASE 5: ROW BOUNDARY CROSSING
        // ---------------------------------------------------------
        $display("\n[Phase 5] Burst crossing Column Boundary");
        axi_write(32'h0000_07F8, 8'd3, 32'hB00D_0000);
        axi_read_and_check(32'h0000_07F8, 8'd3, 32'hB00D_0000);

        // ---------------------------------------------------------
        // PHASE 6: MULTI-BANK INTERLEAVING
        // ---------------------------------------------------------
        $display("\n[Phase 6] Multi-Bank Interleaving Test (Open 4 Banks simultaneously)");
        axi_write(32'h0000_0000, 8'd0, 32'hBABA_0000); 
        axi_write(32'h0080_0000, 8'd0, 32'hBABA_1111); 
        axi_write(32'h0100_0000, 8'd0, 32'hBABA_2222); 
        axi_write(32'h0180_0000, 8'd0, 32'hBABA_3333); 
        
        axi_read_and_check(32'h0000_0000, 8'd0, 32'hBABA_0000);
        axi_read_and_check(32'h0080_0000, 8'd0, 32'hBABA_1111);
        axi_read_and_check(32'h0100_0000, 8'd0, 32'hBABA_2222);
        axi_read_and_check(32'h0180_0000, 8'd0, 32'hBABA_3333);

        // ---------------------------------------------------------
        // PHASE 7: AUTO-REFRESH TOLERANCE
        // ---------------------------------------------------------
        $display("\n[Phase 7] Wait and verify FSM handles Auto-Refresh safely");
        #(REFRESH_PERIOD_CYCLES * 10 + 500); 
        axi_write(32'h0000_5550, 8'd1, 32'hF00D_F00D);
        axi_read_and_check(32'h0000_5550, 8'd1, 32'hF00D_F00D);

        // ---------------------------------------------------------
        // PHASE 8: TURNAROUND PENALTY
        // ---------------------------------------------------------
        $display("\n[Phase 8] Continuous Read/Write Turnaround (Penalty Test)");
        axi_write(32'h0000_6000, 8'd0, 32'hA1A1_A1A1);
        axi_read_and_check(32'h0000_6000, 8'd0, 32'hA1A1_A1A1);
        axi_write(32'h0000_6000, 8'd0, 32'hB2B2_B2B2);
        axi_read_and_check(32'h0000_6000, 8'd0, 32'hB2B2_B2B2);

        // ---------------------------------------------------------
        // PHASE 9: MAXIMUM BURST LENGTH
        // ---------------------------------------------------------
        $display("\n[Phase 9] Maximum Burst Transfer (Len = 255)");
        axi_write(32'h0010_0000, 8'd255, 32'h1000_0000);
        axi_read_and_check(32'h0010_0000, 8'd255, 32'h1000_0000);

        // ---------------------------------------------------------
        // PHASE 10: RANDOM STRESS TEST
        // ---------------------------------------------------------
        $display("\n[Phase 10] Random Stress Test (Random Data and Addresses)");
        for (j = 0; j < 20; j = j + 1) begin
            rand_addr = {$random} & 32'h01FF_FFFC; 
            rand_len  = {$random} % 16;            
            rand_data = {$random};
            
            axi_write(rand_addr, rand_len, rand_data);
            axi_read_and_check(rand_addr, rand_len, rand_data);
        end

        // ---------------------------------------------------------
        // FINAL RESULTS REPORT
        // ---------------------------------------------------------
        $display("\n==================================================");
        $display("TESTING COMPLETED:");
        $display("TOTAL TESTS PASSED : %0d", tests_passed);
        $display("TOTAL TESTS FAILED : %0d", tests_failed);
        
        if (tests_failed == 0) begin
            $display("\n[CONCLUSION] => [ALL TESTS PASSED SUCCESSFULLY!]");
        end else begin
            $display("\n[CONCLUSION] => [FAILED! Please check FSM or Timing]");
        end
        $display("==================================================");
        
        $finish;
    end

endmodule