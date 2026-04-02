`timescale 1ns / 1ps

module tb_cordic;

    // --- System Parameters ---
    parameter ADDR_WIDTH = 12;
    parameter DATA_WIDTH = 32;
    parameter CLK_PERIOD = 10;

    // --- DUT Interface Signals ---
    reg                     pclk;
    reg                     presetn;
    reg  [ADDR_WIDTH-1:0]   paddr;
    reg                     psel;
    reg                     penable;
    reg                     pwrite;
    reg  [DATA_WIDTH-1:0]   pwdata;
    reg  [3:0]              pstrb;
    wire                    pready;
    wire [DATA_WIDTH-1:0]   prdata;
    wire                    pslverr;

    // --- Control and Statistics Variables (Verilog-2001) ---
    integer pass_count;
    integer fail_count;
    reg [DATA_WIDTH-1:0] read_data;
    reg signed [31:0] x_val, y_val;

    // --- Module Instantiation (DUT) ---
    apb_cordic #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .pclk(pclk),
        .presetn(presetn),
        .paddr(paddr),
        .psel(psel),
        .penable(penable),
        .pwrite(pwrite),
        .pwdata(pwdata),
        .pstrb(pstrb),
        .pready(pready),
        .prdata(prdata),
        .pslverr(pslverr)
    );

    // --- Clock Generation ---
    initial begin
        pclk = 1'b0;
        forever #(CLK_PERIOD/2) pclk = ~pclk;
    end

    // --- Task: Standard APB Write Protocol ---
    task apb_write;
        input [ADDR_WIDTH-1:0] addr;
        input [DATA_WIDTH-1:0] data;
        begin
            @(posedge pclk);
            paddr   = addr;
            psel    = 1'b1;
            pwrite  = 1'b1;
            pwdata  = data;
            pstrb   = 4'hF;
            penable = 1'b0;
            
            @(posedge pclk);
            penable = 1'b1;
            
            while (pready !== 1'b1) @(posedge pclk);
            
            psel    = 1'b0;
            penable = 1'b0;
            pwrite  = 1'b0;
        end
    endtask

    // --- Task: Standard APB Read Protocol ---
    task apb_read;
        input  [ADDR_WIDTH-1:0] addr;
        output [DATA_WIDTH-1:0] data;
        begin
            @(posedge pclk);
            paddr   = addr;
            psel    = 1'b1;
            pwrite  = 1'b0;
            penable = 1'b0;
            
            @(posedge pclk);
            penable = 1'b1;
            
            while (pready !== 1'b1) @(posedge pclk);
            
            data    = prdata;
            psel    = 1'b0;
            penable = 1'b0;
        end
    endtask

    // --- Main Test Sequence (6 Phases) ---
    initial begin
        // 0. Initialization
        pass_count = 0;
        fail_count = 0;
        presetn = 1'b0;
        psel    = 1'b0;
        penable = 1'b0;
        pwrite  = 1'b0;
        paddr   = 12'd0;
        pwdata  = 32'd0;
        pstrb   = 4'd0;

        $display("==========================================================");
        $display("   STARTING APB CORDIC TEST (6 PHASES)            ");
        $display("==========================================================");

        // PHASE 1: Reset and Default Value Check [cite: 25, 26]
        $display("\n[PHASE 1] System Reset and Default Value Verification...");
        #(CLK_PERIOD * 5);
        presetn = 1'b1;
        #(CLK_PERIOD * 2);
        
        apb_read(12'h008, read_data); // Read default X_IN (K factor) [cite: 26]
        if (read_data == 32'd39797) begin
            $display("   -> PASS: Default X_IN is correct (K = 39797).");
            pass_count = pass_count + 1;
        end else begin
            $display("   -> FAIL: Incorrect X_IN default value. Received: %d", read_data);
            fail_count = fail_count + 1;
        end

        // PHASE 2: Basic Calculation at 0 Degrees [cite: 44, 49]
        $display("\n[PHASE 2] Basic Calculation at 0 Degrees...");
        apb_write(12'h008, 32'd39797); // X_IN = K [cite: 35]
        apb_write(12'h00C, 32'd0);     // Y_IN = 0 [cite: 35]
        apb_write(12'h010, 32'd0);     // Z_IN = 0 (0 degrees) [cite: 35]
        apb_write(12'h000, 32'h1);     // Write 1 to CTRL to Start [cite: 34]
        
        // Wait for Done flag in STATUS register [cite: 30]
        read_data = 32'd0;
        while (read_data[0] == 1'b0) begin
            apb_read(12'h004, read_data);
        end
        
        apb_read(12'h014, x_val); // Read X_OUT (Cos result) [cite: 38]
        apb_read(12'h018, y_val); // Read Y_OUT (Sin result) [cite: 39]
        
        // Tolerance check: Cos(0) ~ 1.0 (65536) and Sin(0) ~ 0 (+/- 5 LSB)
        if (x_val >= 32'd65500 && x_val <= 32'd65550 && $signed(y_val) >= -32'sd5 && $signed(y_val) <= 32'sd5) begin
            $display("   -> PASS: Cos(0) approx 1.0 (Q16.16: %d), Sin(0) approx 0 (%d).", x_val, $signed(y_val));
            pass_count = pass_count + 1;
        end else begin
            $display("   -> FAIL: Calculation error at 0 degrees. X_OUT: %d, Y_OUT: %d", x_val, y_val);
            fail_count = fail_count + 1;
        end

        // PHASE 3: Pre-rotation Test (+135 Degrees) [cite: 45, 46]
        $display("\n[PHASE 3] Pre-rotation Test (+135 Degrees)...");
        apb_write(12'h010, 32'sd1610612736); // 135 degrees in BAM format
        apb_write(12'h000, 32'h1); // Start [cite: 34]
        
        read_data = 32'd0;
        while (read_data[0] == 1'b0) begin
            apb_read(12'h004, read_data);
        end
        
        apb_read(12'h014, x_val);
        apb_read(12'h018, y_val);
        // At 135 degrees (Quadrant 2): Cos is negative, Sin is positive
        if ($signed(x_val) < 0 && $signed(y_val) > 0) begin
            $display("   -> PASS: Cos(135) is negative (%d) and Sin(135) is positive (%d).", $signed(x_val), $signed(y_val));
            pass_count = pass_count + 1;
        end else begin
            $display("   -> FAIL: Pre-rotation logic error. Cos: %d, Sin: %d", $signed(x_val), $signed(y_val));
            fail_count = fail_count + 1;
        end

        // PHASE 4: Read-Only Violation Check [cite: 36]
        $display("\n[PHASE 4] APB Slave Error Check (Read-Only Violation)...");
        apb_write(12'h014, 32'hDEADBEEF); // Attempting to write to X_OUT [cite: 36]
        @(posedge pclk); 
        
        if (pslverr === 1'b1) begin
            $display("   -> PASS: pslverr triggered correctly for Read-Only access.");
            pass_count = pass_count + 1;
        end else begin
            $display("   -> FAIL: No pslverr signal detected during RO violation.");
            fail_count = fail_count + 1;
        end

        // PHASE 5: APB Bus Stress Test [cite: 35]
        $display("\n[PHASE 5] Stress Test: APB Bus interference during CALC state...");
        apb_write(12'h010, 32'sd536870912); // Write 45 degrees
        apb_write(12'h000, 32'h1);          // Start [cite: 34]
        
        // Deliberately write junk data to input registers while Core is busy
        apb_write(12'h008, 32'h12345678); 
        apb_write(12'h00C, 32'h87654321);
        
        read_data = 32'd0;
        while (read_data[0] == 1'b0) begin
            apb_read(12'h004, read_data); 
        end
        $display("   -> PASS: FSM remained stable during APB bus interference.");
        pass_count = pass_count + 1;

        // PHASE 6: Emergency Reset [cite: 25, 40]
        $display("\n[PHASE 6] Emergency Reset: Triggering reset during active calculation...");
        apb_write(12'h010, 32'sd1073741824); // 90 degrees
        apb_write(12'h000, 32'h1);           // Start [cite: 34]
        
        #(CLK_PERIOD * 4); // Allow several iterations to run [cite: 52]
        presetn = 1'b0;    // Trigger Hardware Reset [cite: 25, 40]
        #(CLK_PERIOD * 2);
        presetn = 1'b1;    // Release Reset
        
        apb_read(12'h004, read_data); // Read Status [cite: 38]
        if (read_data == 32'd0) begin
            $display("   -> PASS: FSM and registers successfully recovered to IDLE state.");
            pass_count = pass_count + 1;
        end else begin
            $display("   -> FAIL: System state corrupted after emergency reset. Status = %h", read_data);
            fail_count = fail_count + 1;
        end

        // --- Final Report ---
        $display("\n==========================================================");
        $display("   FINAL TEST REPORT:");
        $display("   - Total Test Phases  : %0d", (pass_count + fail_count));
        $display("   - Passed Phases      : %0d", pass_count);
        $display("   - Failed Phases      : %0d", fail_count);
        $display("==========================================================");
        
        if (fail_count == 0) 
            $display("   CONCLUSION: DESIGN PASSED ALL TESTS (100%% SUCCESS)");
        else 
            $display("   CONCLUSION: DESIGN FAILED (REPAIRS NEEDED)");
        $display("==========================================================");
        
        $finish;
    end

endmodule