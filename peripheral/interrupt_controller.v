module interrupt_controller (
    input clk,
    input reset,
    input irq_accel,  // ID = 1
    input irq_uart,   // ID = 2
    input irq_spi,    // ID = 3
    input irq_gpio,   // ID = 4
    input [11:0] cpu_addr,
    input cpu_read_req,
    input cpu_write_req,
    input [31:0] cpu_write_data,
    output reg [31:0] cpu_read_data,
    output cpu_ext_irq
);
    // ---------------------------------------------------------
    // 1. Internal Register and Wire Declarations
    // ---------------------------------------------------------
    reg [4:1] ie;             // Interrupt Enable Register
    reg [3:0] count_1, count_2, count_3, count_4; // Queuing counters (max 15 events per ID)
    reg [4:1] overflow;       // Error flags triggered when a counter exceeds its limit
    reg [2:0] last_served_id; // Round-Robin pointer to track the last processed ID
    
    // Delayed registers for edge detection logic
    reg irq_accel_d, irq_uart_d, irq_spi_d, irq_gpio_d;

    // ---------------------------------------------------------
    // 2. Edge Detection and Control Signal Logic
    // ---------------------------------------------------------
    // Detect rising edges of incoming interrupt signals
    wire edge_1 = irq_accel & ~irq_accel_d;
    wire edge_2 = irq_uart  & ~irq_uart_d;
    wire edge_3 = irq_spi   & ~irq_spi_d;
    wire edge_4 = irq_gpio  & ~irq_gpio_d;

    // Detect CPU "Complete" write command for each specific ID
    wire complete_1 = (cpu_write_req && cpu_addr == 12'h004 && cpu_write_data == 32'd1);
    wire complete_2 = (cpu_write_req && cpu_addr == 12'h004 && cpu_write_data == 32'd2);
    wire complete_3 = (cpu_write_req && cpu_addr == 12'h004 && cpu_write_data == 32'd3);
    wire complete_4 = (cpu_write_req && cpu_addr == 12'h004 && cpu_write_data == 32'd4);

    // An interrupt is active only if its counter > 0 AND it is enabled
    wire [4:1] active_irq;
    assign active_irq[1] = (count_1 > 0) & ie[1];
    assign active_irq[2] = (count_2 > 0) & ie[2];
    assign active_irq[3] = (count_3 > 0) & ie[3];
    assign active_irq[4] = (count_4 > 0) & ie[4];

    // Global external interrupt signal sent to the CPU
    assign cpu_ext_irq = (active_irq != 4'b0000);

    // ---------------------------------------------------------
    // 3. Sequential Logic: Counters and State Updates
    // ---------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            ie <= 4'b0000;
            overflow <= 4'b0000;
            count_1 <= 0; count_2 <= 0; count_3 <= 0; count_4 <= 0;
            irq_accel_d <= 0; irq_uart_d <= 0; irq_spi_d <= 0; irq_gpio_d <= 0;
            last_served_id <= 3'd0;
        end else begin
            // Shift interrupt signals into delay registers
            irq_accel_d <= irq_accel;
            irq_uart_d  <= irq_uart;
            irq_spi_d   <= irq_spi;
            irq_gpio_d  <= irq_gpio;

            // Handle CPU writes to the Interrupt Enable (IE) register
            if (cpu_write_req && cpu_addr == 12'h000) ie <= cpu_write_data[4:1];

            // Counter and Overflow Logic for ID 1
            if (edge_1) begin
                if (count_1 < 15) count_1 <= count_1 + 1;
                else overflow[1] <= 1'b1; // Trigger overflow flag if limit reached
            end else if (complete_1 && count_1 > 0) begin
                count_1 <= count_1 - 1; // Decrement when CPU finishes processing
            end

            // Counter and Overflow Logic for ID 2
            if (edge_2) begin
                if (count_2 < 15) count_2 <= count_2 + 1;
                else overflow[2] <= 1'b1;
            end else if (complete_2 && count_2 > 0) begin
                count_2 <= count_2 - 1;
            end

            // Counter and Overflow Logic for ID 3
            if (edge_3) begin
                if (count_3 < 15) count_3 <= count_3 + 1;
                else overflow[3] <= 1'b1;
            end else if (complete_3 && count_3 > 0) begin
                count_3 <= count_3 - 1;
            end

            // Counter and Overflow Logic for ID 4
            if (edge_4) begin
                if (count_4 < 15) count_4 <= count_4 + 1;
                else overflow[4] <= 1'b1;
            end else if (complete_4 && count_4 > 0) begin
                count_4 <= count_4 - 1;
            end

            // Clear-on-read: Reset overflow flags when CPU reads from the error register
            if (cpu_read_req && cpu_addr == 12'h008) overflow <= 4'b0000;

            // Update Round-Robin pointer based on the last completed ID
            if (complete_1) last_served_id <= 3'd1;
            else if (complete_2) last_served_id <= 3'd2;
            else if (complete_3) last_served_id <= 3'd3;
            else if (complete_4) last_served_id <= 3'd4;
        end
    end

    // ---------------------------------------------------------
    // 4. Round-Robin Arbitration Logic
    // ---------------------------------------------------------
    reg [31:0] current_claim_id;
    always @(*) begin
        current_claim_id = 32'd0; // Default: No active interrupt
        case (last_served_id)
            3'd1: begin // After ID 1, check 2 -> 3 -> 4 -> 1
                if      (active_irq[2]) current_claim_id = 32'd2;
                else if (active_irq[3]) current_claim_id = 32'd3;
                else if (active_irq[4]) current_claim_id = 32'd4;
                else if (active_irq[1]) current_claim_id = 32'd1;
            end
            3'd2: begin // After ID 2, check 3 -> 4 -> 1 -> 2
                if      (active_irq[3]) current_claim_id = 32'd3;
                else if (active_irq[4]) current_claim_id = 32'd4;
                else if (active_irq[1]) current_claim_id = 32'd1;
                else if (active_irq[2]) current_claim_id = 32'd2;
            end
            3'd3: begin // After ID 3, check 4 -> 1 -> 2 -> 3
                if      (active_irq[4]) current_claim_id = 32'd4;
                else if (active_irq[1]) current_claim_id = 32'd1;
                else if (active_irq[2]) current_claim_id = 32'd2;
                else if (active_irq[3]) current_claim_id = 32'd3;
            end
            default: begin // After ID 4 or at init, check 1 -> 2 -> 3 -> 4
                if      (active_irq[1]) current_claim_id = 32'd1;
                else if (active_irq[2]) current_claim_id = 32'd2;
                else if (active_irq[3]) current_claim_id = 32'd3;
                else if (active_irq[4]) current_claim_id = 32'd4;
            end
        endcase
    end

    // ---------------------------------------------------------
    // 5. CPU Bus Interface: Read Access
    // ---------------------------------------------------------
    always @(*) begin
        cpu_read_data = 32'b0;
        if (cpu_read_req) begin
            case (cpu_addr)
                12'h000: cpu_read_data = {27'b0, ie, 1'b0};   // Read Interrupt Enable status
                12'h004: cpu_read_data = current_claim_id;    // Claim the highest priority active ID
                12'h008: cpu_read_data = {28'b0, overflow};   // Read Overflow/Error flags
                default: cpu_read_data = 32'b0;
            endcase
        end
    end

endmodule