`timescale 1ns / 1ps

module apb_interrupt_controller #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_WIDTH = 32,
    parameter NUM_IRQ    = 6    
)(
    // --- APB4 Slave Interface ---
    input  wire                   pclk,
    input  wire                   presetn,
    input  wire [ADDR_WIDTH-1:0]  paddr,
    input  wire                   psel,
    input  wire                   penable,
    input  wire                   pwrite,
    input  wire [DATA_WIDTH-1:0]  pwdata,
    input  wire [3:0]             pstrb,
    output reg                    pready,
    output reg  [DATA_WIDTH-1:0]  prdata,
    output reg                    pslverr,

    // --- Interrupt Sources ---
    input  wire                   irq_timer, // ID = 1
    input  wire                   irq_uart,  // ID = 2
    input  wire                   irq_spi,   // ID = 3
    input  wire                   irq_i2c,   // ID = 4
    input  wire                   irq_gpio,  // ID = 5
    input  wire                   irq_accel, // ID = 6

    // --- Signal to CPU ---
    output wire                   cpu_ext_irq
);
    wire [NUM_IRQ:1] raw_irq = {irq_accel, irq_gpio, irq_i2c, irq_spi, irq_uart, irq_timer};

    // 1. ĐỒNG BỘ HÓA & EDGE DETECTION
    reg [NUM_IRQ:1] irq_sync1, irq_sync2, irq_sync3;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) {irq_sync3, irq_sync2, irq_sync1} <= 0;
        else          {irq_sync3, irq_sync2, irq_sync1} <= {irq_sync2, irq_sync1, raw_irq};
    end
    wire [NUM_IRQ:1] irq_edge = irq_sync2 & ~irq_sync3;

    // 2. INTERNAL REGISTERS & COUNTERS
    reg  [NUM_IRQ:1] ie;
    reg  [3:0]       counters [1:NUM_IRQ];
    reg  [NUM_IRQ:1] overflow;
    reg  [2:0]       last_served_id;
    
    wire [NUM_IRQ:1] active_irq;
    genvar g;
    generate
        for (g = 1; g <= NUM_IRQ; g = g + 1) begin : GEN_ACTIVE
            assign active_irq[g] = (counters[g] > 0) & ie[g];
        end
    endgenerate

    assign cpu_ext_irq = (|active_irq);

    wire       apb_write   = psel && penable && pwrite;
    wire       apb_read    = psel && !penable && !pwrite;
    wire       is_complete = apb_write && (paddr[11:0] == 12'h004);
    wire [2:0] complete_id = pwdata[2:0];
    wire [NUM_IRQ:1] ack_this_irq;

    generate
        for (g = 1; g <= NUM_IRQ; g = g + 1) begin : GEN_ACK_LOGIC
            // Tính toán điều kiện ACK cho từng IRQ dựa trên complete_id
            assign ack_this_irq[g] = is_complete && (complete_id == g) && (counters[g] > 0);
        end
    endgenerate


    // 3. QUEUE COUNTERS LOGIC (Đã fix Race Condition)
    integer i;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            for (i = 1; i <= NUM_IRQ; i = i + 1) counters[i] <= 4'd0;
            overflow <= 0;
            last_served_id <= 3'd0;
        end else begin
            for (i = 1; i <= NUM_IRQ; i = i + 1) begin
                if (irq_edge[i] && !ack_this_irq) begin
                    // Chỉ tăng khi có ngắt và không có ACK
                    if (counters[i] < 4'd15) counters[i] <= counters[i] + 1;
                    else                     overflow[i] <= 1'b1;
                end
                else if (!irq_edge[i] && ack_this_irq) begin
                    // Chỉ giảm khi có ACK và không có ngắt mới
                    counters[i] <= counters[i] - 1;
                end
                // Nếu cả 2 cùng xảy ra (irq_edge[i] && ack_this_irq) -> Bù trừ nhau (Counter giữ nguyên)
            end
            
            if (apb_read && (paddr[11:0] == 12'h008)) begin
                overflow <= 0;
            end

            if (is_complete && (complete_id >= 1) && (complete_id <= NUM_IRQ)) begin
                last_served_id <= complete_id;
            end
        end
    end

    // 4. ROUND-ROBIN ARBITRATION
    reg [31:0] current_claim_id;
    always @(*) begin
        current_claim_id = 32'd0;
        case (last_served_id)
            3'd1: if (active_irq[2]) current_claim_id = 32'd2; else if (active_irq[3]) current_claim_id = 32'd3; else if (active_irq[4]) current_claim_id = 32'd4; else if (active_irq[5]) current_claim_id = 32'd5; else if (active_irq[6]) current_claim_id = 32'd6; else if (active_irq[1]) current_claim_id = 32'd1;
            3'd2: if (active_irq[3]) current_claim_id = 32'd3; else if (active_irq[4]) current_claim_id = 32'd4; else if (active_irq[5]) current_claim_id = 32'd5; else if (active_irq[6]) current_claim_id = 32'd6; else if (active_irq[1]) current_claim_id = 32'd1; else if (active_irq[2]) current_claim_id = 32'd2;
            3'd3: if (active_irq[4]) current_claim_id = 32'd4; else if (active_irq[5]) current_claim_id = 32'd5; else if (active_irq[6]) current_claim_id = 32'd6; else if (active_irq[1]) current_claim_id = 32'd1; else if (active_irq[2]) current_claim_id = 32'd2; else if (active_irq[3]) current_claim_id = 32'd3;
            3'd4: if (active_irq[5]) current_claim_id = 32'd5; else if (active_irq[6]) current_claim_id = 32'd6; else if (active_irq[1]) current_claim_id = 32'd1; else if (active_irq[2]) current_claim_id = 32'd2; else if (active_irq[3]) current_claim_id = 32'd3; else if (active_irq[4]) current_claim_id = 32'd4;
            3'd5: if (active_irq[6]) current_claim_id = 32'd6; else if (active_irq[1]) current_claim_id = 32'd1; else if (active_irq[2]) current_claim_id = 32'd2; else if (active_irq[3]) current_claim_id = 32'd3; else if (active_irq[4]) current_claim_id = 32'd4; else if (active_irq[5]) current_claim_id = 32'd5;
            default: if (active_irq[1]) current_claim_id = 32'd1; else if (active_irq[2]) current_claim_id = 32'd2; else if (active_irq[3]) current_claim_id = 32'd3; else if (active_irq[4]) current_claim_id = 32'd4; else if (active_irq[5]) current_claim_id = 32'd5; else if (active_irq[6]) current_claim_id = 32'd6;
        endcase
    end

    // 5. APB INTERFACE
    wire [NUM_IRQ:1] pending_status;
    generate
        for (g = 1; g <= NUM_IRQ; g = g + 1) begin : GEN_PENDING
            assign pending_status[g] = (counters[g] > 0);
        end
    endgenerate

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            ie      <= 0;
            pready  <= 0;
            prdata  <= 0;
            pslverr <= 0;
        end else begin
            pready  <= psel && penable;
            pslverr <= 0;
            
            if (apb_write) begin
                case (paddr[11:0])
                    12'h000: ie <= pwdata[NUM_IRQ:1];
                    12'h004: ; 
                    12'h008, 12'h00C: pslverr <= 1'b1;
                    default: pslverr <= 1'b1;
                endcase
            end
            
            if (apb_read) begin
                case (paddr[11:0])
                    12'h000: prdata <= { {(32-NUM_IRQ-1){1'b0}}, ie, 1'b0 };
                    12'h004: prdata <= current_claim_id;
                    12'h008: prdata <= { {(32-NUM_IRQ-1){1'b0}}, overflow, 1'b0 };
                    12'h00C: prdata <= { {(32-NUM_IRQ-1){1'b0}}, pending_status, 1'b0 };
                    default: begin prdata <= 32'b0; pslverr <= 1'b1; end
                endcase
            end
        end
    end

endmodule