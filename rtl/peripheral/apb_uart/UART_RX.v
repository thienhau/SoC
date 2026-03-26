`timescale 1ns / 1ps

module uart_rx #(
    parameter integer DATA_WIDTH   = 8,
    parameter integer TIMEOUT_CYCLES = 16 * 20  // 20 character timeout
) (
    // Clock and Reset
    input  wire                     clk,
    input  wire                     rst_n,
    
    // UART Physical Interface
    input  wire                     RX,
    
    // Baud Rate Interface
    input  wire                     baudrate_clk_en,
    
    // Control Interface
    output wire                     transaction_en,
    output wire [DATA_WIDTH-1:0]    data_out_rx,
    output wire                     fifo_wr,
    output wire                     frame_error,
    output wire                     timeout_error
);
    
    // CONFIGURATION PARAMETERS
    localparam integer DATA_WIDTH_SELECT = 8;
    localparam integer PARITY_SELECT     = 1;      // 1: Enable parity
    localparam integer STOP_BIT_SELECT   = 2;      // 2 stop bits
    
    // STATE DEFINITIONS
    localparam [1:0] IDLE_STATE  = 2'b00;
    localparam [1:0] RECV_STATE  = 2'b01;
    localparam [1:0] ERROR_STATE = 2'b10;
    
    localparam [1:0] START_BIT_STATE = 2'b00;
    localparam [1:0] DATA_STATE      = 2'b01;
    localparam [1:0] PARITY_STATE    = 2'b10;
    localparam [1:0] STOP_STATE      = 2'b11;
    
    // INTERNAL REGISTERS
    reg [1:0] rx_state            = IDLE_STATE;
    reg [1:0] transaction_state   = START_BIT_STATE;
    
    reg [DATA_WIDTH-1:0] rx_buffer          = 'b0;
    reg parity_buffer              = 1'b0;
    reg fifo_wr_reg               = 1'b0;
    reg frame_error_reg           = 1'b0;
    reg timeout_error_reg         = 1'b0;
    reg stop_bit_error            = 1'b0;
    
    reg [3:0] data_counter        = 'b0;
    reg [1:0] stop_bit_counter    = 'b0;
    reg [3:0] oversample_counter  = 'b0;
    
    reg transaction_start_toggle  = 1'b0;
    reg transaction_stop_toggle   = 1'b0;
    
    reg [7:0] timeout_counter     = 'b0;
    
    // INPUT SYNCHRONIZATION (3-stage for metastability)
    (* ASYNC_REG = "TRUE" *) reg [2:0] rx_sync = 3'b111;
    
    always @(posedge clk) begin
        if (!rst_n) begin
            rx_sync <= 3'b111;
        end else begin
            rx_sync <= {rx_sync[1:0], RX};
        end
    end
    
    wire rx_stable = rx_sync[2];
    
    // BAUD RATE SAMPLING
    wire sample_enable = (oversample_counter == 4'd7);  // Sample at middle
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            oversample_counter <= 'b0;
        end else if (rx_state == IDLE_STATE) begin
            oversample_counter <= 'b0;
        end else if (baudrate_clk_en) begin
            if (oversample_counter == 4'd15) begin
                oversample_counter <= 'b0;
            end else begin
                oversample_counter <= oversample_counter + 1'b1;
            end
        end
    end
    
    // START BIT DETECTION
    reg rx_prev = 1'b1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_prev <= 1'b1;
        end else begin
            rx_prev <= rx_stable;
        end
    end
    wire start_bit_detected = (rx_prev == 1'b1) && (rx_stable == 1'b0);

    // OUTPUT ASSIGNMENTS
    assign transaction_en = transaction_start_toggle ^ transaction_stop_toggle;
    assign data_out_rx    = rx_buffer;
    assign fifo_wr        = fifo_wr_reg;
    assign frame_error    = frame_error_reg;
    assign timeout_error  = timeout_error_reg;
    
    // Parity calculation
    wire calculated_parity = ^rx_buffer;
    wire parity_error = (PARITY_SELECT != 0) ? (calculated_parity != parity_buffer) : 1'b0;
    
    // MAIN RX STATE MACHINE
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state                <= IDLE_STATE;
            transaction_start_toggle <= 1'b0;
            fifo_wr_reg             <= 1'b0;
            frame_error_reg         <= 1'b0;
            timeout_error_reg       <= 1'b0;
            timeout_counter         <= 'b0;
        end else begin
            // Defaults
            fifo_wr_reg       <= 1'b0;
            frame_error_reg   <= 1'b0;
            timeout_error_reg <= 1'b0;
            case (rx_state)
                IDLE_STATE: begin
                    timeout_counter <= 'b0; 
                    if (start_bit_detected) begin
                        rx_state <= RECV_STATE;
                        transaction_start_toggle <= ~transaction_start_toggle;
                    end
                end
                RECV_STATE: begin
                    // Reset timeout counter on any activity
                    if (baudrate_clk_en) begin
                        timeout_counter <= 'b0;
                    end else if (timeout_counter >= TIMEOUT_CYCLES) begin
                        // Timeout occurred
                        rx_state <= ERROR_STATE;
                        timeout_error_reg <= 1'b1;
                        transaction_stop_toggle <= transaction_start_toggle;
                    end else begin
                        timeout_counter <= timeout_counter + 1'b1;
                    end
                    // Transaction complete
                    if (!transaction_en) begin
                        rx_state <= IDLE_STATE;
                        fifo_wr_reg <= 1'b1;
                        frame_error_reg <= parity_error | stop_bit_error;
                    end
                end
                ERROR_STATE: begin
                    rx_state <= IDLE_STATE;
                end
                default: begin
                    rx_state <= IDLE_STATE;
                end
            endcase
        end
    end

    // TRANSACTION STATE MACHINE (Bit-level)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            transaction_state     <= START_BIT_STATE;
            transaction_stop_toggle <= 1'b0;
            parity_buffer         <= 1'b0;
            rx_buffer             <= 'b0;
            data_counter          <= 'b0;
            stop_bit_counter      <= STOP_BIT_SELECT - 1;
            stop_bit_error        <= 1'b0;
        end else begin
            // Default
            stop_bit_error <= 1'b0;
            if (rx_state == RECV_STATE) begin
                if (baudrate_clk_en && sample_enable) begin
                    case (transaction_state)
                        START_BIT_STATE: begin
                            if (rx_stable == 1'b0) begin
                                transaction_state <= DATA_STATE;
                                data_counter <= 'b0;
                            end else begin
                                // False start bit
                                transaction_state <= START_BIT_STATE;
                                transaction_stop_toggle <= transaction_start_toggle;
                            end
                        end
                        DATA_STATE: begin
                            rx_buffer[data_counter] <= rx_stable; 
                            if (data_counter == (DATA_WIDTH_SELECT - 1)) begin
                                if (PARITY_SELECT != 0) begin
                                    transaction_state <= PARITY_STATE;
                                end else begin
                                    transaction_state <= STOP_STATE;
                                    stop_bit_counter <= STOP_BIT_SELECT - 1;
                                end
                            end else begin
                                data_counter <= data_counter + 1'b1;
                            end
                        end
                        PARITY_STATE: begin
                            parity_buffer <= rx_stable;
                            transaction_state <= STOP_STATE;
                            stop_bit_counter <= STOP_BIT_SELECT - 1;
                        end
                        STOP_STATE: begin
                            if (rx_stable == 1'b1) begin
                                if (stop_bit_counter == 'b0) begin
                                    transaction_state <= START_BIT_STATE;
                                    transaction_stop_toggle <= transaction_start_toggle;
                                end else begin
                                    stop_bit_counter <= stop_bit_counter - 1'b1;
                                end
                            end else begin
                                // Invalid stop bit
                                transaction_state <= START_BIT_STATE;
                                transaction_stop_toggle <= transaction_start_toggle;
                                stop_bit_error <= 1'b1;
                            end
                        end
                        default: begin
                            transaction_state <= START_BIT_STATE;
                        end
                    endcase
                end
            end else if (rx_state == IDLE_STATE) begin
                transaction_state <= START_BIT_STATE;
                transaction_stop_toggle <= transaction_start_toggle;
                data_counter <= 'b0;
                stop_bit_counter <= STOP_BIT_SELECT - 1;
            end
        end
    end

endmodule