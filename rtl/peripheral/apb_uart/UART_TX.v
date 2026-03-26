`timescale 1ns / 1ps

module uart_tx #(
    parameter integer DATA_WIDTH   = 8,
    parameter integer PARITY_TYPE  = 1,      // 0: none, 1: even, 2: odd
    parameter integer STOP_BITS    = 2       // 1, 2
) (
    // Clock and Reset
    input  wire                     clk,
    input  wire                     rst_n,
    
    // CPU Interface
    input  wire [DATA_WIDTH-1:0]    data_i,
    input  wire                     data_valid_i,
    output reg                      ready_o,
    
    // Baud Rate Interface
    input  wire                     baudrate_clk_en,
    output reg                      transaction_en,
    
    // UART Physical Interface
    output reg                      TX,
    
    // Status
    output wire                     tx_busy
);

    // STATE DEFINITIONS
    localparam [2:0] IDLE_STATE      = 3'b000;
    localparam [2:0] START_BIT_STATE = 3'b001;
    localparam [2:0] DATA_STATE      = 3'b010;
    localparam [2:0] PARITY_STATE    = 3'b011;
    localparam [2:0] STOP_BIT_STATE  = 3'b100;
    
    // INTERNAL REGISTERS
    reg [2:0] tx_state           = IDLE_STATE;
    reg [DATA_WIDTH-1:0] tx_buffer = 'b0;
    reg [3:0] bit_counter       = 'b0;
    reg [1:0] stop_bit_counter  = 'b0;
    
    // PARITY CALCULATION
    wire parity_bit_even = ^tx_buffer;
    wire parity_bit_odd  = ~parity_bit_even;
    
    wire parity_bit;
    
    generate
        if (PARITY_TYPE == 1) begin
            assign parity_bit = parity_bit_even;      // Even parity
        end else if (PARITY_TYPE == 2) begin
            assign parity_bit = parity_bit_odd;       // Odd parity
        end else begin
            assign parity_bit = 1'b0;                 // No parity
        end
    endgenerate
    
    // STATUS SIGNALS
    assign tx_busy = (tx_state != IDLE_STATE);
    
    // TRANSMIT STATE MACHINE (1x baud rate - standard UART TX)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state          <= IDLE_STATE;
            ready_o           <= 1'b1;
            transaction_en    <= 1'b0;
            TX                <= 1'b1;
            tx_buffer         <= 'b0;
            bit_counter       <= 'b0;
            stop_bit_counter  <= 'b0;
        end else begin
            case (tx_state)
                IDLE_STATE: begin
                    TX <= 1'b1;
                    transaction_en <= 1'b0;
                    
                    if (data_valid_i && ready_o) begin
                        tx_state       <= START_BIT_STATE;
                        tx_buffer      <= data_i;
                        ready_o        <= 1'b0;
                        transaction_en <= 1'b1;
                        TX             <= 1'b0;  // Start bit
                    end else begin
                        ready_o <= 1'b1;
                    end
                end
                
                START_BIT_STATE: begin
                    TX <= 1'b0;  // Hold start bit
                    if (baudrate_clk_en) begin
                        tx_state <= DATA_STATE;
                        bit_counter <= 'b0;
                    end
                end
                
                DATA_STATE: begin
                    TX <= tx_buffer[bit_counter];  // Output current bit
                    if (baudrate_clk_en) begin
                        if (bit_counter == (DATA_WIDTH - 1)) begin
                            if (PARITY_TYPE != 0) begin
                                tx_state <= PARITY_STATE;
                            end else begin
                                tx_state <= STOP_BIT_STATE;
                                stop_bit_counter <= STOP_BITS - 1;
                            end
                        end else begin
                            bit_counter <= bit_counter + 1'b1;
                        end
                    end
                end
                
                PARITY_STATE: begin
                    TX <= parity_bit;  // Hold parity bit
                    if (baudrate_clk_en) begin
                        tx_state <= STOP_BIT_STATE;
                        stop_bit_counter <= STOP_BITS - 1;
                    end
                end
                
                STOP_BIT_STATE: begin
                    TX <= 1'b1;  // Stop bit
                    if (baudrate_clk_en) begin
                        if (stop_bit_counter == 'b0) begin
                            tx_state       <= IDLE_STATE;
                            transaction_en <= 1'b0;
                            ready_o        <= 1'b1;
                        end else begin
                            stop_bit_counter <= stop_bit_counter - 1'b1;
                        end
                    end
                end
                
                default: begin
                    tx_state <= IDLE_STATE;
                end
            endcase
        end
    end

endmodule