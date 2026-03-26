`timescale 1ns / 1ps

module asyn_fifo #(
    parameter integer ASFIFO_TYPE     = 0,            // 0: Normal, 1: Registered
    parameter integer DATA_WIDTH      = 8,
    parameter integer FIFO_DEPTH      = 32,
    parameter integer NUM_SYNC_FF     = 2,            // Set to 0 for same-clock operation
    parameter integer ALMOST_FULL_TH  = 30,
    parameter integer ALMOST_EMPTY_TH = 2
) (
    input  wire                     clk_wr_domain,
    input  wire                     clk_rd_domain,
    input  wire [DATA_WIDTH-1:0]    data_i,
    output wire [DATA_WIDTH-1:0]    data_o,
    input  wire                     wr_valid_i,
    input  wire                     rd_valid_i,
    output wire                     empty_o,
    output wire                     full_o,
    output wire                     wr_ready_o,
    output wire                     rd_ready_o,
    output wire                     almost_empty_o,
    output wire                     almost_full_o,
    input  wire                     rst_n
);

    // LOCAL PARAMETERS
    localparam integer ADDR_WIDTH = (FIFO_DEPTH > 1) ? $clog2(FIFO_DEPTH) : 1;
    localparam integer ADDR_OVF_WIDTH = ADDR_WIDTH + 1;
    
    // INTERNAL SIGNALS
    // Memory array
    (* ram_style = "distributed" *) reg [DATA_WIDTH-1:0] buffer [0:FIFO_DEPTH-1];
    
    // Gray code pointers
    reg [ADDR_OVF_WIDTH-1:0] wr_ptr_bin  = 'b0;
    reg [ADDR_OVF_WIDTH-1:0] wr_ptr_gray = 'b0;
    reg [ADDR_OVF_WIDTH-1:0] rd_ptr_bin  = 'b0;
    reg [ADDR_OVF_WIDTH-1:0] rd_ptr_gray = 'b0;
    
    // Synchronized pointers
    wire [ADDR_OVF_WIDTH-1:0] wr_addr_gray_sync;
    wire [ADDR_OVF_WIDTH-1:0] rd_addr_gray_sync;
    wire [ADDR_OVF_WIDTH-1:0] wr_addr_bin_sync;
    wire [ADDR_OVF_WIDTH-1:0] rd_addr_bin_sync;
    
    // Status signals
    wire full_comb;
    wire empty_comb;
    wire wr_handshake;
    wire rd_handshake;
    
    // SYNCHRONIZERS (Cross-clock domain) or BYPASS (Same-clock)
    generate
        if (NUM_SYNC_FF > 0) begin : gen_async_sync
            // Async FIFO - use synchronizers
            sync_cell #(
                .WIDTH      (ADDR_OVF_WIDTH),
                .NUM_STAGES (NUM_SYNC_FF)
            ) rd_ptr_sync (
                .clk      (clk_wr_domain),
                .rst_n    (rst_n),
                .async_i  (rd_ptr_gray),
                .sync_o   (rd_addr_gray_sync)
            );
            
            sync_cell #(
                .WIDTH      (ADDR_OVF_WIDTH),
                .NUM_STAGES (NUM_SYNC_FF)
            ) wr_ptr_sync (
                .clk      (clk_rd_domain),
                .rst_n    (rst_n),
                .async_i  (wr_ptr_gray),
                .sync_o   (wr_addr_gray_sync)
            );
        end else begin : gen_sync_bypass
            // Same-clock FIFO - bypass synchronizers (no delay)
            assign rd_addr_gray_sync = rd_ptr_gray;
            assign wr_addr_gray_sync = wr_ptr_gray;
        end
    endgenerate
    
    // GRAY ⇔ BINARY CONVERSION FUNCTIONS
    function automatic [ADDR_OVF_WIDTH-1:0] bin2gray (
        input [ADDR_OVF_WIDTH-1:0] bin
    );
        bin2gray = bin ^ (bin >> 1);
    endfunction
    
    function automatic [ADDR_OVF_WIDTH-1:0] gray2bin (
        input [ADDR_OVF_WIDTH-1:0] gray
    );
        reg [ADDR_OVF_WIDTH-1:0] result;
        integer i;
    begin
        result[ADDR_OVF_WIDTH-1] = gray[ADDR_OVF_WIDTH-1];
        for (i = ADDR_OVF_WIDTH-2; i >= 0; i = i - 1) begin
            result[i] = result[i+1] ^ gray[i];
        end
        gray2bin = result;
    end
    endfunction
    
    // COMBINATIONAL LOGIC (Common for both types)
    
    // For same-clock (NUM_SYNC_FF=0): use binary pointers directly (no gray delay)
    // For async (NUM_SYNC_FF>0): use synchronized gray-converted pointers
    generate
        if (NUM_SYNC_FF == 0) begin : gen_sync_logic
            // Same-clock: bypass gray code completely, use binary pointers directly
            assign rd_addr_bin_sync = rd_ptr_bin;  // Direct!
            assign wr_addr_bin_sync = wr_ptr_bin;  // Direct!
            
            assign full_comb  = ((wr_ptr_bin[ADDR_WIDTH-1:0] == rd_ptr_bin[ADDR_WIDTH-1:0]) && 
                                (wr_ptr_bin[ADDR_WIDTH] != rd_ptr_bin[ADDR_WIDTH]));
            assign empty_comb = (wr_ptr_bin == rd_ptr_bin);
        end else begin : gen_async_logic
            // Async: use synchronized gray-converted pointers (safe for CDC)
            assign rd_addr_bin_sync = gray2bin(rd_addr_gray_sync);
            assign wr_addr_bin_sync = gray2bin(wr_addr_gray_sync);
            
            assign full_comb  = ((wr_ptr_bin[ADDR_WIDTH-1:0] == rd_addr_bin_sync[ADDR_WIDTH-1:0]) && 
                                (wr_ptr_bin[ADDR_WIDTH] != rd_addr_bin_sync[ADDR_WIDTH]));
            assign empty_comb = (wr_addr_bin_sync == rd_ptr_bin);
        end
    endgenerate

    assign wr_handshake = wr_valid_i && wr_ready_o;
    assign rd_handshake = rd_valid_i && rd_ready_o;
    
    // NORMAL FIFO (Type 0)
    generate
        if (ASFIFO_TYPE == 0) begin : NORMAL_FIFO
            
            // FIFO count calculations
            wire [ADDR_OVF_WIDTH-1:0] fifo_count_wr;
            wire [ADDR_OVF_WIDTH-1:0] fifo_count_rd;
            
            assign fifo_count_wr = wr_ptr_bin - rd_addr_bin_sync;
            assign fifo_count_rd = wr_addr_bin_sync - rd_ptr_bin;
            
            // Output assignments
            assign full_o  = full_comb;
            assign empty_o = empty_comb;
            assign wr_ready_o = !full_comb;
            assign rd_ready_o = !empty_comb;
            assign data_o = buffer[rd_ptr_bin[ADDR_WIDTH-1:0]];
            
            assign almost_full_o  = (fifo_count_wr >= ALMOST_FULL_TH);
            assign almost_empty_o = (fifo_count_rd <= ALMOST_EMPTY_TH);
            
            // WRITE DOMAIN LOGIC
            always @(posedge clk_wr_domain or negedge rst_n) begin
                if (!rst_n) begin
                    wr_ptr_bin  <= 'b0;
                    wr_ptr_gray <= 'b0;
                end else if (wr_handshake) begin
                    // Write data to memory
                    buffer[wr_ptr_bin[ADDR_WIDTH-1:0]] <= data_i;
                    // Update pointers
                    wr_ptr_bin  <= wr_ptr_bin + 1'b1;
                    wr_ptr_gray <= bin2gray(wr_ptr_bin + 1'b1);
                end
            end
            
            // READ DOMAIN LOGIC
            always @(posedge clk_rd_domain or negedge rst_n) begin
                if (!rst_n) begin
                    rd_ptr_bin  <= 'b0;
                    rd_ptr_gray <= 'b0;
                end else if (rd_handshake) begin
                    rd_ptr_bin  <= rd_ptr_bin + 1'b1;
                    rd_ptr_gray <= bin2gray(rd_ptr_bin + 1'b1);
                end
            end
            
        end 
        // REGISTERED FIFO (Type 1) - Better timing
        else begin : REGISTERED_FIFO
            
            // Registered outputs
            reg full_reg          = 1'b0;
            reg empty_reg         = 1'b1;
            reg wr_ready_reg      = 1'b1;
            wire rd_ready_reg;   // Combinational for immediate response
            reg almost_full_reg   = 1'b0;
            reg almost_empty_reg  = 1'b1;
            reg [DATA_WIDTH-1:0] data_out_reg = 'b0;
            
            // FIFO count registers
            reg [ADDR_OVF_WIDTH-1:0] fifo_count_wr_reg = 'b0;
            reg [ADDR_OVF_WIDTH-1:0] fifo_count_rd_reg = 'b0;
            
            // Output assignments
            assign full_o  = full_reg;
            assign empty_o = empty_reg;
            assign wr_ready_o = wr_ready_reg;
            assign rd_ready_o = rd_ready_reg;
            assign almost_full_o = almost_full_reg;
            assign almost_empty_o = almost_empty_reg;
            assign data_o = data_out_reg;
            
            // WRITE DOMAIN LOGIC
            always @(posedge clk_wr_domain or negedge rst_n) begin
                if (!rst_n) begin
                    wr_ptr_bin        <= 'b0;
                    wr_ptr_gray       <= 'b0;
                    full_reg          <= 1'b0;
                    almost_full_reg   <= 1'b0;
                    wr_ready_reg      <= 1'b1;
                    fifo_count_wr_reg <= 'b0;
                end else begin
                    // Update memory and pointer on write
                    if (wr_handshake && !full_comb) begin
                        buffer[wr_ptr_bin[ADDR_WIDTH-1:0]] <= data_i;
                        wr_ptr_bin  <= wr_ptr_bin + 1'b1;
                        wr_ptr_gray <= bin2gray(wr_ptr_bin + 1'b1);
                    end
                    
                    // Calculate FIFO count
                    fifo_count_wr_reg <= wr_ptr_bin - rd_addr_bin_sync;
                    
                    // Update status flags
                    full_reg <= full_comb;
                    almost_full_reg <= (fifo_count_wr_reg >= ALMOST_FULL_TH);
                    wr_ready_reg <= !full_comb;
                end
            end
            
            // READ DOMAIN LOGIC
            
            // Next read pointer for prefetch calculation
            wire [ADDR_OVF_WIDTH-1:0] rd_ptr_bin_next = rd_ptr_bin + 1'b1;
            
            // Empty after current read (combinational for immediate response)
            wire empty_after_read = (wr_addr_bin_sync == rd_ptr_bin_next);
            
            // rd_ready: Use registered empty_reg for stable output
            // This avoids combinational loop (rd_handshake depends on rd_ready)
            assign rd_ready_reg = !empty_reg;
            
            always @(posedge clk_rd_domain or negedge rst_n) begin
                if (!rst_n) begin
                    rd_ptr_bin        <= 'b0;
                    rd_ptr_gray       <= 'b0;
                    data_out_reg      <= 'b0;
                    empty_reg         <= 1'b1;
                    almost_empty_reg  <= 1'b1;
                    fifo_count_rd_reg <= 'b0;
                end else begin
                    // Calculate FIFO count
                    fifo_count_rd_reg <= wr_addr_bin_sync - rd_ptr_bin;
                    
                    // Read pointer update
                    if (rd_handshake && !empty_comb) begin
                        rd_ptr_bin  <= rd_ptr_bin_next;
                        rd_ptr_gray <= bin2gray(rd_ptr_bin_next);
                        
                        // Update empty_reg immediately based on next state
                        empty_reg <= empty_after_read;
                    end else begin
                        // Normal empty update
                        empty_reg <= empty_comb;
                    end
                    
                    almost_empty_reg <= (fifo_count_rd_reg <= ALMOST_EMPTY_TH);
                    
                    // Data output register - FIRST WORD FALL THROUGH style
                    // Load new data when FIFO becomes non-empty
                    if (empty_reg && !empty_comb) begin
                        // FIFO just became non-empty - load first word
                        data_out_reg <= buffer[rd_ptr_bin[ADDR_WIDTH-1:0]];
                    end else if (rd_handshake && !empty_comb && !empty_after_read) begin
                        // After read, prefetch next word (only if more data exists)
                        data_out_reg <= buffer[rd_ptr_bin_next[ADDR_WIDTH-1:0]];
                    end
                    // If empty_after_read, keep current data_out_reg (don't prefetch garbage)
                end
            end
        end
    endgenerate

endmodule