`timescale 1ns / 1ps

module apb_ascon (
    input  wire        PCLK,
    input  wire        PRESETn,
    input  wire        PSEL,
    input  wire        PENABLE,
    input  wire        PWRITE,
    input  wire [11:0] PADDR,
    input  wire [31:0] PWDATA,
    output wire [31:0] PRDATA,
    output wire        PREADY,
    output wire        PSLVERR,
    output wire        ascon_irq
);

    assign PREADY  = 1'b1;
    assign PSLVERR = 1'b0;

    // Registers Map
    reg [31:0] ctrl_reg; 
    // [0]: Start
    // [1]: Mode (0=AEAD, 1=Hash)
    // [2]: TRNG Reset
    // [3]: TRNG Enable
    // [4]: Data Valid Pulse (Auto-clear)
    // [7:5]: Data Command (0=AD, 1=AD_LAST, 2=PT, 3=PT_LAST, 4=HASH, 5=HASH_LAST)

    reg [31:0] key_reg_0, key_reg_1, key_reg_2, key_reg_3;
    reg [31:0] nonce_reg_0, nonce_reg_1, nonce_reg_2, nonce_reg_3;
    reg [31:0] data_in_reg_0, data_in_reg_1;
    
    wire [63:0]  ascon_data_out;
    wire [127:0] ascon_tag_out;
    wire [255:0] ascon_hash_out;
    wire [127:0] trng_rand_out;
    
    wire ascon_ready, ascon_done, trng_valid;
    wire write_en = PSEL & PENABLE & PWRITE;
    
    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            ctrl_reg <= 32'h0;
            key_reg_0 <= 32'h0; key_reg_1 <= 32'h0; key_reg_2 <= 32'h0; key_reg_3 <= 32'h0;
            nonce_reg_0 <= 32'h0; nonce_reg_1 <= 32'h0; nonce_reg_2 <= 32'h0; nonce_reg_3 <= 32'h0;
            data_in_reg_0 <= 32'h0; data_in_reg_1 <= 32'h0;
        end else if (write_en) begin
            case (PADDR[7:0])
                8'h00: ctrl_reg <= PWDATA; 
                8'h10: key_reg_0 <= PWDATA;
                8'h14: key_reg_1 <= PWDATA;
                8'h18: key_reg_2 <= PWDATA;
                8'h1C: key_reg_3 <= PWDATA;
                8'h20: nonce_reg_0 <= PWDATA;
                8'h24: nonce_reg_1 <= PWDATA;
                8'h28: nonce_reg_2 <= PWDATA;
                8'h2C: nonce_reg_3 <= PWDATA;
                8'h30: data_in_reg_0 <= PWDATA;
                8'h34: data_in_reg_1 <= PWDATA;
                default: ;
            endcase
        end else begin
            ctrl_reg[0] <= 1'b0; // Auto-clear Start pulse
            ctrl_reg[4] <= 1'b0; // Auto-clear Data Valid pulse
        end
    end

    // Read MUX
    reg [31:0] prdata_reg;
    always @(*) begin
        case (PADDR[7:0])
            8'h00: prdata_reg = ctrl_reg;
            8'h04: prdata_reg = {29'h0, trng_valid, ascon_done, ascon_ready};
            8'h40: prdata_reg = ascon_data_out[63:32];
            8'h44: prdata_reg = ascon_data_out[31:0];
            
            8'h50: prdata_reg = ctrl_reg[1] ? ascon_hash_out[255:224] : ascon_tag_out[127:96];
            8'h54: prdata_reg = ctrl_reg[1] ? ascon_hash_out[223:192] : ascon_tag_out[95:64];
            8'h58: prdata_reg = ctrl_reg[1] ? ascon_hash_out[191:160] : ascon_tag_out[63:32];
            8'h5C: prdata_reg = ctrl_reg[1] ? ascon_hash_out[159:128] : ascon_tag_out[31:0];
            8'h60: prdata_reg = ascon_hash_out[127:96];
            8'h64: prdata_reg = ascon_hash_out[95:64];
            8'h68: prdata_reg = ascon_hash_out[63:32];
            8'h6C: prdata_reg = ascon_hash_out[31:0];
            
            8'h70: prdata_reg = trng_rand_out[127:96];
            8'h74: prdata_reg = trng_rand_out[95:64];
            8'h78: prdata_reg = trng_rand_out[63:32];
            8'h7C: prdata_reg = trng_rand_out[31:0];
            default: prdata_reg = 32'h0;
        endcase
    end
    
    assign PRDATA = (PSEL & PENABLE & ~PWRITE) ? prdata_reg : 32'h0;
    assign ascon_irq = ascon_done;

    wire [127:0] full_key   = {key_reg_0, key_reg_1, key_reg_2, key_reg_3};
    wire [127:0] full_nonce = {nonce_reg_0, nonce_reg_1, nonce_reg_2, nonce_reg_3};
    wire [63:0]  full_data  = {data_in_reg_0, data_in_reg_1};

    trng_128b u_trng (
        .clock    (PCLK), 
        .reset    (ctrl_reg[2]), 
        .enable   (ctrl_reg[3]), 
        .valid    (trng_valid), 
        .rand_out (trng_rand_out)
    );

    ascon_core u_ascon (
        .clk        (PCLK), 
        .rst_n      (PRESETn), 
        .start      (ctrl_reg[0]), 
        .mode       (ctrl_reg[1]), 
        .key_in     (full_key), 
        .nonce_in   (full_nonce), 
        .data_in    (full_data),
        .data_valid (ctrl_reg[4]),     // Tín hiệu valid mới
        .data_cmd   (ctrl_reg[7:5]),   // Phân loại khối dữ liệu
        .data_out   (ascon_data_out), 
        .tag_out    (ascon_tag_out),
        .hash_out   (ascon_hash_out),
        .ready      (ascon_ready), 
        .done       (ascon_done)
    );

endmodule