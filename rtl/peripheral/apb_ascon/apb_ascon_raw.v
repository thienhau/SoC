`timescale 1ns / 1ps

module apb_ascon_raw #(
    parameter USE_MASKED_CORE = 0, // 0: v1 Unmasked, 1: v2 Masked
    parameter USE_HYBRID_TRNG = 1  // 0: v1 Pure RO,  1: v2 Hybrid PRNG
)(
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
    output wire        interrupt
);

    assign PREADY  = 1'b1;
    assign PSLVERR = 1'b0;

    // Registers
    reg [31:0] ctrl_reg;
    reg [31:0] key_reg[0:3];
    reg [31:0] nonce_reg[0:3];
    reg [31:0] data_in_reg[0:1];
    
    wire [63:0] ascon_data_out;
    wire [255:0] ascon_tag_out;
    wire [127:0] trng_entropy;
    wire ascon_ready, ascon_done, trng_ready;

    wire write_en = PSEL & PENABLE & PWRITE;
    
    always @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            ctrl_reg <= 32'h0;
            key_reg[0] <= 32'h0; key_reg[1] <= 32'h0; key_reg[2] <= 32'h0; key_reg[3] <= 32'h0;
            nonce_reg[0] <= 32'h0; nonce_reg[1] <= 32'h0; nonce_reg[2] <= 32'h0; nonce_reg[3] <= 32'h0;
            data_in_reg[0] <= 32'h0; data_in_reg[1] <= 32'h0;
        end else if (write_en) begin
            case (PADDR[7:0])
                8'h00: ctrl_reg <= PWDATA; // [0]: Start, [2:1]: Mode, [5]: TRNG_EN
                8'h10: key_reg[0] <= PWDATA;
                8'h14: key_reg[1] <= PWDATA;
                8'h18: key_reg[2] <= PWDATA;
                8'h1C: key_reg[3] <= PWDATA;
                8'h20: nonce_reg[0] <= PWDATA;
                8'h24: nonce_reg[1] <= PWDATA;
                8'h28: nonce_reg[2] <= PWDATA;
                8'h2C: nonce_reg[3] <= PWDATA;
                8'h30: data_in_reg[0] <= PWDATA;
                8'h34: data_in_reg[1] <= PWDATA;
                default: ;
            endcase
        end else begin
            ctrl_reg[0] <= 1'b0; // Auto-clear Start
        end
    end

    // Read MUX
    reg [31:0] prdata_reg;
    always @(*) begin
        case (PADDR[7:0])
            8'h00: prdata_reg = ctrl_reg;
            8'h04: prdata_reg = {29'h0, trng_ready, ascon_done, ascon_ready};
            8'h40: prdata_reg = ascon_data_out[63:32];
            8'h44: prdata_reg = ascon_data_out[31:0];
            8'h50: prdata_reg = ascon_tag_out[255:224];
            8'h54: prdata_reg = ascon_tag_out[223:192];
            8'h58: prdata_reg = ascon_tag_out[191:160];
            8'h5C: prdata_reg = ascon_tag_out[159:128];
            8'h60: prdata_reg = trng_entropy[127:96];
            8'h64: prdata_reg = trng_entropy[95:64];
            8'h68: prdata_reg = trng_entropy[63:32];
            8'h6C: prdata_reg = trng_entropy[31:0];
            default: prdata_reg = 32'h0;
        endcase
    end
    assign PRDATA = (PSEL & PENABLE & ~PWRITE) ? prdata_reg : 32'h0;
    assign interrupt = ascon_done;

    wire [127:0] full_key   = {key_reg[0], key_reg[1], key_reg[2], key_reg[3]};
    wire [127:0] full_nonce = {nonce_reg[0], nonce_reg[1], nonce_reg[2], nonce_reg[3]};
    wire [63:0]  full_data  = {data_in_reg[0], data_in_reg[1]};

    // TRNG Instantiation
    generate
        if (USE_HYBRID_TRNG == 1) begin : gen_hybrid_trng
            trng_v2_hybrid u_trng (
                .clk(PCLK), .rst_n(PRESETn), .enable(ctrl_reg[5]),
                .rand_out(trng_entropy), .ready(trng_ready)
            );
        end else begin : gen_pure_trng
            trng_v1_pure u_trng (
                .clk(PCLK), .rst_n(PRESETn), .enable(ctrl_reg[5]),
                .rand_out(trng_entropy), .ready(trng_ready)
            );
        end
    endgenerate

    // ASCON Core Instantiation
    generate
        if (USE_MASKED_CORE == 1) begin : gen_masked_core
            ascon_core_v2_masked u_ascon (
                .clk(PCLK), .rst_n(PRESETn), .start(ctrl_reg[0]), .mode(ctrl_reg[2:1]),
                .key_in(full_key), .nonce_in(full_nonce), .data_in(full_data),
                .entropy_128b(trng_entropy),
                .data_out(ascon_data_out), .tag_out(ascon_tag_out),
                .ready(ascon_ready), .done(ascon_done)
            );
        end else begin : gen_unmasked_core
            ascon_core_v1_unmasked u_ascon (
                .clk(PCLK), .rst_n(PRESETn), .start(ctrl_reg[0]), .mode(ctrl_reg[2:1]),
                .key_in(full_key), .nonce_in(full_nonce), .data_in(full_data),
                .data_out(ascon_data_out), .tag_out(ascon_tag_out),
                .ready(ascon_ready), .done(ascon_done)
            );
        end
    endgenerate
endmodule