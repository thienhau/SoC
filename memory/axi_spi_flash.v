`timescale 1ns / 1ps

module axi_spi_flash (
    input  wire        aclk,
    input  wire        aresetn,

    // --- AXI4 Read Interface ---
    input  wire [3:0]  arid,
    input  wire [31:0] araddr,
    input  wire [7:0]  arlen,
    input  wire [2:0]  arsize,
    input  wire [1:0]  arburst,
    input  wire        arvalid,
    output reg         arready,

    output reg  [3:0]  rid,
    output reg  [31:0] rdata,
    output reg  [1:0]  rresp,
    output reg         rlast,
    output reg         rvalid,
    input  wire        rready,

    // --- SPI Physical Interface ---
    output reg         spi_cs_n,
    output reg         spi_sck,
    output reg         spi_mosi,
    input  wire        spi_miso
);

    localparam IDLE  = 3'd0;
    localparam SETUP = 3'd1;
    localparam SHIFT = 3'd2;
    localparam DONE  = 3'd3;

    reg [2:0]  state;
    reg [63:0] shift_reg; 
    reg [6:0]  bit_cnt;
    reg        sck_en;
    reg [3:0]  latched_id;

    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            state      <= IDLE;
            arready    <= 1'b1;
            rvalid     <= 1'b0;
            rlast      <= 1'b0;
            rresp      <= 2'b00;
            spi_cs_n   <= 1'b1;
            spi_sck    <= 1'b0;
            spi_mosi   <= 1'b0;
            bit_cnt    <= 7'd63;
            sck_en     <= 1'b0;
            latched_id <= 4'b0000;
        end else begin
            case (state)
                IDLE: begin
                    spi_cs_n <= 1'b1;
                    spi_sck  <= 1'b0;
                    if (rvalid && rready) begin
                        rvalid <= 1'b0;
                        rlast  <= 1'b0;
                    end
                    if (arvalid && arready) begin
                        arready    <= 1'b0;
                        latched_id <= arid; // Lưu lại ID để trả về chính xác
                        // Lệnh 0x03 (Normal Read) + 24-bit Address vật lý
                        shift_reg[63:56] <= 8'h03;
                        shift_reg[55:32] <= araddr[23:0];
                        shift_reg[31:0]  <= 32'h00000000;
                        bit_cnt          <= 7'd63;
                        state            <= SETUP;
                    end
                end

                SETUP: begin
                    spi_cs_n <= 1'b0;
                    sck_en   <= 1'b0;
                    state    <= SHIFT;
                end

                SHIFT: begin
                    if (sck_en == 1'b0) begin
                        spi_sck  <= 1'b0;
                        spi_mosi <= shift_reg[bit_cnt];
                        sck_en   <= 1'b1;
                    end else begin
                        spi_sck <= 1'b1;
                        if (bit_cnt < 7'd32) begin 
                            shift_reg[bit_cnt] <= spi_miso;
                        end
                        sck_en <= 1'b0;
                        
                        if (bit_cnt == 7'd0) begin
                            state <= DONE;
                        end else begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                DONE: begin
                    spi_cs_n <= 1'b1;
                    spi_sck  <= 1'b0;
                    rid      <= latched_id; // Trả đúng ID yêu cầu
                    rdata    <= shift_reg[31:0];
                    rvalid   <= 1'b1;
                    rlast    <= 1'b1; // Báo hiệu đã đọc xong 1 khối 32-bit
                    
                    if (rvalid && rready) begin
                        rvalid  <= 1'b0;
                        rlast   <= 1'b0;
                        arready <= 1'b1;
                        state   <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule