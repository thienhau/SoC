module syscon_apb_slave #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32
)(
    input  wire                   pclk,
    input  wire                   presetn,
    
    input  wire [ADDR_WIDTH-1:0]  paddr,
    input  wire                   psel,
    input  wire                   penable,
    input  wire                   pwrite,
    input  wire [DATA_WIDTH-1:0]  pwdata,
    
    output reg                    pready,
    output reg  [DATA_WIDTH-1:0]  prdata,
    output reg                    pslverr,

    // Tín hiệu xuất ra cho CPU
    output reg  [15:0]            o_reset_vector
);

    wire apb_access = psel && penable;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            pready         <= 1'b0;
            prdata         <= 32'b0;
            pslverr        <= 1'b0;
            o_reset_vector <= 16'h1000; // Mặc định cứng chạy từ ROM
        end else begin
            pready  <= apb_access;
            pslverr <= 1'b0;
            
            if (psel && !penable) begin
                // Pha Setup: Lấy sẵn dữ liệu nếu là lệnh đọc
                if (!pwrite && (paddr[11:0] == 12'h000)) begin
                    prdata <= {16'b0, o_reset_vector};
                end else begin
                    prdata <= 32'h0;
                end
            end
            else if (apb_access && pready) begin
                // Pha Access: Ghi dữ liệu
                if (pwrite && (paddr[11:0] == 12'h000)) begin
                    o_reset_vector <= pwdata[15:0];
                end
            end
        end
    end

endmodule