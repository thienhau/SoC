module onehot_decoder
#(
    parameter INPUT_W  = 3, // includes valid bit (MSB)
    parameter OUTPUT_W = 4
)
(
    input  [INPUT_W-1:0]  i,
    output [OUTPUT_W-1:0] o
);
    // MSB of input is active-low valid bit, remaining bits are binary index
    localparam IDX_W = INPUT_W - 1;

    wire             valid_n = i[INPUT_W-1];
    wire [IDX_W-1:0] index   = i[IDX_W-1:0];

    reg [OUTPUT_W-1:0] decoded;
    integer idx;

    always @(*) begin
        decoded = {OUTPUT_W{1'b0}};
        if (!valid_n) begin
            for (idx = 0; idx < OUTPUT_W; idx = idx + 1) begin
                if (index == idx[IDX_W-1:0])
                    decoded[idx] = 1'b1;
            end
        end
    end

    assign o = decoded;

endmodule
